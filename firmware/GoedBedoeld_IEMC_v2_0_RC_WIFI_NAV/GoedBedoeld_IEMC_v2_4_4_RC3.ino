/*
 Goed Bedoeld - IEM UI hardwaretest v0.7b TOUCH RESET
 Freenove FNK0104N: ST77922 QSPI display, HW-040 rotary encoder.
 Kleurcorrectie ter controle: RGB565-bytewissel.
 Wi-Fi + X32 OSC state-machine v0.9. Geen LVGL of TFT sprite benodigd.

 HW-040: CLK=GPIO43, DT=GPIO44, SW=GPIO46,
         +=3.3V, GND=GND, SW/GPIO46 via ~6.7k naar 3.3V.
 GPIO46 is een STRAPPING-pin: uitsluitend voorlopige testbedrading.
 Bij flashproblemen eerst SW-pullup en zo nodig SW loshalen.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
// Read-only GitHub release check for the OTA prototype. NO FLASH/UPDATE SUPPORT.
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>
#include <Update.h>
#include <mbedtls/sha256.h>

#include <cstring>
#include <TFT_eSPI.h> // Levert de Freenove ST77922-headers/dependencies
#include "ST77922.h"
#include "ST77922_Touch.h"

static constexpr int W = 480, H = 320, TILE_H = 8;
static constexpr int CLK_PIN = 43, DT_PIN = 44, SW_PIN = 46;
static constexpr int CHANNELS = 6; // zichtbare kanaalstroken
static constexpr int TOTAL_CHANNELS = 32; // beschikbare X32-ingangen
static int selected = 0; // absolute kanaalindex, 0..31
static int windowStart = 0; // eerste kanaal in zesstrooksvenster
extern bool redraw;
// Personal order is a list of real X32 channel indices (0..31).
// The ordering and visibility never change the X32 channel state.
static uint8_t userOrder[TOTAL_CHANNELS];
static bool userVisible[TOTAL_CHANNELS];
static uint8_t shown[TOTAL_CHANNELS];
static int shownCount=TOTAL_CHANNELS;
static int setupSelected=0,setupTop=0;
static Preferences channelPrefs;
static int channelAt(int position){
  return position>=0 && position<shownCount ? (int)shown[position] : -1;
}
static void rebuildShown(){
  int previous=channelAt(selected);
  shownCount=0;
  for(int i=0;i<TOTAL_CHANNELS;i++)if(userVisible[userOrder[i]])shown[shownCount++]=userOrder[i];
  if(shownCount==0){userVisible[userOrder[0]]=true;shown[0]=userOrder[0];shownCount=1;}
  int next=-1;
  for(int i=0;i<shownCount;i++)if(shown[i]==previous)next=i;
  if(next<0)next=min(selected,shownCount-1);
  selected=next;
  windowStart=constrain(windowStart,0,max(0,shownCount-CHANNELS));
  if(selected<windowStart)windowStart=selected;
  if(selected>=windowStart+CHANNELS)windowStart=selected-CHANNELS+1;
}
static void loadChannels(){
  for(int i=0;i<TOTAL_CHANNELS;i++){userOrder[i]=i;userVisible[i]=true;shown[i]=i;}
  shownCount=TOTAL_CHANNELS;
  channelPrefs.begin("gb-channels",false);
  uint8_t order[TOTAL_CHANNELS],visibility[TOTAL_CHANNELS];
  bool valid=channelPrefs.getBytesLength("order")==sizeof(order) &&
             channelPrefs.getBytesLength("visible")==sizeof(visibility);
  if(valid){
    channelPrefs.getBytes("order",order,sizeof(order));
    channelPrefs.getBytes("visible",visibility,sizeof(visibility));
    bool used[TOTAL_CHANNELS]={false};
    for(int i=0;i<TOTAL_CHANNELS;i++){
      if(order[i]>=TOTAL_CHANNELS||used[order[i]]){valid=false;break;}
      used[order[i]]=true;
    }
    if(valid){
      for(int i=0;i<TOTAL_CHANNELS;i++){userOrder[i]=order[i];userVisible[i]=visibility[i]!=0;}
    }
  }
  rebuildShown();
  Serial.printf("Kanaalindeling: %s, %d zichtbaar\n",valid?"NVS gelezen":"standaard",shownCount);
}
static void saveChannels(){
  uint8_t flags[TOTAL_CHANNELS];
  for(int i=0;i<TOTAL_CHANNELS;i++)flags[i]=userVisible[i]?1:0;
  channelPrefs.putBytes("order",userOrder,sizeof(userOrder));
  channelPrefs.putBytes("visible",flags,sizeof(flags));
}
static void changeVisibility(){
  int ch=userOrder[setupSelected];
  if(userVisible[ch]&&shownCount==1)return;
  userVisible[ch]=!userVisible[ch];
  rebuildShown();saveChannels();redraw=true;
  Serial.printf("SETUP CH%02d %s\n",ch+1,userVisible[ch]?"ZICHTBAAR":"VERBORGEN");
}
static void moveSetupChannel(int dir){
  int to=setupSelected+dir;
  if(to<0||to>=TOTAL_CHANNELS)return;
  uint8_t tmp=userOrder[to];userOrder[to]=userOrder[setupSelected];userOrder[setupSelected]=tmp;
  setupSelected=to;
  if(setupSelected<setupTop)setupTop=setupSelected;
  if(setupSelected>=setupTop+7)setupTop=setupSelected-6;
  rebuildShown();saveChannels();redraw=true;
  Serial.printf("SETUP KANAALVOLGORDE: CH%02d naar positie %d\n",userOrder[setupSelected]+1,setupSelected+1);
}
static constexpr uint32_t LONG_MS = 3000, TIMEOUT_MS = 5000;
static constexpr uint32_t DEBOUNCE_MS = 35;


// Gedeeld met de UI-code; definitie staat verderop in de sketch.

// -- Freenove X32-communicatielaag v1.1 (6 live sends + meter stream) --
// Proefconfiguratie: bus 03, kanalen 01-06; veilig schrijven pas na volledige sync.
// v1.2f: profielen in aparte NVS-namespace; bestaande kanaal- en bus-NVS blijft intact.
static constexpr int WIFI_PROFILES=3;
static constexpr char SETUP_SSID[]="GB-IEM-SETUP";
static constexpr char SETUP_PASSWORD[]="GoedBedoeld32"; // alleen voor lokale test; wijzig voor gebruik op podium
static IPAddress X32_IP(192,168,4,1); // runtime-adres, niet meer vast in firmware
static Preferences netPrefs;
static String savedSsid[WIFI_PROFILES],savedPass[WIFI_PROFILES]; // NVS: passwords stored unencrypted; physical flash access is trusted
static uint8_t activeWifi=0;
static bool setupPortalActive=false; // compatibility guard; no AP or web server in v1.2j
static bool manualX32=false;
static uint32_t discoveryStarted=0;
static void loadNetworkSettings(){
  netPrefs.begin("gb-network",false);
  for(int i=0;i<WIFI_PROFILES;i++){
    savedSsid[i]=netPrefs.getString((String("s")+i).c_str(),"");
    savedPass[i]=netPrefs.getString((String("p")+i).c_str(),"");
  }
  if(savedSsid[0].isEmpty() && savedSsid[1].isEmpty() && savedSsid[2].isEmpty()){
    savedSsid[0]="GB-X32-SIM";
    savedPass[0]="GoedBedoeld32";
    netPrefs.putString("s0",savedSsid[0]);netPrefs.putString("p0",savedPass[0]);
  }
  activeWifi=netPrefs.getUChar("active",0);
  if(activeWifi>=WIFI_PROFILES||savedSsid[activeWifi].isEmpty()){
    for(int i=0;i<WIFI_PROFILES;i++)if(!savedSsid[i].isEmpty()){activeWifi=i;break;}
  }
  manualX32=netPrefs.getBool("fixed",false);
  String address=netPrefs.getString("mixer", "192.168.4.1");
  if(!X32_IP.fromString(address))X32_IP=IPAddress(192,168,4,1);
  Serial.printf("Netwerkprofiel %d: %s | mixer %s (%s)\n",(int)activeWifi+1,savedSsid[activeWifi].c_str(),X32_IP.toString().c_str(),manualX32?"handmatig":"automatisch");
}
static void connectSavedWifi(){
  if(setupPortalActive||savedSsid[activeWifi].isEmpty())return;
  WiFi.begin(savedSsid[activeWifi].c_str(),savedPass[activeWifi].c_str());
  Serial.printf("Wi-Fi verbinden met profiel %d: %s\n",(int)activeWifi+1,savedSsid[activeWifi].c_str());
}

// v1.2j: complete on-device Wi-Fi configuration with touch keyboard.
static constexpr int WIFI_SCAN_MAX=16;
static String scannedSsid[WIFI_SCAN_MAX];
static int32_t scannedRssi[WIFI_SCAN_MAX];
static bool scannedSecure[WIFI_SCAN_MAX];
static int scannedCount=0, wifiCursor=0, wifiTop=0;
static bool wifiScanning=false, wifiHaveResults=false;
static uint32_t wifiScanStarted=0;
// On-device touchscreen keyboard: no phone, browser or temporary AP required.
enum WifiEditorMode : uint8_t { EDIT_NONE, EDIT_PASSWORD, EDIT_SSID, EDIT_MIXER_IP };
static WifiEditorMode wifiEditor=EDIT_NONE;
static String editBuffer, editSsid;
static bool keyShift=false, keySymbols=false;
static int editSlot=0;
static String wifiNotice;
// Shared touch/encoder focus: 0..39 keyboard keys, 40..44 actions, 45 cancel.
static int keyboardFocus=0;
// v2.0 RC: Wi-Fi heeft net als kanalen/bussen een aparte actiemodus.
static bool wifiActionFocus=false;
static int wifiAction=0; // 0 verbinden, 1 wachtwoord, 2 X32 IP, 3 scan, 4 nieuw SSID
static constexpr const char* GB_VERSION="v2.4.4-rc.3";

// TV-menu: draaien = navigeren, kort drukken = openen/uitvoeren,
// lang drukken = een niveau terug. Geen dubbelklik of ingedrukt draaien.
// Kanaalbeheer: eerst een kanaal selecteren, daarna een actie kiezen.
static int manualBusTarget=0; // laatst aangewezen bus voor filtertoggle
static bool channelActionFocus=false;
static uint8_t channelAction=0; // 0 aan/uit, 1 omhoog, 2 omlaag, 3 volgend tabblad
static bool setupTabFocus=false; // tabbladen bereikbaar met draaien tot bovenaan
static uint8_t focusedTab=0;
static bool busActionFocus=false; // IEM BUS: bus selecteren -> actie selecteren
static uint8_t busAction=0; // 0 tonen/verbergen, 1 kiezen, 2 filter, 3 volgende tab


static void startWifiEditor(int mode, const String& ssid="");
static void wifiEditorKey(int x,int y);
static void finishWifiEditor();
static void beginWifiScan(){
  if(setupPortalActive || wifiScanning)return;
  WiFi.scanDelete();
  int r=WiFi.scanNetworks(true,false); // async; do not block encoder/OSC loop
  if(r==WIFI_SCAN_RUNNING || r>=0){
    wifiScanning=(r==WIFI_SCAN_RUNNING);wifiScanStarted=millis();
    wifiHaveResults=false;scannedCount=0;wifiCursor=0;wifiTop=0;
    Serial.println("WIFI SCAN: gestart");redraw=true;
  }else Serial.printf("WIFI SCAN: starten mislukt (%d)\n",r);
}
static void serviceWifiScan(){
  if(!wifiScanning)return;
  int n=WiFi.scanComplete();
  if(n==WIFI_SCAN_RUNNING){
    if(millis()-wifiScanStarted>15000){WiFi.scanDelete();wifiScanning=false;Serial.println("WIFI SCAN: timeout");redraw=true;}
    return;
  }
  wifiScanning=false;scannedCount=0;
  if(n>=0){
    for(int i=0;i<n&&scannedCount<WIFI_SCAN_MAX;i++){
      String ssid=WiFi.SSID(i);
      if(ssid.isEmpty())continue;
      bool duplicate=false;
      for(int j=0;j<scannedCount;j++)if(scannedSsid[j]==ssid){duplicate=true;break;}
      if(duplicate)continue;
      scannedSsid[scannedCount]=ssid;
      scannedRssi[scannedCount]=WiFi.RSSI(i);
      scannedSecure[scannedCount]=(WiFi.encryptionType(i)!=WIFI_AUTH_OPEN);
      scannedCount++;
    }
    wifiHaveResults=true;
  }
  WiFi.scanDelete();
  Serial.printf("WIFI SCAN: %d netwerken gevonden\n",scannedCount);
  redraw=true;
}
static int knownProfileFor(const String& ssid){
  for(int i=0;i<WIFI_PROFILES;i++)if(!savedSsid[i].isEmpty()&&savedSsid[i]==ssid)return i;
  return -1;
}
static void wifiSelectRow(); // defined with editor actions

static constexpr uint16_t X32_PORT = 10023;
static constexpr uint16_t FREENOVE_LOCAL_PORT = 10024;
static constexpr uint32_t WIFI_RETRY_MS = 5000;
static constexpr uint32_t XINFO_INTERVAL_MS = 3000;
static constexpr uint32_t X32_TIMEOUT_MS = 9000;
static constexpr uint32_t XREMOTE_INTERVAL_MS = 7000;
static constexpr uint32_t SUBSCRIBE_INTERVAL_MS = 6000; // Simulator vervalt na 10 s.
static constexpr uint32_t SYNC_RETRY_MS = 2000;
static constexpr uint32_t LIVE_POLL_MS = 1500;
static constexpr uint32_t METER_REQUEST_MS = 6000; // X32 meter subscription expires after 10 seconds
static constexpr uint32_t METER_STALE_MS = 1000;
static constexpr uint32_t METER_RENDER_MS = 65;    // max ~15 UI updates/s
static constexpr float REF_DB = -18.0f;
static constexpr float REF_OSC = (REF_DB+50.0f)/80.0f; // -18 dB = 0.4 as an X32 value
static constexpr float REF_DISPLAY = 0.75f; // dedicated visual fader scale
static float sendToDisplay(float x){
  x=constrain(x,0.0f,1.0f);
  return x<=REF_OSC ? REF_DISPLAY*(x/REF_OSC) :
    REF_DISPLAY+(1.0f-REF_DISPLAY)*(x-REF_OSC)/(1.0f-REF_OSC);
}
static uint32_t lastMeterRequest=0, lastMeterRx=0, meterPackets=0, rejectedMeters=0;
static float meterTarget[TOTAL_CHANNELS]={0}, meterSmoothed[TOTAL_CHANNELS]={0}, meterPeak[TOTAL_CHANNELS]={0};
static uint32_t meterPeakUntil[TOTAL_CHANNELS]={0};
// Inflight local send edits have priority over delayed readback from simulator.
static uint32_t lastLocalEditMs[TOTAL_CHANNELS]={0};
static constexpr uint32_t LOCAL_WRITE_GRACE_MS=260;
static bool metersValid=false;
static uint8_t selectedBus=3;
static char BUS_PATH[40]="/bus/03/config/name";
static char busNamePaths[16][40];
static char busNames[16][24] = {};
static bool busNameValid[16] = {false};
static bool manualBusAllowed[16] = {true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true};
static uint8_t busFilterMode=0;
static uint8_t setupPage=0;
static int otaCursor=0; // 0 = controleer; tabfocus = terug/navigeren
static int busCursor=2,busTop=0;
static Preferences busPrefs;
static uint32_t lastBusScan=0;
static constexpr int LIVE_CHANNELS = TOTAL_CHANNELS;
char liveName[LIVE_CHANNELS][16] = {};
float liveSend[LIVE_CHANNELS] = {0};
bool liveHasName[LIVE_CHANNELS] = {false};
bool liveHasSend[LIVE_CHANNELS] = {false};
static uint8_t liveColor[LIVE_CHANNELS] = {0};
static bool liveHasColor[LIVE_CHANNELS] = {false};
static char namePath[LIVE_CHANNELS][40],sendPath[LIVE_CHANNELS][40],colorPath[LIVE_CHANNELS][40];
extern int level[TOTAL_CHANNELS];
static void initLivePaths(){
  for(int i=0;i<LIVE_CHANNELS;++i){
    snprintf(liveName[i],sizeof(liveName[i]),"CH%02d",i+1);
    snprintf(namePath[i],sizeof(namePath[i]),"/ch/%02d/config/name",i+1);
    snprintf(sendPath[i],sizeof(sendPath[i]),"/ch/%02d/mix/%02d/level",i+1,selectedBus);
    snprintf(colorPath[i],sizeof(colorPath[i]),"/ch/%02d/config/color",i+1);
  }
}
static bool allLiveReceived(){
  for(int j=windowStart;j<windowStart+CHANNELS && j<shownCount;++j){int i=channelAt(j);if(i<0||!liveHasName[i]||!liveHasSend[i])return false;}
  return true;
}

enum X32LinkState : uint8_t { LINK_WIFI_DOWN, LINK_DISCOVERY, LINK_SYNCING, LINK_READY };
WiFiUDP x32udp;
X32LinkState linkState = LINK_WIFI_DOWN;
bool oscListening = false;
bool x32Identified = false;
bool gotBusName = false;
char simBusName[48] = "";
uint32_t lastInfoRequest=0, lastXinfoReply=0, lastSubscribe=0, lastSyncRequest=0;
uint32_t lastKeepalive=0, lastReconnectTry=0, packetsReceived=0, sendUpdates=0, lastLivePoll=0;
String netCommand;

static const char* linkLabel(){
  switch(linkState){
    case LINK_READY:return "ONLINE";
    case LINK_SYNCING:return "SYNC";
    case LINK_DISCOVERY:return "ZOEKEN";
    default:return "OFFLINE";
  }
}
static void setLink(uint8_t next){
  if(linkState!=next){
    linkState=static_cast<X32LinkState>(next);
    redraw=true;
    Serial.printf("X32 STATUS: %s\n",linkLabel());
  }
}
// OSC: nul-afgesloten strings, steeds 4-byte uitgelijnd; floats netwerk-byteorder.
static size_t oscPutStr(uint8_t* b,size_t cap,size_t off,const char* s){
  const size_t n=strlen(s)+1,padded=(n+3)&~(size_t)3;
  if(off+padded>cap)return cap+1;
  memcpy(b+off,s,n);memset(b+off+n,0,padded-n);return off+padded;
}
static size_t oscReadStr(const uint8_t* b,size_t length,size_t off,char* out,size_t cap){
  if(off>=length||cap<1)return length+1;
  size_t end=off;while(end<length&&b[end])end++;
  if(end>=length)return length+1;
  const size_t len=min(end-off,cap-1);
  memcpy(out,b+off,len);out[len]=0;return (end+4)&~(size_t)3;
}
static uint32_t oscGetU32(const uint8_t* b){
  return ((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
}
static void oscPutU32(uint8_t* b,size_t at,uint32_t n){
  b[at]=n>>24;b[at+1]=n>>16;b[at+2]=n>>8;b[at+3]=n;
}
static bool oscSend(const char* path,char type=0,float f=0,const char* str=nullptr,int32_t iv=0){
  if(WiFi.status()!=WL_CONNECTED||!oscListening)return false;
  uint8_t b[192]={};size_t p=oscPutStr(b,sizeof(b),0,path);
  if(p>sizeof(b))return false;
  if(type=='f'||type=='s'||type=='i'){
    char tag[3]={',',type,0};p=oscPutStr(b,sizeof(b),p,tag);
    if(type=='s')p=oscPutStr(b,sizeof(b),p,str?str:"");
    else if(p+4<=sizeof(b)){
      uint32_t n=0;
      if(type=='f')memcpy(&n,&f,4);else n=(uint32_t)iv;
      oscPutU32(b,p,n);p+=4;
    } else return false;
  }
  if(p>sizeof(b)||!x32udp.beginPacket(X32_IP,X32_PORT))return false;
  x32udp.write(b,p);return x32udp.endPacket()==1;
}
static bool oscSubscribe(const char* path){
  // Simulator v0.2g: '/subscribe' OSC ,si (pad, intervalFactor); 1 = 50 ms.
  // Dit is voorlopig een SIMULATOR-compatibiliteitsmodus, niet de
  // definitieve methode voor communicatie met de echte X32 Rack.
  if(WiFi.status()!=WL_CONNECTED||!oscListening)return false;
  uint8_t b[192]={};size_t p=oscPutStr(b,sizeof(b),0,"/subscribe");
  p=oscPutStr(b,sizeof(b),p,",si");p=oscPutStr(b,sizeof(b),p,path);
  if(p+4>sizeof(b))return false;
  oscPutU32(b,p,10);p+=4; // factor 10: om de 500 ms
  if(!x32udp.beginPacket(X32_IP,X32_PORT))return false;
  x32udp.write(b,p);return x32udp.endPacket()==1;
}
static void resetX32(bool dropIdentity){
  metersValid=false; lastMeterRx=0; lastMeterRequest=0;
  for(int i=0;i<TOTAL_CHANNELS;i++)meterTarget[i]=meterSmoothed[i]=meterPeak[i]=0;
  gotBusName=false;
  // Behoud laatst bekende namen en waarden zichtbaar, maar markeer ze ongeldig
  // voor bediening totdat alle 6 kanalen opnieuw zijn ingelezen.
  for(int i=0;i<LIVE_CHANNELS;++i){liveHasName[i]=false;liveHasSend[i]=false;liveHasColor[i]=false;}
  if(dropIdentity){x32Identified=false;lastXinfoReply=0;}
  lastSyncRequest=0;lastSubscribe=0;
  redraw=true;
}
static uint32_t lastMetadataPoll=0;
static int metadataCursor=0;
static void serviceChannelMetadata(uint32_t now){
  if(now-lastMetadataPoll<100)return;
  lastMetadataPoll=now;
  int ch=metadataCursor;metadataCursor=(metadataCursor+1)%LIVE_CHANNELS;
  oscSend(namePath[ch]);oscSend(colorPath[ch]);
}
static void requestMixerState(){
  oscSend(BUS_PATH);
  for(int j=windowStart;j<windowStart+CHANNELS && j<shownCount;++j){
    int i=channelAt(j);
    if(!liveHasName[i])oscSend(namePath[i]);
    if(!liveHasSend[i])oscSend(sendPath[i]);
    if(!liveHasColor[i])oscSend(colorPath[i]);
  }
  lastSyncRequest=millis();
}
static void subscribeMixerState(){
  oscSend("/xremote");
  oscSubscribe(BUS_PATH);
  for(int j=windowStart;j<windowStart+CHANNELS && j<shownCount;++j){
    int i=channelAt(j);oscSubscribe(namePath[i]);oscSubscribe(sendPath[i]);oscSubscribe(colorPath[i]);
  }
  lastSubscribe=millis();
}
static void processMixerUpdate(const char* path,const char* tags,const uint8_t* b,
                               size_t len,size_t at){
  if(strcmp(tags,",s")==0){
    for(int bi=0;bi<16;bi++)if(strcmp(path,busNamePaths[bi])==0){
      char name[32];size_t after=oscReadStr(b,len,at,name,sizeof(name));
      if(after<=len){
        bool changed=!busNameValid[bi]||strcmp(busNames[bi],name)!=0;
        snprintf(busNames[bi],sizeof(busNames[bi]),"%.23s",name);
        busNameValid[bi]=true;
        if(changed){redraw=true;Serial.printf("BUS%02d LABEL: %s\n",bi+1,busNames[bi]);}
      }
      break;
    }
  }
  if(strcmp(path,BUS_PATH)==0 && strcmp(tags,",s")==0){
    char name[48];size_t after=oscReadStr(b,len,at,name,sizeof(name));
    if(after<=len){
      bool changed=!gotBusName||strcmp(simBusName,name)!=0;
      snprintf(simBusName,sizeof(simBusName),"%s",name);gotBusName=true;
      if(changed){Serial.printf("X32 BUS%02u NAAM: %s\n",selectedBus,simBusName);redraw=true;}
    }
  }
  for(int i=0;i<LIVE_CHANNELS;++i){
    if(strcmp(path,namePath[i])==0 && strcmp(tags,",s")==0){
      char name[32];size_t after=oscReadStr(b,len,at,name,sizeof(name));
      if(after<=len){
        if(strcmp(liveName[i],name)!=0){
          snprintf(liveName[i],sizeof(liveName[i]),"%.15s",name);
          redraw=true;
        }
        liveHasName[i]=true;
      }
    } else if(strcmp(path,colorPath[i])==0 && strcmp(tags,",i")==0 && at+4<=len){
      int32_t v=(int32_t)oscGetU32(b+at);
      if(v>=0 && v<=15){
        if(!liveHasColor[i] || liveColor[i]!=(uint8_t)v){
          liveColor[i]=(uint8_t)v; redraw=true;
          Serial.printf("X32 KLEUR CH%02d = %d\n",i+1,(int)v);
        }
        liveHasColor[i]=true;
      }
    } else if(strcmp(path,sendPath[i])==0 && strcmp(tags,",f")==0 && at+4<=len){
      uint32_t bits=oscGetU32(b+at);float value=0;memcpy(&value,&bits,4);
      if(isfinite(value)&&value>=0.0f&&value<=1.0f){
        // Ignore a stale OSC reply during a short local-encoder-edit window.
        // Matching acks are accepted. External MX-Q edits remain live otherwise.
        if(lastLocalEditMs[i]!=0 &&
           (uint32_t)(millis()-lastLocalEditMs[i])<LOCAL_WRITE_GRACE_MS &&
           fabsf(value-liveSend[i])>0.0006f) continue;
        bool changed=!liveHasSend[i]||fabsf(value-liveSend[i])>0.0001f;
        liveSend[i]=value;liveHasSend[i]=true;
        int v=(int)lroundf(100.0f*value);
        if(level[i]!=v){level[i]=v;redraw=true;}
        if(changed){
          sendUpdates++;
          Serial.printf("X32 CH%02d -> BUS%02u LEVEL: %.6f\n",i+1,selectedBus,(double)value);
        }
      }
    }
  }
  if(x32Identified&&gotBusName&&allLiveReceived()&&linkState==LINK_SYNCING){
    setLink(LINK_READY);
    Serial.println("X32 ZICHTBARE KANALEN GESYNCHRONISEERD: encoder vrijgegeven");
  }
}
static bool changeLiveSend(int channel,float value){
  if(channel<0||channel>=LIVE_CHANNELS||linkState!=LINK_READY||
     !liveHasSend[channel]||!isfinite(value)){
    Serial.println("OSC SCHRIJVEN GEBLOKKEERD: offline of synchronisatie niet compleet");
    return false;
  }
  value=constrain(value,0.0f,1.0f);
  if(!oscSend(sendPath[channel],'f',value))return false;
  // Optimistische weergave voor directe encoderrespons; volgende X32-update
  // blijft gezaghebbend en corrigeert eventuele afwijking.
  liveSend[channel]=value;
  lastLocalEditMs[channel]=millis();
  level[channel]=(int)lroundf(value*100.0f);
  redraw=true;
  return true;
}

static bool busNameStartsIEM(int idx){
  if(idx<0||idx>=16||!busNameValid[idx])return false;
  const char* p=busNames[idx];
  while(*p==' ')p++;
  return (p[0]=='I'||p[0]=='i')&&(p[1]=='E'||p[1]=='e')&&(p[2]=='M'||p[2]=='m');
}
static bool busAllowed(int idx){
  return idx>=0&&idx<16&&(busFilterMode==0 ||
    (busFilterMode==1 && busNameStartsIEM(idx)) ||
    (busFilterMode==2 && manualBusAllowed[idx]));
}
static void saveBusSettings(){
  busPrefs.putUChar("chosen",selectedBus);
  busPrefs.putUChar("mode",busFilterMode);
  uint8_t flags[16];for(int i=0;i<16;i++)flags[i]=manualBusAllowed[i]?1:0;
  busPrefs.putBytes("allowed",flags,sizeof(flags));
}
static void loadBusSettings(){
  busPrefs.begin("gb-bus",false);
  uint8_t b=busPrefs.getUChar("chosen",3);
  selectedBus=(b>=1&&b<=16)?b:3;
  uint8_t m=busPrefs.getUChar("mode",0);busFilterMode=m<=2?m:0;
  if(busPrefs.getBytesLength("allowed")==16){uint8_t flags[16];busPrefs.getBytes("allowed",flags,16);
    for(int i=0;i<16;i++)manualBusAllowed[i]=flags[i]!=0;}
  busCursor=selectedBus-1;
  for(int i=0;i<16;i++)snprintf(busNamePaths[i],sizeof(busNamePaths[i]),"/bus/%02d/config/name",i+1);
  snprintf(BUS_PATH,sizeof(BUS_PATH),"/bus/%02d/config/name",selectedBus);
  Serial.printf("IEM-bus uit NVS: %02u | filter %u\n",selectedBus,busFilterMode);
}
static void regenerateBusPaths(){
  snprintf(BUS_PATH,sizeof(BUS_PATH),"/bus/%02d/config/name",selectedBus);
  for(int i=0;i<32;i++)snprintf(sendPath[i],sizeof(sendPath[i]),"/ch/%02d/mix/%02d/level",i+1,selectedBus);
}
static void requestBusNames(){
  if(!x32Identified)return;
  for(int i=0;i<16;i++)if(!busNameValid[i])oscSend(busNamePaths[i]);
  lastBusScan=millis();
}
static void chooseBus(int bus){
  if(bus<1||bus>16||!busAllowed(bus-1)){
    Serial.println("Bus niet beschikbaar in huidig filter");return;}
  if(selectedBus==bus)return;
  selectedBus=(uint8_t)bus;
  regenerateBusPaths();
  // Cruciaal: geen oude bus-sends gebruiken bij wisselen.
  gotBusName=false;simBusName[0]=0;
  for(int i=0;i<32;i++){liveHasSend[i]=false;lastLocalEditMs[i]=0;}
  if(x32Identified){setLink(LINK_SYNCING);requestMixerState();subscribeMixerState();}
  saveBusSettings();redraw=true;
  Serial.printf("IEM BUS GEKOZEN: %02d (opnieuw synchroniseren)\n",bus);
}

// Request group 1: first 32 values are incoming input-channel meters.
// OSC request /meters ,si "/meters/1" 2 => nominally every 100 ms for ~10 s.
static bool requestInputMeters(){
  if(WiFi.status()!=WL_CONNECTED || !oscListening || !x32Identified)return false;
  uint8_t b[96]={};size_t p=oscPutStr(b,sizeof(b),0,"/meters");
  p=oscPutStr(b,sizeof(b),p,",si");
  p=oscPutStr(b,sizeof(b),p,"/meters/1");
  if(p+4>sizeof(b))return false;
  oscPutU32(b,p,2);p+=4;
  if(!x32udp.beginPacket(X32_IP,X32_PORT))return false;
  x32udp.write(b,p);return x32udp.endPacket()==1;
}
static uint32_t readLE32(const uint8_t *b){
  return (uint32_t)b[0] | ((uint32_t)b[1]<<8) |
         ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24);
}
static void processMeterBlob(const uint8_t* b,size_t len,size_t at){
  // OSC blob: 4-byte BE byte length; 4-byte LE float count; LE float32 values.
  if(at+8>len){rejectedMeters++;return;}
  uint32_t blobLen=oscGetU32(b+at);at+=4;
  if(blobLen<4+TOTAL_CHANNELS*4 || blobLen>len-at){rejectedMeters++;return;}
  uint32_t count=readLE32(b+at);at+=4;
  if(count<TOTAL_CHANNELS || count>96 || (uint64_t)count*4+4>blobLen){rejectedMeters++;return;}
  bool valid=true;float samples[TOTAL_CHANNELS];
  for(int i=0;i<TOTAL_CHANNELS;i++){
    uint32_t bits=readLE32(b+at+i*4);memcpy(&samples[i],&bits,4);
    if(!isfinite(samples[i]) || samples[i]<0.0f || samples[i]>8.0f)valid=false;
  }
  if(!valid){rejectedMeters++;return;}
  for(int i=0;i<TOTAL_CHANNELS;i++)meterTarget[i]=samples[i];
  lastMeterRx=millis();meterPackets++;metersValid=true;
}
static void serviceMeterAnimation(){
  const uint32_t now=millis();
  bool fresh=linkState==LINK_READY&&metersValid&&(uint32_t)(now-lastMeterRx)<METER_STALE_MS;
  // 10 Hz sampled animation: fast attack, 250ms-ish release, peak hold 550ms.
  for(int i=0;i<TOTAL_CHANNELS;i++){
    float goal=fresh?meterTarget[i]:0.0f;
    float prev=meterSmoothed[i];
    if(goal>prev)meterSmoothed[i]=goal;
    else meterSmoothed[i]=prev*0.40f+goal*0.60f;
    if(meterSmoothed[i]<0.00001f)meterSmoothed[i]=0.0f;
    if(goal>=meterPeak[i]){meterPeak[i]=goal;meterPeakUntil[i]=now+400;}
    else if((int32_t)(now-meterPeakUntil[i])>0){
      meterPeak[i]*=0.64f;
      if(meterPeak[i]<meterSmoothed[i])meterPeak[i]=meterSmoothed[i];
    }
  }
  redraw=true;
}
static void oscServiceRx(){
  for(int packet=0;packet<6;packet++){
    int n=x32udp.parsePacket();if(n<=0)break;
    IPAddress source=x32udp.remoteIP();
    uint8_t b[512];int received=x32udp.read(b,min(n,(int)sizeof(b)));
    while(x32udp.available())x32udp.read();
    if(received<=0||(x32Identified && source!=X32_IP))continue;
    packetsReceived++;
    char path[100],tags[24]={};
    size_t at=oscReadStr(b,received,0,path,sizeof(path));
    if(at>(size_t)received)continue;
    if(at<(size_t)received&&b[at]==','){
      at=oscReadStr(b,received,at,tags,sizeof(tags));
      if(at>(size_t)received)continue;
    }
    if(strcmp(path,"/meters/1")==0 && strcmp(tags,",b")==0){
      if(x32Identified)processMeterBlob(b,received,at);
      continue;
    }
    if(strcmp(path,"/xinfo")==0&&strcmp(tags,",ssss")==0){
      char ip[32],name[48],model[32],firmware[32];
      at=oscReadStr(b,received,at,ip,sizeof(ip));
      at=oscReadStr(b,received,at,name,sizeof(name));
      at=oscReadStr(b,received,at,model,sizeof(model));
      at=oscReadStr(b,received,at,firmware,sizeof(firmware));
      if(at>(size_t)received)continue;
      // Beperk deze eerste versie tot verwachte simulator.
      // Echte X32 Rack en simulator hebben een verschillende netwerknaam.
      // Alleen mixers met X32-model worden geaccepteerd; geen willekeurige UDP-bronnen.
      if(strncmp(model,"X32",3)!=0 && strncmp(model,"M32",3)!=0)continue;
      if(!x32Identified){
        if(manualX32 && source!=X32_IP)continue;
        if(!manualX32 && X32_IP!=source){
          X32_IP=source;
          netPrefs.putString("mixer",X32_IP.toString());
          Serial.printf("X32 ontdekt op %s; adres bewaard\n",X32_IP.toString().c_str());
        }
      }
      lastXinfoReply=millis();
      if(!x32Identified){
        x32Identified=true;
        Serial.printf("X32 IDENTITEIT: %s | %s | versie %s | IP %s\n",name,model,firmware,ip);
        setLink(LINK_SYNCING);
        requestMixerState();
        subscribeMixerState();
        requestBusNames();
        if(requestInputMeters())lastMeterRequest=millis();
      }
    } else if(x32Identified){
      processMixerUpdate(path,tags,b,received,at);
    }
  }
}
// Eerst het laatst gevalideerde IP, daarna broadcast op het actuele subnet.
static bool sendDiscoveryBroadcast(){
  if(!oscListening||WiFi.status()!=WL_CONNECTED)return false;
  uint8_t packet[32]={};size_t n=oscPutStr(packet,sizeof(packet),0,"/xinfo");
  IPAddress local=WiFi.localIP(),mask=WiFi.subnetMask(),broadcast;
  for(int i=0;i<4;i++)broadcast[i]=local[i] | uint8_t(~mask[i]);
  if(!x32udp.beginPacket(broadcast,X32_PORT))return false;
  x32udp.write(packet,n);bool ok=x32udp.endPacket()==1;
  Serial.printf("X32 zoekbroadcast -> %s\n",broadcast.toString().c_str());
  return ok;
}
static void netStartUdp(){
  if(oscListening)return;
  oscListening=x32udp.begin(FREENOVE_LOCAL_PORT);
  Serial.printf("FREENOVE OSC UDP %u: %s\n",FREENOVE_LOCAL_PORT,oscListening?"GEREED":"MISLUKT");
  if(oscListening){
    resetX32(true);lastInfoRequest=0;lastKeepalive=0;discoveryStarted=millis();
    setLink(LINK_DISCOVERY);
    if(X32_IP!=IPAddress(0,0,0,0))oscSend("/xinfo");lastInfoRequest=millis();
  }
}
static void startWifiEditor(int mode, const String& ssid){
  wifiEditor=static_cast<WifiEditorMode>(mode); editBuffer=""; editSsid=ssid; keyShift=false;keySymbols=false;
  wifiNotice=""; keyboardFocus=0;
  if(mode==EDIT_MIXER_IP){editBuffer=manualX32?X32_IP.toString():"";}
  if(mode==EDIT_PASSWORD){
    int known=knownProfileFor(ssid);
    editSlot=known>=0?known:-1;
    if(editSlot<0)for(int i=0;i<WIFI_PROFILES;i++)if(savedSsid[i].isEmpty()){editSlot=i;break;}
    if(editSlot<0){editSlot=(activeWifi+1)%WIFI_PROFILES;
      Serial.printf("WIFI: alle profielen bezet, nieuw netwerk vervangt profiel %d\n",editSlot+1);}
  }
  redraw=true;
}
static void finishWifiEditor(){
  if(wifiEditor==EDIT_SSID){
    if(editBuffer.isEmpty()||editBuffer.length()>32){wifiNotice="SSID: 1-32 BYTES";redraw=true;return;}
    String ssid=editBuffer;
    startWifiEditor(EDIT_PASSWORD,ssid);
    return;
  }
  if(wifiEditor==EDIT_PASSWORD){
    if(editSsid.isEmpty()||editSsid.length()>32||editBuffer.length()>63 ||
       (editBuffer.length()>0&&editBuffer.length()<8)){
      wifiNotice="WACHTWOORD: 8-63 OF LEEG";redraw=true;return;
    }
    savedSsid[editSlot]=editSsid;savedPass[editSlot]=editBuffer;
    netPrefs.putString((String("s")+editSlot).c_str(),editSsid);
    netPrefs.putString((String("p")+editSlot).c_str(),editBuffer);
    netPrefs.putUChar("active",editSlot);
    netPrefs.putBool("fixed",false);
    netPrefs.putString("mixer","0.0.0.0");
    Serial.printf("WIFI: profiel %d opgeslagen, verbinden met %s\n",editSlot+1,editSsid.c_str());
    ESP.restart();return;
  }
  if(wifiEditor==EDIT_MIXER_IP){
    IPAddress ip;
    if(editBuffer.length()&&!ip.fromString(editBuffer)){
      wifiNotice="ONGELDIG IPv4-ADRES";redraw=true;return;
    }
    manualX32=editBuffer.length()>0;
    netPrefs.putBool("fixed",manualX32);
    netPrefs.putString("mixer",manualX32?editBuffer:String("0.0.0.0"));
    Serial.printf("X32: %s\n",manualX32?editBuffer.c_str():"AUTOMATISCH");
    ESP.restart();return;
  }
}
static void cancelWifiEditor(){
  wifiEditor=EDIT_NONE;editBuffer="";wifiNotice="";keyboardFocus=0;
  redraw=true;Serial.println("WIFI EDITOR: geannuleerd (geen wijzigingen opgeslagen)");
}
static void keyboardActivate(int focus){
  if(focus==45){cancelWifiEditor();return;}
  if(focus>=0&&focus<40){
    int row=focus/10,col=focus%10;
    static const char* alpha[4]={"1234567890","QWERTYUIOP","ASDFGHJKL-","ZXCVBNM._@"};
    static const char* symbol[4]={"1234567890","!@#$%^&*()","-_=+[]{}\\|",";:'\",./?~<"};
    const char* keys=keySymbols?symbol[row]:alpha[row];
    if(col>=(int)strlen(keys))return;
    char c=keys[col];
    if(!keyShift&&c>='A'&&c<='Z')c=char(c-'A'+'a');
    size_t limit=wifiEditor==EDIT_PASSWORD?63:(wifiEditor==EDIT_SSID?32:15);
    if(editBuffer.length()<limit)editBuffer+=c;
    wifiNotice="";redraw=true;return;
  }
  switch(focus){
    case 40:keyShift=!keyShift;break;
    case 41:keySymbols=!keySymbols;break;
    case 42:if(editBuffer.length()<(wifiEditor==EDIT_PASSWORD?63u:(wifiEditor==EDIT_SSID?32u:15u)))editBuffer+=' ';break;
    case 43:if(editBuffer.length())editBuffer.remove(editBuffer.length()-1);break;
    case 44:finishWifiEditor();break;
  }
  redraw=true;
}
static void wifiEditorKey(int x,int y){
  // ANNULEREN is painted at x=365..469, y=40..71.
  if(x>=365&&x<470&&y>=40&&y<73){keyboardActivate(45);return;}
  if(y>=105&&y<261){
    int col=(x-7)/46,row=(y-105)/39;
    if(x>=7&&col>=0&&col<10&&row>=0&&row<4){
      keyboardFocus=row*10+col;keyboardActivate(keyboardFocus);
    }
    return;
  }
  if(y>=268&&y<313){
    int action=(x-7)/95;
    if(x>=7&&action>=0&&action<5){keyboardFocus=40+action;keyboardActivate(keyboardFocus);}
  }
}
// Change password for a previously saved profile without revealing the old password.
// wifiCursor selects a saved profile or a scanned, already-known SSID.
static void editSavedWifiPassword(){
  int profile=-1;
  if(wifiCursor>=0 && wifiCursor<WIFI_PROFILES){
    profile=wifiCursor;
  }else if(wifiCursor>=WIFI_PROFILES && wifiCursor<WIFI_PROFILES+scannedCount){
    profile=knownProfileFor(scannedSsid[wifiCursor-WIFI_PROFILES]);
  }
  if(profile<0 || profile>=WIFI_PROFILES || savedSsid[profile].isEmpty()){
    wifiNotice="SELECTEER OPGESLAGEN NETWERK";
    redraw=true;
    return;
  }
  Serial.printf("WIFI: nieuw wachtwoord invoeren voor profiel %d (%s)\n",profile+1,savedSsid[profile].c_str());
  startWifiEditor(EDIT_PASSWORD,savedSsid[profile]);
  // No old password is ever prefilled. SAVE replaces it; CANCEL keeps it.
}
static void wifiSelectRow(){
  if(wifiCursor<0||wifiCursor>=WIFI_PROFILES+scannedCount)return;
  int profile=-1;String ssid;
  if(wifiCursor<WIFI_PROFILES){
    profile=wifiCursor;ssid=savedSsid[profile];
    if(ssid.isEmpty()){startWifiEditor(EDIT_SSID);return;}
  }else{
    ssid=scannedSsid[wifiCursor-WIFI_PROFILES];
    profile=knownProfileFor(ssid);
  }
  if(profile>=0){
    if(activeWifi!=profile || WiFi.status()!=WL_CONNECTED || WiFi.SSID()!=ssid){
      netPrefs.putUChar("active",profile);
      netPrefs.putBool("fixed",false);netPrefs.putString("mixer","0.0.0.0");
      Serial.printf("WIFI: profiel %d gekozen, herstart\n",profile+1);
      ESP.restart();
    }else{wifiNotice="DIT NETWERK IS ACTIEF";redraw=true;}
    return;
  }
  startWifiEditor(EDIT_PASSWORD,ssid);
}
// Een profiel is alleen actief als de ESP32 werkelijk met die SSID is verbonden.
static int wifiCurrentProfile(){
  if(wifiCursor>=0 && wifiCursor<WIFI_PROFILES)return savedSsid[wifiCursor].isEmpty()?-1:wifiCursor;
  if(wifiCursor>=WIFI_PROFILES && wifiCursor<WIFI_PROFILES+scannedCount)
    return knownProfileFor(scannedSsid[wifiCursor-WIFI_PROFILES]);
  return -1;
}
static String wifiCurrentSsid(){
  if(wifiCursor>=0 && wifiCursor<WIFI_PROFILES)return savedSsid[wifiCursor];
  if(wifiCursor>=WIFI_PROFILES && wifiCursor<WIFI_PROFILES+scannedCount)
    return scannedSsid[wifiCursor-WIFI_PROFILES];
  return "";
}
static bool wifiCurrentConnected(){
  String ssid=wifiCurrentSsid();
  return !ssid.isEmpty() && WiFi.status()==WL_CONNECTED && WiFi.SSID()==ssid;
}
static bool wifiActionAvailable(int action){
  switch(action){
    case 0:return !wifiCurrentSsid().isEmpty() && !wifiCurrentConnected(); // VERBINDEN
    case 1:return wifiCurrentProfile()>=0;                          // WIJZIG PW
    case 2:return wifiCurrentConnected();                           // X32 IP
    case 3:return true;                                             // SCAN
    case 4:return true;                                             // NIEUW SSID
    default:return false;
  }
}
static int firstWifiAction(){
  for(int i=0;i<5;i++)if(wifiActionAvailable(i))return i;
  return 3;
}
static int nextWifiAction(int current,int direction){
  for(int i=current+direction;i>=0&&i<5;i+=direction)
    if(wifiActionAvailable(i))return i;
  return -1;
}
static void selectWifiForEditing(int ix){
  if(ix<0||ix>=WIFI_PROFILES+scannedCount)return;
  wifiCursor=ix;
  wifiActionFocus=true;
  wifiAction=firstWifiAction();
  wifiNotice="";
  redraw=true;
}
static void runWifiAction(int action){
  if(!wifiActionAvailable(action)){wifiNotice="ACTIE NIET BESCHIKBAAR";redraw=true;return;}
  switch(action){
    case 0:wifiSelectRow();break;  // profiel opnieuw verbinden of wachtwoord vragen
    case 1:editSavedWifiPassword();break;
    case 2:startWifiEditor(EDIT_MIXER_IP);break;
    case 3:beginWifiScan();break;
    case 4:startWifiEditor(EDIT_SSID);break;
  }
}
static void netLoop(){
  if(setupPortalActive)return;
  uint32_t now=millis();
  if(WiFi.status()!=WL_CONNECTED){
    if(oscListening){
      x32udp.stop();oscListening=false;
      resetX32(true);setLink(LINK_WIFI_DOWN);
      Serial.println("Wi-Fi verbinding verloren: schrijven geblokkeerd");
    }
    if(now-lastReconnectTry>=WIFI_RETRY_MS){
      lastReconnectTry=now;
      connectSavedWifi();
      Serial.println("Wi-Fi opnieuw verbinden ...");
    }
    return;
  }
  if(!oscListening){
    Serial.printf("FREENOVE WIFI OK | IP %s | RSSI %ld dBm\n",
       WiFi.localIP().toString().c_str(),(long)WiFi.RSSI());
    netStartUdp();
  }
  if(!oscListening)return;
  oscServiceRx();now=millis();
  if(!x32Identified && !manualX32 && (uint32_t)(now-discoveryStarted)>4500 && (uint32_t)(now-lastInfoRequest)>=XINFO_INTERVAL_MS){
    sendDiscoveryBroadcast();
  }
  if((uint32_t)(now-lastInfoRequest)>=XINFO_INTERVAL_MS){
    lastInfoRequest=now;if(X32_IP!=IPAddress(0,0,0,0))oscSend("/xinfo");
  }
  if(x32Identified && (uint32_t)(now-lastXinfoReply)>X32_TIMEOUT_MS){
    Serial.println("X32 TIMEOUT: mixer antwoordt niet, schrijven geblokkeerd");
    resetX32(true);setLink(LINK_DISCOVERY);discoveryStarted=now;
    if(X32_IP!=IPAddress(0,0,0,0))oscSend("/xinfo");lastInfoRequest=now;
    return;
  }
  if(!x32Identified)return;
  serviceChannelMetadata(now);
  if((uint32_t)(now-lastBusScan)>=15000){requestBusNames();}
  if((uint32_t)(now-lastKeepalive)>=XREMOTE_INTERVAL_MS){
    lastKeepalive=now;oscSend("/xremote");
  }
  if((uint32_t)(now-lastMeterRequest)>=METER_REQUEST_MS){
    if(requestInputMeters())lastMeterRequest=now;
  }
  if((uint32_t)(now-lastSubscribe)>=SUBSCRIBE_INTERVAL_MS){
    subscribeMixerState();
  }
  if(linkState==LINK_SYNCING && (uint32_t)(now-lastSyncRequest)>=SYNC_RETRY_MS){
    requestMixerState();
  }
  if(linkState==LINK_READY && (uint32_t)(now-lastLivePoll)>=LIVE_POLL_MS){
    lastLivePoll=now;
    for(int j=windowStart;j<windowStart+CHANNELS && j<shownCount;++j){
      int i=channelAt(j);oscSend(sendPath[i]);if(!liveHasColor[i])oscSend(colorPath[i]);
    }
  }
}

// ===== OTA partition inspection (read-only, no flash writes) =====
// Inspect the *actual* running partition table. Do not infer it from IDE settings.
struct OtaPartitionReport {
  bool runningPresent=false;
  bool updatePresent=false;
  bool fitsCurrentImage=false;
  uint32_t runningSize=0;
  uint32_t nextSize=0;
  uint32_t imageSize=0;
  uint32_t nextOffset=0;
  char runningLabel[17]={0};
  char nextLabel[17]={0};
};
static OtaPartitionReport otaParts;
static void otaInspectPartitions(bool verbose){
  otaParts=OtaPartitionReport{};
  const esp_partition_t* running=esp_ota_get_running_partition();
  const esp_partition_t* next=esp_ota_get_next_update_partition(nullptr);
  otaParts.imageSize=(uint32_t)ESP.getSketchSize();
  if(running){
    otaParts.runningPresent=true;
    otaParts.runningSize=running->size;
    strncpy(otaParts.runningLabel,running->label,sizeof(otaParts.runningLabel)-1);
  }
  if(next){
    otaParts.updatePresent=true;
    otaParts.nextSize=next->size;
    otaParts.nextOffset=next->address;
    strncpy(otaParts.nextLabel,next->label,sizeof(otaParts.nextLabel)-1);
  }
  otaParts.fitsCurrentImage=otaParts.updatePresent && otaParts.imageSize>0 && otaParts.imageSize<=otaParts.nextSize;
  if(verbose){
    Serial.println("=== OTA PARTITIE DIAGNOSE (ALLEEN LEZEN) ===");
    Serial.printf("Flash fysiek: %lu bytes | Sketch: %lu bytes\n",
      (unsigned long)ESP.getFlashChipSize(),(unsigned long)otaParts.imageSize);
    if(running) Serial.printf("Actieve app: %s | adres 0x%06lX | grootte %lu bytes | subtype 0x%02X\n",
      running->label,(unsigned long)running->address,(unsigned long)running->size,(unsigned)running->subtype);
    else Serial.println("Actieve app: NIET GEVONDEN");
    if(next) Serial.printf("OTA doelslot: %s | adres 0x%06lX | grootte %lu bytes | subtype 0x%02X\n",
      next->label,(unsigned long)next->address,(unsigned long)next->size,(unsigned)next->subtype);
    else Serial.println("OTA doelslot: ONTBREEKT (geen OTA mogelijk met deze indeling)");
    Serial.printf("Huidige image past in OTA doelslot: %s\n",otaParts.fitsCurrentImage?"JA":"NEE");
    Serial.println("LET OP: dit is een partitietest, geen download-/flash-/rollbacktest.");
    Serial.println("NVS/instellingen worden NIET gewijzigd.");
  }
}
// ===== End OTA partition inspection =====

// ===== OTA PROBE (read-only): NO firmware writes =====
extern bool setupMode;
static const char* OTA_BASE_VERSION="v2.4.4-rc.3";
static const char* OTA_PRIVATE_API="https://api.github.com/repos/MPet67/goedbedoeld-iem-controller/releases?per_page=1";
static const char* OTA_PUBLIC_TEST_API="https://api.github.com/repos/espressif/arduino-esp32/releases?per_page=1";
static const char OTA_ROOT_CA[] PROGMEM=R"cert(-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----

-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----

-----BEGIN CERTIFICATE-----
MIICOjCCAcGgAwIBAgIQQvLM2htpN0RfFf51KBC49DAKBggqhkjOPQQDAzBfMQsw
CQYDVQQGEwJHQjEYMBYGA1UEChMPU2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQDEy1T
ZWN0aWdvIFB1YmxpYyBTZXJ2ZXIgQXV0aGVudGljYXRpb24gUm9vdCBFNDYwHhcN
MjEwMzIyMDAwMDAwWhcNNDYwMzIxMjM1OTU5WjBfMQswCQYDVQQGEwJHQjEYMBYG
A1UEChMPU2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQDEy1TZWN0aWdvIFB1YmxpYyBT
ZXJ2ZXIgQXV0aGVudGljYXRpb24gUm9vdCBFNDYwdjAQBgcqhkjOPQIBBgUrgQQA
IgNiAAR2+pmpbiDt+dd34wc7qNs9Xzjoq1WmVk/WSOrsfy2qw7LFeeyZYX8QeccC
WvkEN/U0NSt3zn8gj1KjAIns1aeibVvjS5KToID1AZTc8GgHHs3u/iVStSBDHBv+
6xnOQ6OjQjBAMB0GA1UdDgQWBBTRItpMWfFLXyY4qp3W7usNw/upYTAOBgNVHQ8B
Af8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAKBggqhkjOPQQDAwNnADBkAjAn7qRa
qCG76UeXlImldCBteU/IvZNeWBj7LRoAasm4PdCkT0RHlAFWovgzJQxC36oCMB3q
4S6ILuH5px0CMk7yn2xVdOOurvulGu7t0vzCAxHrRVxgED1cf5kDW21USAGKcw==
-----END CERTIFICATE-----

-----BEGIN CERTIFICATE-----
MIIFijCCA3KgAwIBAgIQdY39i658BwD6qSWn4cetFDANBgkqhkiG9w0BAQwFADBf
MQswCQYDVQQGEwJHQjEYMBYGA1UEChMPU2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQD
Ey1TZWN0aWdvIFB1YmxpYyBTZXJ2ZXIgQXV0aGVudGljYXRpb24gUm9vdCBSNDYw
HhcNMjEwMzIyMDAwMDAwWhcNNDYwMzIxMjM1OTU5WjBfMQswCQYDVQQGEwJHQjEY
MBYGA1UEChMPU2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQDEy1TZWN0aWdvIFB1Ymxp
YyBTZXJ2ZXIgQXV0aGVudGljYXRpb24gUm9vdCBSNDYwggIiMA0GCSqGSIb3DQEB
AQUAA4ICDwAwggIKAoICAQCTvtU2UnXYASOgHEdCSe5jtrch/cSV1UgrJnwUUxDa
ef0rty2k1Cz66jLdScK5vQ9IPXtamFSvnl0xdE8H/FAh3aTPaE8bEmNtJZlMKpnz
SDBh+oF8HqcIStw+KxwfGExxqjWMrfhu6DtK2eWUAtaJhBOqbchPM8xQljeSM9xf
iOefVNlI8JhD1mb9nxc4Q8UBUQvX4yMPFF1bFOdLvt30yNoDN9HWOaEhUTCDsG3X
ME6WW5HwcCSrv0WBZEMNvSE6Lzzpng3LILVCJ8zab5vuZDCQOc2TZYEhMbUjUDM3
IuM47fgxMMxF/mL50V0yeUKH32rMVhlATc6qu/m1dkmU8Sf4kaWD5QazYw6A3OAS
VYCmO2a0OYctyPDQ0RTp5A1NDvZdV3LFOxxHVp3i1fuBYYzMTYCQNFu31xR13NgE
SJ/AwSiItOkcyqex8Va3e0lMWeUgFaiEAin6OJRpmkkGj80feRQXEgyDet4fsZfu
+Zd4KKTIRJLpfSYFplhym3kT2BFfrsU4YjRosoYwjviQYZ4ybPUHNs2iTG7sijbt
8uaZFURww3y8nDnAtOFr94MlI1fZEoDlSfB1D++N6xybVCi0ITz8fAr/73trdf+L
HaAZBav6+CuBQug4urv7qv094PPK306Xlynt8xhW6aWWrL3DkJiy4Pmi1KZHQ3xt
zwIDAQABo0IwQDAdBgNVHQ4EFgQUVnNYZJX5khqwEioEYnmhQBWIIUkwDgYDVR0P
AQH/BAQDAgGGMA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEMBQADggIBAC9c
mTz8Bl6MlC5w6tIyMY208FHVvArzZJ8HXtXBc2hkeqK5Duj5XYUtqDdFqij0lgVQ
YKlJfp/imTYpE0RHap1VIDzYm/EDMrraQKFz6oOht0SmDpkBm+S8f74TlH7Kph52
gDY9hAaLMyZlbcp+nv4fjFg4exqDsQ+8FxG75gbMY/qB8oFM2gsQa6H61SilzwZA
Fv97fRheORKkU55+MkIQpiGRqRxOF3yEvJ+M0ejf5lG5Nkc/kLnHvALcWxxPDkjB
JYOcCj+esQMzEhonrPcibCTRAUH4WAP+JWgiH5paPHxsnnVI84HxZmduTILA7rpX
DhjvLpr3Etiga+kFpaHpaPi8TD8SHkXoUsCjvxInebnMMTzD9joiFgOgyY9mpFui
TdaBJQbpdqQACj7LzTWb4OE4y2BThihCQRxEV+ioratF4yUQvNs+ZUH7G6aXD+u5
dHn5HrwdVw1Hr8Mvn4dGp+smWg9WY7ViYG4A++MnESLn/pmPNPW56MORcr3Ywx65
LvKRRFHQV80MNNVIIb/bE/FmJUNS0nAiNs2fxBx1IK1jcmMGDw4nztJqDby1ORrp
0XZ60Vzk50lJLVU3aPAaOpg+VBeHVOmmJ1CJeyAvP/+/oYtKR5j/K3tJPsMpRmAY
QqszKbrAKbkTidOIijlBO8n9pu0f9GBj39ItVQGL
-----END CERTIFICATE-----)cert";
static String otaLastTag="";
static String otaLastStatus="NOG NIET GECONTROLEERD";
static bool otaHasChecked=false;
static bool otaAwaitConfirm=false;
static bool otaInstalling=false;
static bool rollbackConfirm=false;
static String rollbackVersion,rollbackDigest;
static String imageDigest(const esp_partition_t* p){
  uint8_t hash[32]; char hex[65];
  if(!p || esp_partition_get_sha256(p,hash)!=ESP_OK)return "";
  for(int i=0;i<32;i++)snprintf(hex+2*i,3,"%02x",hash[i]);
  return String(hex);
}
static void inspectRollback(){
  rollbackVersion="";rollbackDigest="";
  const esp_partition_t* p=esp_ota_get_next_update_partition(nullptr);
  const esp_partition_t* running=esp_ota_get_running_partition();
  if(!p||!running||p->address==running->address)return;
  String digest=imageDigest(p);
  if(digest.isEmpty())return;
  if(digest=="d8bf5b8fcf434179e5f062ef4223c8a9121e953ed8d79e864221ad54f8d9fe83")rollbackVersion="v2.4.4-rc.2";
  if(digest=="6c11dd069886ebe5962c317d5d4ba24087f05060c3a16759b573d9b5b7be6211")rollbackVersion="v2.4.3";
  if(!rollbackVersion.isEmpty())rollbackDigest=digest;
  Serial.printf("ROLLBACK: %s\n",rollbackVersion.isEmpty()?"niet beschikbaar":rollbackVersion.c_str());
}
static void rollbackAction();
static void updateMenuAction();
static void otaProgress(const char* stage,size_t received,size_t total,bool failed);

static void otaNetworkDiagnose(){
  Serial.printf("NET: status=%d SSID=%s RSSI=%ld dBm\n",(int)WiFi.status(),WiFi.SSID().c_str(),(long)WiFi.RSSI());
  Serial.printf("NET: IP=%s GATEWAY=%s DNS1=%s DNS2=%s\n",
    WiFi.localIP().toString().c_str(),WiFi.gatewayIP().toString().c_str(),
    WiFi.dnsIP(0).toString().c_str(),WiFi.dnsIP(1).toString().c_str());
  if(WiFi.status()!=WL_CONNECTED){Serial.println("NET: geen Wi-Fi-verbinding");return;}
  IPAddress ip;
  bool githubOk=WiFi.hostByName("api.github.com",ip)==1;
  Serial.printf("DNS api.github.com: %s%s\n",githubOk?"OK -> ":"MISLUKT",githubOk?ip.toString().c_str():"");
  bool ntpOk=WiFi.hostByName("pool.ntp.org",ip)==1;
  Serial.printf("DNS pool.ntp.org: %s%s\n",ntpOk?"OK -> ":"MISLUKT",ntpOk?ip.toString().c_str():"");
  time_t now=time(nullptr);
  Serial.printf("KLOK UNIX: %lld | %s\n",(long long)now,now>1700000000?"GELDIG":"NOG NIET GESYNCHRONISEERD");
}
static bool otaWaitClock(){
  // TLS certificate validation requires correct UTC. Never bypass TLS checks.
  time_t now=time(nullptr);
  if(now>1700000000)return true;
  Serial.println("NTP: synchronisatie starten (maximaal 20 seconden)...");
  configTime(0,0,"pool.ntp.org","time.google.com","time.cloudflare.com");
  const uint32_t started=millis();
  while(millis()-started<20000){
    now=time(nullptr);
    if(now>1700000000){
      struct tm utc;
      gmtime_r(&now,&utc);
      Serial.printf("NTP: OK %04d-%02d-%02d %02d:%02d:%02d UTC\n",
        utc.tm_year+1900,utc.tm_mon+1,utc.tm_mday,utc.tm_hour,utc.tm_min,utc.tm_sec);
      return true;
    }
    delay(200);
  }
  Serial.println("NTP: timeout; controleer DNS, internettoegang en UDP 123");
  return false;
}
static String otaFirstJsonTag(const String& json){
  // Lightweight extraction for GitHub release list; only reads the tag.
  int key=json.indexOf("\"tag_name\"");
  if(key<0)return "";
  int colon=json.indexOf(':',key+10);if(colon<0)return "";
  int begin=json.indexOf('"',colon+1);if(begin<0)return "";
  int end=json.indexOf('"',begin+1);if(end<0||end-begin>70)return "";
  return json.substring(begin+1,end);
}
static int otaCmpVersions(String left,String right){
  left.trim();right.trim();
  if(left.startsWith("v"))left.remove(0,1);
  if(right.startsWith("v"))right.remove(0,1);
  int la=0,lb=0,lc=0,ra=0,rb=0,rc=0;
  if(sscanf(left.c_str(),"%d.%d.%d",&la,&lb,&lc)!=3 ||
     sscanf(right.c_str(),"%d.%d.%d",&ra,&rb,&rc)!=3)return -999;
  if(la!=ra)return la>ra?1:-1;
  if(lb!=rb)return lb>rb?1:-1;
  if(lc!=rc)return lc>rc?1:-1;
  bool lpre=left.indexOf('-')>=0,rpre=right.indexOf('-')>=0;
  if(lpre!=rpre)return lpre?-1:1;
  // Experimental checker: do not infer prerelease ordering from different suffixes.
  if(lpre && left!=right)return -999;
  return 0;
}
// ===== v2.1c: add trusted Sectigo roots for GitHub TLS =====
// These operations are deliberately manual and only allowed in SETUP.
static bool otaDiagReady(){
  if(!setupMode){Serial.println("GITHUB: open eerst SETUP.");return false;}
  if(WiFi.status()!=WL_CONNECTED){Serial.println("GITHUB: Wi-Fi niet verbonden.");return false;}
  if(!otaWaitClock()){Serial.println("GITHUB: geen geldige klok; TLS niet uitgevoerd.");return false;}
  return true;
}
static void otaPrintTlsError(WiFiClientSecure& client){
  char message[192]={0};
  int code=client.lastError(message,sizeof(message));
  Serial.printf("TLS lastError: %d | %s\n",code,message[0]?message:"(geen detail beschikbaar)");
}
static void otaTcpDiagnose(){
  if(!otaDiagReady())return;
  IPAddress ip;
  if(WiFi.hostByName("api.github.com",ip)!=1){Serial.println("TCP: DNS MISLUKT");return;}
  Serial.printf("TCP: api.github.com -> %s:443\n",ip.toString().c_str());
  WiFiClient client;
  client.setTimeout(8000);
  uint32_t t=millis();
  bool ok=client.connect("api.github.com",443,8000);
  Serial.printf("TCP: %s in %lu ms\n",ok?"VERBONDEN":"MISLUKT",(unsigned long)(millis()-t));
  client.stop();
}
static void otaTlsDiagnose(){
  if(!otaDiagReady())return;
  WiFiClientSecure client;
  client.setCACert(OTA_ROOT_CA);
  client.setTimeout(12000);
  Serial.println("TLS: handshake naar api.github.com:443 met CA-verificatie (DigiCert/ISRG/Sectigo E46/R46)...");
  uint32_t t=millis();
  bool ok=client.connect("api.github.com",443,12000);
  Serial.printf("TLS: %s in %lu ms\n",ok?"GESLAAGD (certificaat gecontroleerd)":"MISLUKT",(unsigned long)(millis()-t));
  if(!ok)otaPrintTlsError(client);
  client.stop();
}
static void otaHttpDiagnose(){
  if(!otaDiagReady())return;
  WiFiClientSecure tls;
  tls.setCACert(OTA_ROOT_CA);
  tls.setTimeout(12000);
  HTTPClient http;
  http.setConnectTimeout(12000);
  http.setTimeout(12000);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  // Do not force HTTP/1.0 for this test: use the library default (HTTP/1.1).
  const char* url="https://api.github.com/repos/MPet67/goedbedoeld-iem-controller/releases?per_page=1";
  Serial.printf("HTTP: GET %s\n",url);
  if(!http.begin(tls,url)){Serial.println("HTTP: begin() MISLUKT");return;}
  http.addHeader("User-Agent","GoedBedoeld-IEMC-HTTPS-DIAG/0.2");
  http.addHeader("Accept","application/vnd.github+json");
  http.addHeader("X-GitHub-Api-Version","2022-11-28");
  int status=http.GET();
  Serial.printf("HTTP: status=%d (%s)\n",status,http.errorToString(status).c_str());
  if(status<0)otaPrintTlsError(tls);
  if(status>0){
    Serial.printf("HTTP: Content-Length=%d\n",http.getSize());
    if(status==200){
      // Keep JSON size bounded; do not print credentials or full network data.
      if(http.getSize()>50000){Serial.println("HTTP: respons te groot");}
      else{
        String response=http.getString();
        if(response.length()>50000)Serial.println("HTTP: respons te groot");
        else Serial.printf("HTTP: release tag=%s\n",otaFirstJsonTag(response).c_str());
      }
    }
  }
  http.end();
}
static void otaCheckReadOnly(bool publicTest){
  if(!publicTest){otaHasChecked=true;otaLastTag="";otaAwaitConfirm=false;}
  const char* url=publicTest?OTA_PUBLIC_TEST_API:OTA_PRIVATE_API;
  if(!setupMode){
    Serial.println("GITHUB: open eerst SETUP; HTTPS controle kan enkele seconden duren.");return;
  }
  if(WiFi.status()!=WL_CONNECTED){otaLastStatus="WIFI NIET VERBONDEN";Serial.println(otaLastStatus);return;}
  otaProgress("GITHUB / NTP CONTROLEREN",0,0,false);
  if(!otaWaitClock()){
    otaLastStatus="TIJD/NTP NIET BESCHIKBAAR";
    Serial.println("GITHUB: NTP tijd ontbreekt; HTTPS veilig afgebroken.");return;
  }
  WiFiClientSecure tls;
  tls.setCACert(OTA_ROOT_CA); // Never use setInsecure().
  tls.setTimeout(6000);
  HTTPClient http;
  http.setConnectTimeout(6000);
  http.setTimeout(6000);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  http.useHTTP10(true);
  if(!http.begin(tls,url)){otaLastStatus="HTTPS START MISLUKT";Serial.println(otaLastStatus);return;}
  http.addHeader("Accept","application/vnd.github+json");
  http.addHeader("User-Agent","GoedBedoeld-IEMC-OTA-ReadOnly/0.1");
  http.addHeader("X-GitHub-Api-Version","2022-11-28");
  int status=http.GET();
  Serial.printf("GITHUB HTTPS %s: HTTP %d | %s\n",publicTest?"PUBLIEKE TEST":"EIGEN REPOSITORY",status,http.errorToString(status).c_str());
  if(status<0)otaPrintTlsError(tls);
  if(status==200){
    int bytes=http.getSize();
    if(bytes>50000){otaLastStatus="RESPONSE TE GROOT";}
    else {
      String json=http.getString();
      if(json.length()>50000)otaLastStatus="RESPONSE TE GROOT";
      else {
        String tag=otaFirstJsonTag(json);
        if(tag.isEmpty())otaLastStatus="GEEN GEPUBLICEERDE RELEASE GEVONDEN";
        else{
          otaLastTag=tag;
          int cmp=otaCmpVersions(tag,OTA_BASE_VERSION);
          if(publicTest)otaLastStatus="HTTPS EN RELEASE JSON OK";
          else if(cmp==-999)otaLastStatus="TAG ONBEKEND / NIET VERGELIJKBAAR";
          else otaLastStatus=cmp>0?"NIEUWE VERSIE BESCHIKBAAR":
            cmp==0?"HUIDIGE VERSIE":"GEEN NIEUWERE VERSIE";
          Serial.printf("GITHUB release tag: %s | referentie: %s\n",tag.c_str(),OTA_BASE_VERSION);
        }
      }
    }
  }else if(status==404){otaLastStatus="HTTP 404: PRIVÉ REPO OF GEEN TOEGANG";}
  else if(status==403){otaLastStatus="HTTP 403: TOEGANGSLIMIET / NIET TOEGESTAAN";}
  else if(status<0){otaLastStatus="HTTPS/TLS VERBINDING MISLUKT";}
  else{otaLastStatus="GITHUB HTTP FOUT "+String(status);}
  Serial.printf("GITHUB: %s\n",otaLastStatus.c_str());
  http.end();
}

// ===== v2.4 guarded GitHub OTA (only after double confirmation) =====
// Asset conventions: GoedBedoeld_IEMC.bin + GoedBedoeld_IEMC.bin.sha256
// Trust boundary: GitHub HTTPS and own-release account integrity. No signature-based OTA.
static const char* OTA_BIN_NAME="GoedBedoeld_IEMC.bin";
static String otaAssetUrl(const String& tag,const String& filename){
  return "https://github.com/MPet67/goedbedoeld-iem-controller/releases/download/"+tag+"/"+filename;
}
static bool otaValidTag(const String& tag){
  if(tag.length()<2||tag.length()>48)return false;
  for(size_t i=0;i<tag.length();i++){
    char c=tag[i];
    if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='.'||c=='_'||c=='-'))return false;
  }
  return true;
}
static bool otaAllowedHttpsUrl(const String& url){
  // Disallow HTTPS downgrade and unexpected redirect hosts.
  if(!url.startsWith("https://"))return false;
  int start=8,end=url.indexOf('/',start);
  if(end<0)return false;
  String host=url.substring(start,end);host.toLowerCase();
  return host=="github.com"||host=="release-assets.githubusercontent.com"||
         host=="objects.githubusercontent.com"||host=="api.github.com";
}
static bool otaRedirect(int status){return status==301||status==302||status==303||status==307||status==308;}
static void otaFailure(const String& why){
  otaLastStatus=why;otaAwaitConfirm=false;otaInstalling=false;redraw=true;
  Serial.printf("OTA AFGEBROKEN: %s\n",why.c_str());
  otaProgress(why.c_str(),0,0,true);
  delay(1800);
}
static bool otaReadyForInstall(){
  if(!setupMode||WiFi.status()!=WL_CONNECTED){otaFailure("SETUP/WIFI VEREIST");return false;}
  if(!otaHasChecked||!otaValidTag(otaLastTag)||otaCmpVersions(otaLastTag,OTA_BASE_VERSION)<=0){
    otaFailure("GEEN NIEUWERE RELEASE");return false;
  }
  otaInspectPartitions(false);
  if(!otaParts.updatePresent||!otaParts.runningPresent){otaFailure("OTA SLOT ONTBREEKT");return false;}
  if(!otaWaitClock()){otaFailure("NTP/TIJD ONTBREEKT");return false;}
  return true;
}
// Resolve signed GitHub release URL redirects; each hop uses verified TLS.
// Returns a SHA256 file in the form '<64 hex>  GoedBedoeld_IEMC.bin'.
static bool otaLoadExpectedSha(String& expected){
  String url=otaAssetUrl(otaLastTag,String(OTA_BIN_NAME)+".sha256");
  for(int hop=0;hop<5;hop++){
    if(!otaAllowedHttpsUrl(url)){otaFailure("ONGELDIGE HTTPS REDIRECT");return false;}
    WiFiClientSecure tls;tls.setCACert(OTA_ROOT_CA);tls.setTimeout(12000);
    HTTPClient http;http.setConnectTimeout(12000);http.setTimeout(12000);
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    if(!http.begin(tls,url)){otaFailure("SHA256 HTTPS START FOUT");return false;}
    http.addHeader("User-Agent","GoedBedoeld-IEMC-OTA/2.4");
    int code=http.GET();
    if(otaRedirect(code)){
      String next=http.getLocation();http.end();url=next;continue;
    }
    if(code!=200){Serial.printf("OTA SHA HTTP=%d\n",code);http.end();otaFailure("SHA256 DOWNLOAD FOUT");return false;}
    int sz=http.getSize();
    if(sz<0||sz>256){http.end();otaFailure("SHA256 BESTANDSGROOTTE ONGELDIG");return false;}
    String body=http.getString();http.end();body.trim();
    if(body.length()<64){otaFailure("SHA256 INHOUD TE KORT");return false;}
    expected=body.substring(0,64);expected.toLowerCase();
    for(int i=0;i<64;i++){
      char c=expected[i];
      if(!((c>='0'&&c<='9')||(c>='a'&&c<='f'))){otaFailure("SHA256 GEEN HEX");return false;}
    }
    if(body.length()>64){
      String suffix=body.substring(64);suffix.trim();
      if(suffix.length() && suffix!="GoedBedoeld_IEMC.bin"&&suffix!="*GoedBedoeld_IEMC.bin"){
        otaFailure("SHA256 BESTANDSNAAM WIJKT AF");return false;
      }
    }
    Serial.printf("OTA verwacht SHA256: %s\n",expected.c_str());
    return true;
  }
  otaFailure("TE VEEL SHA256 REDIRECTS");return false;
}
static bool otaWriteFromGitHub(const String& expected){
  String url=otaAssetUrl(otaLastTag,OTA_BIN_NAME);
  for(int hop=0;hop<5;hop++){
    if(!otaAllowedHttpsUrl(url)){otaFailure("ONGELDIGE BINARY REDIRECT");return false;}
    WiFiClientSecure tls;tls.setCACert(OTA_ROOT_CA);tls.setTimeout(15000);
    HTTPClient http;http.setConnectTimeout(15000);http.setTimeout(15000);
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    if(!http.begin(tls,url)){otaFailure("BIN HTTPS START FOUT");return false;}
    http.addHeader("User-Agent","GoedBedoeld-IEMC-OTA/2.4");
    int code=http.GET();
    if(otaRedirect(code)){
      String next=http.getLocation();http.end();url=next;continue;
    }
    if(code!=200){Serial.printf("OTA BIN HTTP=%d\n",code);http.end();otaFailure("BIN DOWNLOAD FOUT");return false;}
    int length=http.getSize();
    if(length<=0||(uint32_t)length>otaParts.nextSize){
      http.end();otaFailure("BIN TE GROOT/GEEN CONTENT-LENGTH");return false;
    }
    otaProgress("DOWNLOAD / FLASH",0,(size_t)length,false);
    Serial.printf("OTA BIN %d bytes -> %s %lu bytes\n",length,otaParts.nextLabel,(unsigned long)otaParts.nextSize);
    if(!Update.begin((size_t)length,U_FLASH)){
      Update.printError(Serial);http.end();otaFailure("FLASH INITIALISATIE MISLUKT");return false;
    }
    mbedtls_sha256_context sha;mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha,0);
    WiFiClient* stream=http.getStreamPtr();
    uint8_t buf[2048];size_t received=0;
    uint32_t lastData=millis(),lastLog=0,lastPaint=0;
    bool ok=true;
    while(received<(size_t)length){
      int avail=stream->available();
      if(avail<=0){
        if(!http.connected()||millis()-lastData>15000){ok=false;break;}
        delay(5);continue;
      }
      size_t ask=min((size_t)avail,min(sizeof(buf),(size_t)length-received));
      int got=stream->readBytes(buf,ask);
      if(got<=0){ok=false;break;}
      if(received==0 && buf[0]!=0xE9){ok=false;Serial.println("OTA: ESP32 IMAGE MAGIC ONGELDIG");break;}
      if(Update.write(buf,(size_t)got)!=(size_t)got){ok=false;Update.printError(Serial);break;}
      mbedtls_sha256_update(&sha,buf,(size_t)got);
      received+=(size_t)got;lastData=millis();
      if(received-lastLog>=65536){lastLog=received;Serial.printf("OTA: %u/%d bytes\n",(unsigned)received,length);}
      if(millis()-lastPaint>=250 || received==(size_t)length){lastPaint=millis();otaProgress("DOWNLOAD / FLASH",received,(size_t)length,false);}
      delay(1);
    }
    unsigned char digest[32]={0};
    if(ok)mbedtls_sha256_finish(&sha,digest);
    mbedtls_sha256_free(&sha);
    http.end();
    if(!ok||received!=(size_t)length){Update.abort();otaFailure("DOWNLOAD/FLASH ONVOLLEDIG");return false;}
    char hex[65];const char* alphabet="0123456789abcdef";
    for(int i=0;i<32;i++){hex[i*2]=alphabet[digest[i]>>4];hex[i*2+1]=alphabet[digest[i]&15];}
    hex[64]='\0';
    if(expected!=String(hex)){
      Update.abort();Serial.printf("OTA SHA256 ontvangen: %s\n",hex);
      otaFailure("SHA256 MISMATCH - GEEN BOOTSWITCH");return false;
    }
    otaProgress("IMAGE VALIDEREN",received,(size_t)length,false);
    Serial.println("OTA: SHA256 OK; ESP32 image validatie starten");
    if(!Update.end(false)){
      Update.printError(Serial);otaFailure("UPDATE.END/IMAGE FOUT");return false;
    }
    Serial.printf("OTA: voltooid. Versie %s wordt na reboot gestart.\n",otaLastTag.c_str());
    return true;
  }
  otaFailure("TE VEEL BINARY REDIRECTS");return false;
}
static void otaBeginOrConfirm(){
  if(otaInstalling)return;
  if(!otaAwaitConfirm){
    if(!otaReadyForInstall())return;
    otaAwaitConfirm=true;
    otaLastStatus="BEVESTIG: DRUK NOGMAALS VOOR OTA";
    Serial.printf("OTA BEVESTIGING: installeer %s? Druk opnieuw, of houd in voor terug.\n",otaLastTag.c_str());
    redraw=true;return;
  }
  if(!otaReadyForInstall())return;
  otaAwaitConfirm=false;otaInstalling=true;
  otaLastStatus="OTA INSTALLATIE BEZIG";redraw=true;
  Serial.println("OTA: begin gecontroleerde installatie (onderbreek de voeding niet)");
  otaProgress("SHA256 OPHALEN",0,0,false);
  String sha;
  if(!otaLoadExpectedSha(sha)){otaInstalling=false;return;}
  if(!otaWriteFromGitHub(sha)){otaInstalling=false;return;}
  otaLastStatus="OTA GESLAAGD: HERSTART";redraw=true;
  otaProgress("INSTALLATIE OK - HERSTART",1,1,false);
  Serial.flush();delay(1500);ESP.restart();
}
static void rollbackAction(){
  if(otaInstalling)return;
  String expected=rollbackDigest;
  inspectRollback();
  if(expected.isEmpty()||expected!=rollbackDigest){rollbackConfirm=false;otaLastStatus="ROLLBACK NIET BESCHIKBAAR";redraw=true;return;}
  if(!rollbackConfirm){rollbackConfirm=true;otaLastStatus="DRUK NOGMAALS VOOR ROLLBACK";redraw=true;return;}
  rollbackConfirm=false;otaInstalling=true;
  otaProgress("ROLLBACK: IMAGE VALIDEREN",0,0,false);
  const esp_partition_t* p=esp_ota_get_next_update_partition(nullptr);
  if(!p||esp_ota_set_boot_partition(p)!=ESP_OK){otaInstalling=false;otaLastStatus="ROLLBACK IMAGE/BOOT FOUT";otaProgress("ROLLBACK MISLUKT",0,0,true);delay(1500);redraw=true;return;}
  Serial.printf("ROLLBACK: %s; herstart\n",rollbackVersion.c_str());
  otaProgress("ROLLBACK OK - HERSTART",1,1,false);
  Serial.flush();delay(1500);ESP.restart();
}
static void updateMenuAction(){
  if(otaCursor==1){rollbackAction();return;}
  if(!otaHasChecked||otaLastTag.isEmpty()){otaCheckReadOnly(false);redraw=true;}
  else if(otaAwaitConfirm||otaCmpVersions(otaLastTag,OTA_BASE_VERSION)>0)otaBeginOrConfirm();
  else {otaCheckReadOnly(false);redraw=true;}
}
// ===== einde v2.4 guarded OTA =====

// ===== end OTA PROBE =====

static void netSerial(){
  while(Serial.available()){
    char ch=Serial.read();
    if(ch=='\n'||ch=='\r'){
      netCommand.trim();
      if(netCommand.length()){
        if(netCommand=="ota partitions" || netCommand=="ota status"){
          otaInspectPartitions(true);
          redraw=true;
        } else if(netCommand=="github test"){
          otaCheckReadOnly(true);
        } else if(netCommand=="github check"){
          otaCheckReadOnly(false);
        } else if(netCommand=="github tcp"){
          otaTcpDiagnose();
        } else if(netCommand=="github tls"){
          otaTlsDiagnose();
        } else if(netCommand=="github http"){
          otaHttpDiagnose();
        } else if(netCommand=="github status"){
          Serial.printf("GITHUB: %s | laatst: %s\n",otaLastStatus.c_str(),otaLastTag.c_str());
        } else if(netCommand=="github diagnose"){
          otaNetworkDiagnose();
        } else if(netCommand=="github ntp"){
          otaNetworkDiagnose();
          if(WiFi.status()==WL_CONNECTED)Serial.printf("NTP RESULTAAT: %s\n",otaWaitClock()?"OK":"MISLUKT");
        } else if(netCommand=="wifi setup"){
          startWifiEditor(EDIT_SSID);
        } else if(netCommand=="wifi list"){
          for(int i=0;i<WIFI_PROFILES;i++)Serial.printf("%d: %s %s\n",i+1,savedSsid[i].isEmpty()?"(leeg)":savedSsid[i].c_str(),activeWifi==i?"[ACTIEF]":"");
          Serial.printf("Mixer %s (%s)\n",X32_IP.toString().c_str(),manualX32?"vast":"auto");
        } else if(netCommand=="status"){
          Serial.printf("Wi-Fi %s | IP %s | X32 %s | B%02u %s | RX %lu | updates %lu\n",
            WiFi.status()==WL_CONNECTED?"ONLINE":"OFFLINE",
            WiFi.localIP().toString().c_str(),linkLabel(),selectedBus,
            gotBusName?simBusName:"(nog niet gelezen)",
            (unsigned long)packetsReceived,(unsigned long)sendUpdates);
          Serial.printf("METERS /meters/1: %s | RX %lu | fout %lu | laatste %lu ms geleden\n",
            metersValid?"ONTVANGEN":"WACHT",(unsigned long)meterPackets,
            (unsigned long)rejectedMeters,
            lastMeterRx?(unsigned long)(millis()-lastMeterRx):0UL);
          for(int j=windowStart;j<windowStart+CHANNELS && j<shownCount;++j){int i=channelAt(j);
            Serial.printf("CH%02d %-15s SEND %.6f %s\n",i+1,liveName[i],
              (double)liveSend[i],liveHasSend[i]?"GELDIG":"WACHT OP SYNC");}
        } else if(netCommand=="probe"){
          oscSend("/xinfo");
          if(x32Identified)requestMixerState();
        } else if(netCommand=="getsend"){
          if(x32Identified)for(int j=windowStart;j<windowStart+CHANNELS && j<shownCount;++j)oscSend(sendPath[channelAt(j)]);
          else Serial.println("Geen mixeridentiteit: getsend overgeslagen");
        } else if(netCommand.startsWith("setsend ")){
          String arg=netCommand.substring(8);char* end=nullptr;
          float value=strtof(arg.c_str(),&end);
          if(end==arg.c_str()||*end||!isfinite(value)||value<0.0f||value>1.0f){
            Serial.println("Waarde moet tussen 0.0 en 1.0 liggen");
          } else if(linkState!=LINK_READY){
            Serial.printf("SCHRIJVEN GEBLOKKEERD: X32 status %s\n",linkLabel());
          } else {
            Serial.printf("OSC SET CH01 BUS%02u = %.6f: %s\n",selectedBus,(double)value,
              changeLiveSend(0,value)?"VERZONDEN":"MISLUKT");
          }
        } else if(netCommand=="buses"){
          Serial.printf("Busfilter %u | gekozen %02u\n",busFilterMode,selectedBus);
          for(int i=0;i<16;i++)Serial.printf("BUS%02d %-23s %s %s\n",i+1,busNameValid[i]?busNames[i]:"(nog niet gelezen)",busAllowed(i)?"KEUZE":"VERBORGEN",manualBusAllowed[i]?"M:JA":"M:NEE");
        } else if(netCommand.startsWith("bus ")){
          int number=netCommand.substring(4).toInt();chooseBus(number);
        } else if(netCommand=="scanbuses"){
          for(int i=0;i<16;i++)busNameValid[i]=false;
          requestBusNames();
        } else if(netCommand=="colors"){
          for(int j=windowStart;j<windowStart+CHANNELS && j<shownCount;++j){
            int i=channelAt(j);
            Serial.printf("CH%02d %-15s KLEUR=%u %s\n",i+1,liveName[i],(unsigned)liveColor[i],liveHasColor[i]?"OK":"WACHT");
          }
        } else Serial.println("Commando's: ota partitions | github diagnose | github ntp | github tcp | github tls | github http | github test | github check | github status | wifi setup | wifi list | status | probe | getsend | setsend 0.65 | colors | buses | bus 3 | scanbuses");
        netCommand="";
      }
    } else if(netCommand.length()<70&&ch>=32&&ch<127)netCommand+=ch;
  }
}

ST77922 lcd; // Freenove constructor initialiseert het display
ST77922_TOUCH touchPanel; // Touchcontroller op I2C GPIO38/GPIO39
uint16_t tile[W * TILE_H];
int level[TOTAL_CHANNELS] = {0};
// VU is nu alleen OSC-meterdata; geen demo-randomwaarden meer.
bool volumeMode = false, setupMode = false;
bool redraw = true;
bool diagnostics = false;  // Start direct met zes kanaalstroken
bool useCorrection = true; // RGB565-bytewissel voor iedere UI-kleur
// Hardware-interrupt op CLK: encoderpulsen blijven behouden tijdens SPI-displayrefresh.
// ISR raakt geen LCD, netwerk of Serial aan.
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int16_t encoderPending = 0;
static volatile uint8_t encoderPreviousAB = 0;
static volatile int8_t encoderQuarterSteps = 0;
static volatile uint32_t encoderInvalidTransitions = 0;
uint32_t lastAction = 0, lastVu = 0;

// Gray-code quadrature decoder. Every change on either input is checked
// against the previous 2-bit state. An invalid jump is discarded.
// One detent on the tested HW-040 is expected to produce four transitions.
void IRAM_ATTR onEncoderChange() {
  const uint8_t nowAB = (digitalRead(CLK_PIN)==HIGH ? 1u : 0u) |
                        (digitalRead(DT_PIN)==HIGH ? 2u : 0u);
  portENTER_CRITICAL_ISR(&encoderMux);
  const uint8_t oldAB=encoderPreviousAB;
  const uint8_t key=(oldAB<<2)|nowAB;
  static const int8_t directions[16]={
    0,+1,-1,0,
    -1,0,0,+1,
    +1,0,0,-1,
    0,-1,+1,0
  };
  const int8_t d=directions[key];
  encoderPreviousAB=nowAB;
  if(d!=0){
    int8_t q=encoderQuarterSteps+d;
    if(q>=4 || q<=-4){
      int16_t n=encoderPending+(q>=4 ? 1 : -1);
      if(n>=-64 && n<=64)encoderPending=n;
      q=0;
    }
    encoderQuarterSteps=q;
  } else if(oldAB!=nowAB) {
    encoderInvalidTransitions++;
    encoderQuarterSteps=0;
  }
  portEXIT_CRITICAL_ISR(&encoderMux);
}
bool lastRawPressed = false, stablePressed = false, longHandled = false;
uint32_t lastRawChange = 0, pressStart = 0;
// Touchtest, uitsluitend actief in setup.
bool touchReady = false;
bool touchWasDown = false;
static int dragChannel=-1;
static float dragValue=0;
static uint32_t dragSent=0;
static bool dragPending=false;
uint32_t lastTouchPoll = 0;
uint32_t touchCount = 0;
uint16_t touchX = 0, touchY = 0;
int8_t lastTouchZone = -1; // -1: geen vak, 0/1/2: testvak


// RGB565 in normale (logische) RGB-volgorde.
uint16_t rawRGB565(uint8_t r, uint8_t g, uint8_t b) {
  return (((uint16_t)((r * 31U + 127) / 255)) << 11) |
         (((uint16_t)((g * 63U + 127) / 255)) << 5) |
          ((uint16_t)((b * 31U + 127) / 255));
}
// RGB565-bytewissel: kandidaatcorrectie voor deze ST77922-tekenroute.
// Nog te bevestigen met de fysieke UI. De werkende schermdriver blijft intact.
uint16_t correctedRGB565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c = rawRGB565(r, g, b);
  return (uint16_t)((c << 8) | (c >> 8));
}
uint16_t uiRGB(uint8_t r, uint8_t g, uint8_t b) {
  return useCorrection ? correctedRGB565(r,g,b) : rawRGB565(r,g,b);
}
#define C_BG    uiRGB(10,20,33)
#define C_HEAD  uiRGB(16,34,51)
#define C_PANEL uiRGB(23,39,56)
#define C_EDGE  uiRGB(54,74,95)
#define C_TEXT  uiRGB(242,249,255)
#define C_MUTED uiRGB(165,190,210)
#define C_BLUE  uiRGB(59,168,236)
#define C_GREEN uiRGB(59,199,124)
#define C_AMBER uiRGB(242,179,62)
#define C_BARBG uiRGB(47,63,81)

int stripTop = 0;
void rect(int x, int y, int w, int h, uint16_t color) {
  if (w <= 0 || h <= 0) return;
  int x0 = max(0, x), x1 = min(W, x+w);
  int y0 = max(stripTop, y), y1 = min(stripTop+TILE_H, y+h);
  for (int py=y0; py<y1; ++py)
    for (int px=x0; px<x1; ++px)
      tile[(py-stripTop)*W+px] = color;
}
void border(int x,int y,int w,int h,int thickness,uint16_t color) {
  rect(x,y,w,thickness,color);
  rect(x,y+h-thickness,w,thickness,color);
  rect(x,y,thickness,h,color);
  rect(x+w-thickness,y,thickness,h,color);
}

// Kleine vaste 5x7 bitmapletters, zodat geen andere grafische bibliotheek nodig is.
// Een rij is een vijfbits patroon, hoogste bit is de linkerkolom.
void glyph(char ch, uint8_t p[7]) {
  // v1.2l: echte kleine letters, zodat de Shift-status ook op toetsen zichtbaar is.
  static const uint8_t blank[7] = {0,0,0,0,0,0,0};
  const uint8_t* q=blank;
  switch(ch) {
    case 'a': { static const uint8_t a[]={0,0,14,1,15,17,15}; q=a; break; }
    case 'b': { static const uint8_t a[]={16,16,30,17,17,17,30}; q=a; break; }
    case 'c': { static const uint8_t a[]={0,0,14,17,16,17,14}; q=a; break; }
    case 'd': { static const uint8_t a[]={1,1,15,17,17,17,15}; q=a; break; }
    case 'e': { static const uint8_t a[]={0,0,14,17,31,16,14}; q=a; break; }
    case 'f': { static const uint8_t a[]={6,9,8,28,8,8,8}; q=a; break; }
    case 'g': { static const uint8_t a[]={0,15,17,17,15,1,14}; q=a; break; }
    case 'h': { static const uint8_t a[]={16,16,30,17,17,17,17}; q=a; break; }
    case 'i': { static const uint8_t a[]={4,0,12,4,4,4,14}; q=a; break; }
    case 'j': { static const uint8_t a[]={2,0,6,2,2,18,12}; q=a; break; }
    case 'k': { static const uint8_t a[]={16,16,18,20,24,20,18}; q=a; break; }
    case 'l': { static const uint8_t a[]={12,4,4,4,4,4,14}; q=a; break; }
    case 'm': { static const uint8_t a[]={0,0,26,21,21,21,21}; q=a; break; }
    case 'n': { static const uint8_t a[]={0,0,30,17,17,17,17}; q=a; break; }
    case 'o': { static const uint8_t a[]={0,0,14,17,17,17,14}; q=a; break; }
    case 'p': { static const uint8_t a[]={0,30,17,17,30,16,16}; q=a; break; }
    case 'q': { static const uint8_t a[]={0,15,17,17,15,1,1}; q=a; break; }
    case 'r': { static const uint8_t a[]={0,0,22,25,16,16,16}; q=a; break; }
    case 's': { static const uint8_t a[]={0,0,15,16,14,1,30}; q=a; break; }
    case 't': { static const uint8_t a[]={8,8,28,8,8,9,6}; q=a; break; }
    case 'u': { static const uint8_t a[]={0,0,17,17,17,19,13}; q=a; break; }
    case 'v': { static const uint8_t a[]={0,0,17,17,17,10,4}; q=a; break; }
    case 'w': { static const uint8_t a[]={0,0,17,17,21,21,10}; q=a; break; }
    case 'x': { static const uint8_t a[]={0,0,17,10,4,10,17}; q=a; break; }
    case 'y': { static const uint8_t a[]={0,17,17,17,15,1,14}; q=a; break; }
    case 'z': { static const uint8_t a[]={0,0,31,2,4,8,31}; q=a; break; }
    case 'A': { static const uint8_t a[]={14,17,17,31,17,17,17}; q=a; break; }
    case 'B': { static const uint8_t a[]={30,17,17,30,17,17,30}; q=a; break; }
    case 'C': { static const uint8_t a[]={14,17,16,16,16,17,14}; q=a; break; }
    case 'D': { static const uint8_t a[]={30,17,17,17,17,17,30}; q=a; break; }
    case 'E': { static const uint8_t a[]={31,16,16,30,16,16,31}; q=a; break; }
    case 'F': { static const uint8_t a[]={31,16,16,30,16,16,16}; q=a; break; }
    case 'G': { static const uint8_t a[]={14,17,16,23,17,17,15}; q=a; break; }
    case 'H': { static const uint8_t a[]={17,17,17,31,17,17,17}; q=a; break; }
    case 'I': { static const uint8_t a[]={31,4,4,4,4,4,31}; q=a; break; }
    case 'J': { static const uint8_t a[]={7,2,2,2,18,18,12}; q=a; break; }
    case 'K': { static const uint8_t a[]={17,18,20,24,20,18,17}; q=a; break; }
    case 'L': { static const uint8_t a[]={16,16,16,16,16,16,31}; q=a; break; }
    case 'M': { static const uint8_t a[]={17,27,21,21,17,17,17}; q=a; break; }
    case 'N': { static const uint8_t a[]={17,25,21,19,17,17,17}; q=a; break; }
    case 'O': { static const uint8_t a[]={14,17,17,17,17,17,14}; q=a; break; }
    case 'P': { static const uint8_t a[]={30,17,17,30,16,16,16}; q=a; break; }
    case 'Q': { static const uint8_t a[]={14,17,17,17,21,18,13}; q=a; break; }
    case 'R': { static const uint8_t a[]={30,17,17,30,20,18,17}; q=a; break; }
    case 'S': { static const uint8_t a[]={15,16,16,14,1,1,30}; q=a; break; }
    case 'T': { static const uint8_t a[]={31,4,4,4,4,4,4}; q=a; break; }
    case 'U': { static const uint8_t a[]={17,17,17,17,17,17,14}; q=a; break; }
    case 'V': { static const uint8_t a[]={17,17,17,17,17,10,4}; q=a; break; }
    case 'W': { static const uint8_t a[]={17,17,17,21,21,21,10}; q=a; break; }
    case 'X': { static const uint8_t a[]={17,17,10,4,10,17,17}; q=a; break; }
    case 'Y': { static const uint8_t a[]={17,17,10,4,4,4,4}; q=a; break; }
    case 'Z': { static const uint8_t a[]={31,1,2,4,8,16,31}; q=a; break; }
    case '0': { static const uint8_t a[]={14,17,19,21,25,17,14}; q=a; break; }
    case '1': { static const uint8_t a[]={4,12,4,4,4,4,14}; q=a; break; }
    case '2': { static const uint8_t a[]={14,17,1,2,4,8,31}; q=a; break; }
    case '3': { static const uint8_t a[]={30,1,1,14,1,1,30}; q=a; break; }
    case '4': { static const uint8_t a[]={2,6,10,18,31,2,2}; q=a; break; }
    case '5': { static const uint8_t a[]={31,16,30,1,1,17,14}; q=a; break; }
    case '6': { static const uint8_t a[]={6,8,16,30,17,17,14}; q=a; break; }
    case '7': { static const uint8_t a[]={31,1,2,4,8,8,8}; q=a; break; }
    case '8': { static const uint8_t a[]={14,17,17,14,17,17,14}; q=a; break; }
    case '9': { static const uint8_t a[]={14,17,17,15,1,2,12}; q=a; break; }
    case '-': { static const uint8_t a[]={0,0,0,31,0,0,0}; q=a; break; }
    case '%': { static const uint8_t a[]={17,18,2,4,8,9,17}; q=a; break; }
    case ':': { static const uint8_t a[]={0,4,4,0,4,4,0}; q=a; break; }
    case '/': { static const uint8_t a[]={1,2,2,4,8,8,16}; q=a; break; }
    case '.': { static const uint8_t a[]={0,0,0,0,0,12,12}; q=a; break; }
    case '|': { static const uint8_t a[]={4,4,4,4,4,4,4}; q=a; break; }
    case '_': { static const uint8_t a[]={0,0,0,0,0,0,31}; q=a; break; }
    case ' ': break;
    case '?': { static const uint8_t a[]={14,17,1,2,4,0,4}; q=a; break; }
    case '+': { static const uint8_t a[]={0,4,4,31,4,4,0}; q=a; break; }
    case '!': { static const uint8_t a[]={4,4,4,4,4,0,4}; q=a; break; }
    case '$': { static const uint8_t a[]={4,15,20,14,5,30,4}; q=a; break; }
    case '^': { static const uint8_t a[]={4,10,17,0,0,0,0}; q=a; break; }
    case '&': { static const uint8_t a[]={12,18,20,8,21,18,13}; q=a; break; }
    case '*': { static const uint8_t a[]={0,21,14,31,14,21,0}; q=a; break; }
    case '=': { static const uint8_t a[]={0,0,31,0,31,0,0}; q=a; break; }
    case '[': { static const uint8_t a[]={14,8,8,8,8,8,14}; q=a; break; }
    case ']': { static const uint8_t a[]={14,2,2,2,2,2,14}; q=a; break; }
    case '{': { static const uint8_t a[]={3,4,4,8,4,4,3}; q=a; break; }
    case '}': { static const uint8_t a[]={24,4,4,2,4,4,24}; q=a; break; }
    case 92: { static const uint8_t a[]={16,8,8,4,2,2,1}; q=a; break; }
    case ';': { static const uint8_t a[]={0,4,4,0,4,4,8}; q=a; break; }
    case 39: { static const uint8_t a[]={4,4,8,0,0,0,0}; q=a; break; }
    case 34: { static const uint8_t a[]={10,10,10,0,0,0,0}; q=a; break; }
    case ',': { static const uint8_t a[]={0,0,0,0,4,4,8}; q=a; break; }
    case '~': { static const uint8_t a[]={0,0,9,22,0,0,0}; q=a; break; }
    case '<': { static const uint8_t a[]={2,4,8,16,8,4,2}; q=a; break; }
    case '(': { static const uint8_t a[]={2,4,8,8,8,4,2}; q=a; break; }
    case ')': { static const uint8_t a[]={8,4,2,2,2,4,8}; q=a; break; }
    case '#': { static const uint8_t a[]={10,10,31,10,31,10,10}; q=a; break; }
    case '@': { static const uint8_t a[]={14,17,23,21,23,16,14}; q=a; break; }
    default: { static const uint8_t a[]={14,17,1,2,4,0,4}; q=a; break; }
  }
  for(int i=0;i<7;++i) p[i]=q[i];
}
void text(int x,int y,const char* s,uint16_t color,int scale=1) {
  while(*s) {
    uint8_t p[7]; glyph(*s++,p);
    for(int row=0;row<7;++row) for(int bit=0;bit<5;++bit)
      if(p[row] & (1<<(4-bit)))
        rect(x+bit*scale,y+row*scale,scale,scale,color);
    x+=6*scale;
  }
}
void centerText(int cx,int y,const char* s,uint16_t color,int scale=1) {
  text(cx - ((int)strlen(s)*6*scale)/2,y,s,color,scale);
}

void paintDiagnostics() {
  // De hele diagnose gebruikt exact dezelfde tile + Fill_Colors-route als de UI.
  rect(0,0,W,H,rawRGB565(12,20,32));
  text(10,10,"KLEURTEST LANDSCAPE",rawRGB565(255,255,255),2);
  text(10,37,"BOVEN: RAW RGB565",rawRGB565(255,255,255),1);
  text(10,159,"ONDER: CORRECTIE",rawRGB565(255,255,255),1);
  const uint8_t rgb[6][3] = {
    {255,0,0},{0,255,0},{0,0,255},
    {255,255,0},{0,255,255},{255,0,255}
  };
  const char* labels[6] = {"ROOD","GROEN","BLAUW","GEEL","CYAAN","MAGENTA"};
  for(int i=0;i<6;++i) {
    int x=7+i*79;
    rect(x,55,73,78,rawRGB565(rgb[i][0],rgb[i][1],rgb[i][2]));
    rect(x,177,73,78,correctedRGB565(rgb[i][0],rgb[i][1],rgb[i][2]));
    centerText(x+36,140,labels[i],rawRGB565(255,255,255),1);
  }
  rect(0,276,W,44,rawRGB565(24,39,55));
  text(8,284,useCorrection?"UI MODUS: CORRECTIE":"UI MODUS: RAW",rawRGB565(255,255,255),1);
  text(8,302,"DRAAI: WISSEL   DRUK: OPEN UI",rawRGB565(255,255,255),1);
}
static uint16_t x32LabelBg(uint8_t code){
  switch(code & 7){
    case 2:return uiRGB(35,143,78); // groen
    case 4:return uiRGB(48,99,203); // blauw
    case 5:return uiRGB(159,62,174); // magenta
    case 1:return uiRGB(181,43,54);
    case 3:return uiRGB(195,150,39);
    case 6:return uiRGB(36,153,166);
    case 7:return uiRGB(210,215,220);
    default:return C_PANEL;
  }
}
void paintMix() {
  rect(0,0,W,H,C_BG);
  rect(0,0,W,34,C_HEAD);
  text(10,10,"GOED BEDOELD",C_TEXT,2);
  char busTitle[30];
  snprintf(busTitle,sizeof(busTitle),"B%02u %.16s",selectedBus,simBusName[0]?simBusName:"...");
  text(205,14,busTitle,C_TEXT,1);
  text(360,26,linkLabel(),linkState==LINK_READY?C_GREEN:C_AMBER,1);
  rect(410,2,66,32,C_PANEL);border(410,2,66,32,1,C_BLUE);centerText(443,14,"SETUP",C_TEXT,1);
  for (int i=0;i<CHANNELS;++i) {
    const int ch=channelAt(windowStart+i);
    if(ch<0)continue;
    int x=5+i*79;
    rect(x,40,74,243,C_PANEL);
    border(x,40,74,243,(windowStart+i==selected?3:1),
           (windowStart+i==selected?(volumeMode?C_GREEN:C_BLUE):C_EDGE));
    // Kleurlabel komt uit X32 OSC, niet uit de kanaalnaam.
    uint8_t code=liveColor[ch];
    uint16_t bg=x32LabelBg(code);
    rect(x+5,51,64,17,bg);
    uint16_t fg=(code==3 || code==7)?uiRGB(18,22,26):uiRGB(255,255,255);
    centerText(x+37,55,liveName[ch],fg,1);
    // Links: 20 discrete segmenten (VU), rechts: fader + referentielijn.
    text(x+8,77,"IN",C_MUTED,1);
    text(x+43,77,"MIX",C_MUTED,1);
    constexpr int Y_TOP=96, Y_BOTTOM=238, TRACK=Y_BOTTOM-Y_TOP;
    const int meterX=x+11, meterW=12, faderX=x+51;
    rect(meterX-2,Y_TOP-2,meterW+4,TRACK+4,C_BARBG);
    // Audio amplitude is linear; convert dBFS for the display.
    float audio=meterSmoothed[ch];
    float db=audio>0.000001f?20.0f*log10f(audio):-90.0f;
    float frac=constrain((db+60.0f)/60.0f,0.0f,1.0f);
    int active=constrain((int)ceilf(frac*20.0f),0,20);
    float peakDb=meterPeak[ch]>0.000001f?20.0f*log10f(meterPeak[ch]):-90.0f;
    int peakSegment=constrain((int)ceilf(constrain((peakDb+60.0f)/60.0f,0.0f,1.0f)*20.0f),0,20);
    for(int seg=0;seg<20;seg++){
      int y=Y_BOTTOM-7-(seg*7);
      uint16_t color=(seg>=17?uiRGB(239,69,64):(seg>=13?C_AMBER:C_GREEN));
      bool lit=seg<active||seg==peakSegment-1;
      rect(meterX,y,meterW,5,lit?color:uiRGB(35,50,59));
    }
    // Nonlinear visual scale: -18 dB (OSC 0.4) is at 75% of track height.
    // Both the fixed marker and the movable thumb share exactly this mapping.
    rect(faderX,Y_TOP,7,TRACK,C_BARBG);
    const int refY=Y_BOTTOM-(int)lroundf(REF_DISPLAY*TRACK);
    rect(x+39,refY,30,2,C_AMBER);
    const float send=constrain(liveSend[ch],0.0f,1.0f);
    const int thumbY=Y_BOTTOM-(int)lroundf(sendToDisplay(send)*TRACK);
    rect(x+41,thumbY-5,26,11,C_TEXT);
    rect(x+42,thumbY,24,2,C_HEAD);
    // Show actual send in dB rather than a misleading percentage.
    char txt[16];
    if(send<=0.00001f)snprintf(txt,sizeof(txt),"-INF");
    else {
      float d=send>=0.5f?send*40.0f-30.0f:
               (send>=0.25f?send*80.0f-50.0f:
               (send>=0.0625f?send*160.0f-70.0f:send*480.0f-90.0f));
      snprintf(txt,sizeof(txt),"%+.0fDB",(double)d);
    }
    centerText(x+37,254,txt,C_TEXT,1);
    centerText(x+37,270,"-18 REF",C_AMBER,1);
  }
  // Vensterindicator: toont welk deel van de 32 kanalen zichtbaar is.
  char range[30];snprintf(range,sizeof(range),"%02d-%02d / %02d",windowStart+1,min(windowStart+CHANNELS,shownCount),shownCount);
  text(205,26,range,C_MUTED,1);
  rect(0,286,W,34,C_HEAD);
  text(10,297,volumeMode?"VOLUME":"SELECTIE",volumeMode?C_GREEN:C_BLUE,2);
  text(202,299,linkState==LINK_READY?"DRAAI: MIX  HOUD: SETUP":"MIX NIET BESCHIKBAAR",C_MUTED,1);
  text(428,299,GB_VERSION,C_GREEN,1);
}
// v1.2r: vaste visuele taal voor alle SETUP-schermen.
// Blauw = geselecteerd/navigatie, groen = een actie bewerken.
static void focusOutline(int x,int y,int w,int h,bool selected,bool editing=false){
  if(selected)border(x,y,w,h,2,editing?C_GREEN:C_BLUE);
}
static void paintSetupTabs(){
  static const int xs[5]={76,160,244,328,412};
  static const int ws[5]={81,81,81,81,64};
  static const char* names[5]={"KANALEN","IEM BUS","WI-FI","UPDATE","MIXER"};
  for(int i=0;i<5;i++){
    rect(xs[i],2,ws[i],32,i==focusedTab?C_PANEL:C_HEAD);
    focusOutline(xs[i],2,ws[i],32,focusedTab==i);
    centerText(xs[i]+ws[i]/2,12,names[i],C_TEXT,1);
  }
}
// Handmatige GitHub release-controle; deze pagina installeert geen firmware.
static void paintSetupUpdate(){
  rect(0,0,W,H,C_BG);
  rect(0,0,W,37,C_HEAD);
  text(6,10,"SETUP",C_TEXT,2);
  paintSetupTabs();
  text(15,47,"FIRMWARE / OTA CONTROLE",C_TEXT,2);
  text(15,80,"GEINSTALLEERDE RELEASE",C_MUTED,1);
  text(15,96,OTA_BASE_VERSION,C_TEXT,2);
  text(15,126,"GITHUB RELEASE",C_MUTED,1);
  String tag=otaHasChecked?(otaLastTag.isEmpty()?String("NIET BESCHIKBAAR"):otaLastTag):String("NOG NIET GECONTROLEERD");
  text(15,141,tag.c_str(),C_TEXT,2);
  String status=otaLastStatus;
  if(status.length()>54)status=status.substring(0,54);
  text(15,179,status.c_str(),C_AMBER,1);
  text(15,202,"VORIGE FIRMWARE:",C_MUTED,1);
  text(150,202,rollbackVersion.isEmpty()?"NIET BESCHIKBAAR":rollbackVersion.c_str(),C_TEXT,1);
  rect(15,224,450,32,C_PANEL);focusOutline(15,224,450,32,otaCursor==0);
  centerText(240,236,otaAwaitConfirm?"BEVESTIG INSTALLATIE":
    (otaHasChecked && otaCmpVersions(otaLastTag,OTA_BASE_VERSION)>0)?"INSTALLEER NIEUWE VERSIE":"CONTROLEER OP UPDATES",C_TEXT,1);
  rect(15,261,450,32,C_PANEL);focusOutline(15,261,450,32,otaCursor==1);
  centerText(240,273,rollbackConfirm?"BEVESTIG ROLLBACK":"ROLLBACK",rollbackVersion.isEmpty()?C_MUTED:C_TEXT,1);
  text(15,305,"DRAAI: ACTIE  DRUK: KIES  HOUD: TERUG",C_MUTED,1);
}

void paintSetup() {
  rect(0,0,W,H,C_BG);
  rect(0,0,W,37,C_HEAD);
  text(6,10,"SETUP",C_TEXT,2);
  paintSetupTabs();
  text(14,47,"KANAAL",C_MUTED,1);
  text(215,47,"ZICHTBAAR",C_MUTED,1);
  for(int row=0;row<7;row++){
    int ix=setupTop+row;if(ix>=TOTAL_CHANNELS)break;
    int y=64+row*30;
    rect(10,y,460,28,C_PANEL);focusOutline(10,y,460,28,ix==setupSelected,!(!channelActionFocus)&&ix==setupSelected);
    char info[48];int ch=userOrder[ix];
    snprintf(info,sizeof(info),"%02d CH%02d",ix+1,ch+1);
    text(18,y+10,info,C_TEXT,1);
    uint8_t code=liveColor[ch];
    rect(92,y+4,155,20,x32LabelBg(code));
    centerText(169,y+10,liveName[ch],(code==3||code==7)?uiRGB(18,22,26):uiRGB(255,255,255),1);
    rect(260,y+4,84,20,C_HEAD);focusOutline(260,y+4,84,20,ix==setupSelected&&channelActionFocus&&channelAction==0,true);
    centerText(302,y+11,userVisible[ch]?"AAN":"UIT",userVisible[ch]?C_GREEN:C_MUTED,1);
    rect(359,y+4,45,20,C_HEAD);focusOutline(359,y+4,45,20,ix==setupSelected&&channelActionFocus&&channelAction==1,true);centerText(381,y+11,"UP",C_TEXT,1);
    rect(414,y+4,45,20,C_HEAD);focusOutline(414,y+4,45,20,ix==setupSelected&&channelActionFocus&&channelAction==2,true);centerText(436,y+11,"DN",C_TEXT,1);
  }
  rect(0,281,W,39,C_HEAD);
  if(!channelActionFocus){
    text(12,292,"DRAAI: KANAAL  DRUK: ACTIES",C_MUTED,1);
    text(12,305,setupTabFocus?"DRAAI: TAB  DRUK: OPEN":"HOUD: TERUG NAAR TABS",C_MUTED,1);
  }else{
    const char* actions[]={"AAN/UIT","OMHOOG","OMLAAG","TERUG NAAR TABS"};
    char nav[72];snprintf(nav,sizeof(nav),"ACTIE: %s",actions[channelAction]);
    text(12,292,nav,C_AMBER,1);
    text(12,305,"DRAAI: ACTIE  DRUK: UITVOEREN  HOUD: TERUG",C_MUTED,1);
  }
}

static void paintSetupBuses(){
  rect(0,0,W,H,C_BG);rect(0,0,W,37,C_HEAD);
  text(6,10,"SETUP",C_TEXT,2);
  paintSetupTabs();
  char title[90];
  snprintf(title,sizeof(title),"FILTER: %s",busFilterMode==0?"ALLE":busFilterMode==1?"DETECTEER IEM":"HANDMATIG");
  rect(8,41,360,24,C_PANEL);text(14,48,title,C_TEXT,1);
  rect(374,41,99,24,C_HEAD);focusOutline(374,41,99,24,!setupTabFocus&&busCursor==16,busActionFocus);centerText(423,49,"FILTER >",C_TEXT,1);
  for(int row=0;row<7;row++){
    int bi=busTop+row;if(bi>=16)break;
    int y=70+row*29;
    rect(10,y,460,27,C_PANEL);focusOutline(10,y,460,27,!setupTabFocus&&bi==busCursor,busActionFocus&&bi==busCursor);
    char info[72];snprintf(info,sizeof(info),"B%02d %-21.21s",bi+1,busNameValid[bi]?busNames[bi]:"(laden...)");
    text(17,y+9,info,C_TEXT,1);
    bool allowed=busAllowed(bi);
    rect(302,y+3,64,21,C_HEAD);focusOutline(302,y+3,64,21,bi==busCursor&&busActionFocus&&busAction==0,true);
    centerText(334,y+11,allowed?"JA":"NEE",allowed?C_GREEN:C_MUTED,1);
    rect(375,y+3,87,21,C_HEAD);focusOutline(375,y+3,87,21,bi==busCursor&&busActionFocus&&busAction==1,true);
    centerText(418,y+11,bi+1==selectedBus?"ACTIEF":"KIES",bi+1==selectedBus?C_GREEN:C_TEXT,1);
  }
  rect(0,281,W,39,C_HEAD);
  if(busActionFocus){
    const char* options[]={"AAN/UIT","KIES BUS","FILTER","TERUG NAAR TABS"};
    char footer[52];snprintf(footer,sizeof(footer),"BUS B%02d  ACTIE: %s",busCursor+1,options[busAction]);
    text(9,291,footer,C_AMBER,1);
    text(9,305,"DRAAI: ACTIE  DRUK: UITVOER  HOUD: BUS",C_MUTED,1);
  }else if(setupTabFocus){
    text(9,291,"TAB FOCUS: DRAAI  DRUK: OPEN",C_AMBER,1);
    text(9,305,"DRUK: OPEN",C_MUTED,1);
  }else{
    text(9,291,"DRAAI: BUS  DRUK: ACTIES",C_MUTED,1);
    text(9,305,"HOUD: TERUG NAAR TABS",C_MUTED,1);
  }
}

// Alleen voor het display. De originele SSID in scannedSsid/savedSsid blijft intact.
// UTF-8 diakritische tekens worden waar mogelijk vervangen door ASCII.
static String wifiLabelDisplay(const String& original){
  String result;
  for(size_t i=0;i<original.length() && result.length()<39;i++){
    uint8_t c=(uint8_t)original[i];
    if(c<128){result+=(char)c;continue;}
    if(c==0xC3 && i+1<original.length()){
      uint8_t d=(uint8_t)original[++i];
      if(d==0xA4||d==0x84||d==0xA0||d==0xA1||d==0xA2||d==0xA3||d==0x85||d==0x80||d==0x81)result+='A';
      else if(d==0xA9||d==0xA8||d==0xAA||d==0xAB||d==0x89)result+='E';
      else if(d==0xAD||d==0xAC||d==0xAE||d==0xAF)result+='I';
      else if(d==0xB6||d==0x96||d==0xB3||d==0xB2)result+='O';
      else if(d==0xBC||d==0x9C||d==0xBA||d==0xB9)result+='U';
      else if(d==0xB1||d==0x91)result+='N';
      else result+='?';
    }else if((c&0xC0)!=0x80)result+='?';
  }
  return result;
}
// De volgorde is cyclisch per scherm: huidig tabblad -> alle subacties -> volgend tabblad.
// Tabfocus is steeds eerst zichtbaar; de volgende rechtsom-stap betreedt de inhoud.
static void leaveSetupUI();
static void openSetupTab(uint8_t tab){
  if(tab==4){leaveSetupUI();return;}
  if(tab>4)return;
  setupPage=tab;focusedTab=tab;setupTabFocus=false;
  channelActionFocus=false;busActionFocus=false;wifiActionFocus=false;
  if(tab==0){setupSelected=0;setupTop=0;}
  else if(tab==1){busCursor=selectedBus-1;busTop=constrain(busCursor-3,0,9);requestBusNames();}
  else if(tab==2){wifiCursor=activeWifi;wifiTop=0;beginWifiScan();}
  else {otaCursor=0;otaAwaitConfirm=false;rollbackConfirm=false;otaInspectPartitions(false);inspectRollback();}
  redraw=true;
}
static void focusAdjacentTab(int direction){
  focusedTab=constrain((int)focusedTab+direction,0,4);if(focusedTab<4)setupPage=focusedTab;setupTabFocus=true;redraw=true;
}
static void setupNextPage(){setupTabFocus=true;focusedTab=setupPage;redraw=true;}
static void stepTabFocus(int steps){focusAdjacentTab(steps);}
static void channelEncoderAction(){
  if(!channelActionFocus){ channelActionFocus=true;channelAction=0;redraw=true;return; }
  switch(channelAction){
    case 0:changeVisibility();break;
    case 1:moveSetupChannel(-1);break;
    case 2:moveSetupChannel(+1);break;
    case 3:channelActionFocus=false;setupNextPage();break;
  }
  redraw=true;
}
static void wifiEncoderAction(){
  if(!wifiActionFocus){
    // Druk op een netwerk: groen (EDIT) en focus naar de beschikbare subacties.
    selectWifiForEditing(wifiCursor);
    return;
  }
  runWifiAction(wifiAction);
}
static void toggleBusAllowed(int idx){
  if(idx<0||idx>=16)return;
  // Neem de huidige beschikbare bussen over wanneer we overschakelen naar handmatig.
  if(busFilterMode!=2){
    bool previous[16];for(int i=0;i<16;i++)previous[i]=busAllowed(i);
    for(int i=0;i<16;i++)manualBusAllowed[i]=previous[i];
    busFilterMode=2;
  }
  if(manualBusAllowed[idx]){
    int remaining=0;for(int i=0;i<16;i++)if(i!=idx&&manualBusAllowed[i])remaining++;
    if(!remaining){Serial.println("Minstens een IEM-bus moet beschikbaar blijven");return;}
    if((int)selectedBus==idx+1){
      // Kies een andere toegestane bus voordat de huidige wordt verborgen.
      for(int i=0;i<16;i++)if(i!=idx&&manualBusAllowed[i]){chooseBus(i+1);break;}
    }
  }
  manualBusAllowed[idx]=!manualBusAllowed[idx];
  saveBusSettings();redraw=true;
  Serial.printf("IEM BUS%02d %s (handmatig filter)\n",idx+1,manualBusAllowed[idx]?"AAN":"UIT");
}
static void busEncoderAction(){
  if(!busActionFocus){
    if(busCursor>=16){ // Legacy acties onderaan behouden als extra route.
      if(busCursor==18){setupNextPage();return;}
      if(busCursor==16){busFilterMode=(busFilterMode+1)%3;saveBusSettings();redraw=true;return;}
      if(busCursor==17){toggleBusAllowed(manualBusTarget);return;}
    }
    busActionFocus=true;busAction=0;redraw=true;return;
  }
  switch(busAction){
    case 0:toggleBusAllowed(busCursor);break;
    case 1:chooseBus(busCursor+1);break;
    case 2:busFilterMode=(busFilterMode+1)%3;saveBusSettings();requestBusNames();redraw=true;break;
    case 3:busActionFocus=false;setupNextPage();break;
  }
}
static void paintWifiKeyboard(){
  rect(0,0,W,H,C_BG);rect(0,0,W,37,C_HEAD);
  rect(412,2,64,32,C_PANEL);centerText(444,12,"MIXER",C_TEXT,1);
  text(12,10,wifiEditor==EDIT_PASSWORD?"WIFI WACHTWOORD":wifiEditor==EDIT_MIXER_IP?"X32 IP-ADRES":"WIFI NAAM",C_TEXT,2);
  rect(365,40,105,32,C_PANEL);focusOutline(365,40,105,32,keyboardFocus==45);centerText(417,51,"ANNULEREN",C_TEXT,1);
  if(wifiEditor==EDIT_PASSWORD){
    String n=wifiLabelDisplay(editSsid);
    if(n.length()>32)n=n.substring(0,32);
    text(10,43,n.c_str(),C_GREEN,1);
  }else text(10,43,wifiEditor==EDIT_MIXER_IP?"LEEG = AUTOMATISCH":"SSID HANDMATIG",C_MUTED,1);
  rect(9,75,461,27,C_PANEL);
  String display;
  if(wifiEditor==EDIT_PASSWORD){
    // Mask the entered password; never echo it to serial logs.
    for(size_t i=0;i<editBuffer.length()&&i<50;i++)display+='#';
    if(editBuffer.length()>50)display=String("...")+display.substring(display.length()-46);
  }else{display=wifiLabelDisplay(editBuffer);if(display.length()>72)display=display.substring(display.length()-72);}
  text(16,83,display.c_str(),C_TEXT,1);
  if(wifiEditor!=EDIT_MIXER_IP){
    rect(228,55,130,17,keyShift?C_BLUE:C_GREEN);
    centerText(293,60,keyShift?"ABC HOOFD":"abc klein",C_TEXT,1);
  }
  static const char* alpha[4]={"1234567890","QWERTYUIOP","ASDFGHJKL-","ZXCVBNM._@"};
  static const char* symbol[4]={"1234567890","!@#$%^&*()","-_=+[]{}\\|",";:'\",./?~<"};
  for(int r=0;r<4;r++)for(int c=0;c<10;c++){
    const char* keys=keySymbols?symbol[r]:alpha[r];
    if(c>=(int)strlen(keys))continue;
    int x=7+c*46,y=105+r*39;
    rect(x,y,43,35,C_PANEL);focusOutline(x,y,43,35,keyboardFocus==r*10+c);
    char b[2]={keys[c],0};
    if(!keyShift&&b[0]>='A'&&b[0]<='Z')b[0]=char(b[0]-'A'+'a');
    centerText(x+21,y+12,b,C_TEXT,1);
  }
  const char* b[]={keyShift?"KLEIN":"HOOFD",keySymbols?"ABC":"SYMB","SPATIE","WIS","OPSLAAN"};
  for(int i=0;i<5;i++){
    int x=7+i*95;
    uint16_t bg=i==4?C_GREEN:(i==0?(keyShift?C_BLUE:C_PANEL):C_HEAD);
    rect(x,269,91,43,i==0&&keyShift?C_PANEL:C_HEAD);focusOutline(x,269,91,43,keyboardFocus==40+i);centerText(x+45,284,b[i],C_TEXT,1);
  }
  if(wifiNotice.length()){
    rect(8,240,463,23,C_AMBER);centerText(240,248,wifiNotice.c_str(),C_BG,1);
  }
}
static void paintSetupWifi(){
  rect(0,0,W,H,C_BG);rect(0,0,W,37,C_HEAD);
  text(6,10,"SETUP",C_TEXT,2);
  paintSetupTabs();
  // Vijf vaste actieposities; niet-toepasselijke opties zijn verborgen.
  static const char* actionNames[5]={"VERBINDEN","WIJZIG PW","X32 IP","SCAN","NIEUW"};
  for(int a=0;a<5;a++){
    if(!wifiActionAvailable(a))continue;
    int x=7+a*95;
    rect(x,41,91,28,C_HEAD);
    focusOutline(x,41,91,28,wifiActionFocus&&wifiAction==a,true);
    centerText(x+45,51,actionNames[a],C_TEXT,1);
  }
  int total=WIFI_PROFILES+scannedCount;
  for(int row=0;row<7;row++){
    int ix=wifiTop+row;if(ix>=total)break;
    int y=72+row*29;
    rect(10,y,460,27,C_PANEL);
    focusOutline(10,y,460,27,!setupTabFocus&&ix==wifiCursor,wifiActionFocus&&ix==wifiCursor);
    String name;String details;
    if(ix<WIFI_PROFILES){
      name=savedSsid[ix].isEmpty()?"(LEEG - CONFIGUREREN)":savedSsid[ix];
      details=(!savedSsid[ix].isEmpty()&&WiFi.status()==WL_CONNECTED&&WiFi.SSID()==savedSsid[ix])?"ACTIEF":"OPGESLAGEN";
    }else{
      int k=ix-WIFI_PROFILES;name=scannedSsid[k];
      int known=knownProfileFor(name);
      details=(WiFi.status()==WL_CONNECTED&&WiFi.SSID()==name)?"ACTIEF":(known>=0?"BEKEND":(scannedSecure[k]?"SLOT":"OPEN"));
    }
    name=wifiLabelDisplay(name);
    char prefix[12];snprintf(prefix,sizeof(prefix),"%c%d ",ix<WIFI_PROFILES?'P':'N',ix<WIFI_PROFILES?ix+1:ix-WIFI_PROFILES+1);
    text(17,y+9,prefix,C_MUTED,1);
    text(53,y+9,name.c_str(),C_TEXT,1);
    rect(359,y+3,101,21,C_HEAD);
    centerText(410,y+11,details.c_str(),details=="ACTIEF"?C_GREEN:C_TEXT,1);
  }
  rect(0,281,W,39,C_HEAD);
  if(setupTabFocus){
    text(9,291,"DRAAI: TAB  DRUK: OPEN",C_MUTED,1);
  }else if(wifiActionFocus){
    text(9,291,"NETWERK BEWERKEN - GROENE RAND",C_GREEN,1);
    text(9,305,"DRAAI: ACTIE  DRUK: UITVOER  HOUD: TERUG",C_MUTED,1);
  }else{
    text(9,291,"DRAAI: NETWERK  DRUK: SUBACTIES",C_MUTED,1);
    text(9,305,"HOUD: TERUG NAAR TABS",C_MUTED,1);
  }
  if(wifiNotice.length())text(9,277,wifiNotice.c_str(),C_AMBER,1);
}

static void enterSetupUI(){
  setupMode=true;volumeMode=false;setupSelected=0;setupTop=0;setupPage=0;focusedTab=0;setupTabFocus=true;
  channelActionFocus=false;channelAction=0;busActionFocus=false;wifiActionFocus=false;redraw=true;
}
static void leaveSetupUI(){
  if(otaInstalling)return;
  if(wifiEditor!=EDIT_NONE)cancelWifiEditor();
  if(otaAwaitConfirm||rollbackConfirm){otaAwaitConfirm=false;rollbackConfirm=false;otaLastStatus="ACTIE GEANNULEERD";}
  setupMode=false;setupTabFocus=false;volumeMode=false;lastAction=millis();redraw=true;
  if(x32Identified){if(!allLiveReceived())setLink(LINK_SYNCING);requestMixerState();subscribeMixerState();}
}
static void updateTouchFader(uint16_t y,uint32_t now,bool force){
  if(dragChannel<0)return;
  float p=constrain((238.0f-y)/142.0f,0.0f,1.0f);
  dragValue=p<=REF_DISPLAY?(p/REF_DISPLAY)*REF_OSC:REF_OSC+(p-REF_DISPLAY)/(1.0f-REF_DISPLAY)*(1.0f-REF_OSC);
  dragPending=true;
  if(force||now-dragSent>=50){changeLiveSend(dragChannel,dragValue);dragSent=now;dragPending=false;}
  lastAction=now;
}
void pollTouch() {
  if(!touchReady) { touchWasDown=false;dragChannel=-1; return; }
  uint32_t now=millis();if(now-lastTouchPoll<25)return;lastTouchPoll=now;
  bool down=touchPanel.Get_Touch();
  if(!down){
    if(dragChannel>=0&&!setupMode&&dragPending)changeLiveSend(dragChannel,dragValue);
    dragChannel=-1;dragPending=false;touchWasDown=false;return;
  }
  uint16_t x=touchPanel.touch.x[0],y=touchPanel.touch.y[0];
  if(x>=W||y>=H)return;
  if(touchWasDown){if(!setupMode&&dragChannel>=0)updateTouchFader(y,now,false);return;}
  touchWasDown=true;
  if(diagnostics||otaInstalling)return;
  if(!setupMode){
    if(y<34&&x>=410){enterSetupUI();return;}
    if(y>=40&&y<=282&&x>=5){
      int slot=(x-5)/79;
      if(slot<CHANNELS){int index=windowStart+slot,ch=channelAt(index);
        if(ch>=0){selected=index;lastAction=now;redraw=true;
          int localX=x-(5+slot*79);
          if(localX>=38&&localX<=72&&y>=96&&y<=238&&linkState==LINK_READY){dragChannel=ch;updateTouchFader(y,now,true);}
        }
      }
    }
    return;
  }
  if(y<37&&x>=412){leaveSetupUI();return;}
  if(wifiEditor!=EDIT_NONE){wifiEditorKey(x,y);return;}
  if(y<37){
    if(x>=76&&x<409){int tab=(x-76)/84;if(tab<4){otaAwaitConfirm=false;rollbackConfirm=false;openSetupTab(tab);}}
    return;
  }
  if(setupTabFocus)return;
  if(setupPage==3){
    if(x>=15&&x<465&&((y>=224&&y<256)||(y>=261&&y<293))){
      int choice=y>=261?1:0;
      if(choice!=otaCursor){otaAwaitConfirm=false;rollbackConfirm=false;otaCursor=choice;}
      updateMenuAction();redraw=true;
    }
    return;
  }
  if(setupPage==2){
    if(y>=41&&y<70){
      int action=(x-7)/95;
      if(x>=7&&action>=0&&action<5&&wifiActionAvailable(action)){
        wifiActionFocus=true;wifiAction=action;runWifiAction(action);redraw=true;
      }
      return;
    }
    if(y>=72&&y<275){
      int ix=wifiTop+(y-72)/29;
      if(ix<WIFI_PROFILES+scannedCount)selectWifiForEditing(ix);
    }
    return;
  }
  if(setupPage==1){
    if(y>=41&&y<68&&x>=374){busCursor=16;busFilterMode=(busFilterMode+1)%3;saveBusSettings();redraw=true;requestBusNames();return;}
    if(y>=70&&y<273){
      int bi=busTop+(y-70)/29;
      if(bi>=16)return;
      busCursor=bi;busActionFocus=false;
      if(x>=302&&x<372){
        toggleBusAllowed(bi);
      }else if(x>=375){chooseBus(bi+1);}
      redraw=true;
    }
    return;
  }
  if(y>=64 && y<274){
    int row=(y-64)/30;
    int ix=setupTop+row;
    if(ix<TOTAL_CHANNELS){
      setupSelected=ix;channelActionFocus=false;
      if(x>=260&&x<347)changeVisibility();
      else if(x>=359&&x<408)moveSetupChannel(-1);
      else if(x>=414&&x<463)moveSetupChannel(1);
      redraw=true;
    }
  }
}

// Directe tegelweergave: ook tijdens blokkerende HTTPS/flash-aanroepen.
static void otaProgress(const char* stage,size_t received,size_t total,bool failed){
  char detail[64];
  unsigned percent=total?(unsigned)((uint64_t)received*100/total):0;
  snprintf(detail,sizeof(detail),"%u%%  %lu / %lu BYTES",percent,(unsigned long)received,(unsigned long)total);
  for(stripTop=0;stripTop<H;stripTop+=TILE_H){
    rect(0,0,W,H,C_BG);rect(0,0,W,38,C_HEAD);
    text(12,12,"FIRMWARE UPDATE",C_TEXT,2);
    centerText(240,60,OTA_BASE_VERSION,C_MUTED,1);
    centerText(240,83,(otaCursor==1?rollbackVersion:otaLastTag).c_str(),C_TEXT,2);
    centerText(240,133,stage,failed?C_AMBER:C_TEXT,1);
    if(total){rect(20,174,440,24,C_BARBG);rect(20,174,(int)(440*percent/100),24,C_BLUE);centerText(240,211,detail,C_TEXT,1);}
    centerText(240,286,failed?"UPDATE AFGEBROKEN":"ONDERBREEK DE VOEDING NIET",C_AMBER,1);
    lcd.Fill_Colors(0,stripTop,W,TILE_H,tile);
  }
  redraw=true;
}
void render() {
  for (stripTop=0; stripTop<H; stripTop+=TILE_H) {
    if(diagnostics) paintDiagnostics();
    else if(wifiEditor!=EDIT_NONE)paintWifiKeyboard();
    else if(setupMode){if(setupTabFocus){rect(0,0,W,H,C_BG);rect(0,0,W,37,C_HEAD);text(6,10,"SETUP",C_TEXT,2);paintSetupTabs();centerText(240,125,"KIES EEN CATEGORIE",C_TEXT,2);centerText(240,165,"DRAAI: SELECTEER  DRUK: OPEN",C_MUTED,1);}else if(setupPage==3)paintSetupUpdate();else if(setupPage==2)paintSetupWifi();else if(setupPage==1)paintSetupBuses();else paintSetup();} else paintMix();
    lcd.Fill_Colors(0,stripTop,W,TILE_H,tile);
  }
  redraw=false;
}
void pollEncoder() {
  int16_t moves;
  portENTER_CRITICAL(&encoderMux);
  moves = encoderPending;
  encoderPending = 0;
  portEXIT_CRITICAL(&encoderMux);
  if (!moves) return;
  // Verwerk alle stappen, maar beperk OSC-verkeer tot één wijziging per loop.
  int steps = constrain((int)moves,-16,16);
  const uint32_t now = millis();
  if (diagnostics) {
    useCorrection = !useCorrection;
    redraw = true;
    Serial.println(useCorrection?"KLEURMODUS CORRECTIE":"KLEURMODUS RAW");
  } else if(wifiEditor!=EDIT_NONE){
    keyboardFocus=(keyboardFocus+steps)%46;
    if(keyboardFocus<0)keyboardFocus+=46;
    redraw=true;
  } else if(setupMode && setupTabFocus){
    stepTabFocus(steps);
  } else if(setupMode && setupPage==3){
    if(steps){otaCursor=constrain(otaCursor+steps,0,1);otaAwaitConfirm=false;rollbackConfirm=false;}
    redraw=true;
  } else if(setupMode && setupPage==2){
    if(wifiActionFocus){
      int direction=steps>0?1:-1;
      for(int i=0;i<abs(steps);i++){
        int next=nextWifiAction(wifiAction,direction);
        if(next<0){
          if(direction<0){wifiActionFocus=false;}
          else {wifiActionFocus=false;setupNextPage();}
          break;
        }
        wifiAction=next;
      }
      redraw=true;
      return;
    }

    const int total=WIFI_PROFILES+scannedCount;

    wifiCursor=constrain(wifiCursor+steps,0,max(0,total-1));
    if(wifiCursor<wifiTop)wifiTop=wifiCursor;
    if(wifiCursor>=wifiTop+7)wifiTop=wifiCursor-6;
    redraw=true;
  } else if (setupMode && setupPage==1) {
    if(busActionFocus){busAction=constrain((int)busAction+steps,0,3);redraw=true;return;}


    busCursor=constrain(busCursor+steps,0,18);
    if(busCursor>=0&&busCursor<16)manualBusTarget=busCursor;
    if(busCursor<busTop)busTop=busCursor;
    if(busCursor>=busTop+7&&busCursor<16)busTop=busCursor-6;
    redraw=true;
  } else if (setupMode) {


    if(channelActionFocus){
      channelAction=constrain((int)channelAction+steps,0,3);
    }else{
      setupSelected=constrain(setupSelected+steps,0,TOTAL_CHANNELS-1);
      if(setupSelected<setupTop)setupTop=setupSelected;
      if(setupSelected>=setupTop+7)setupTop=setupSelected-6;
    }
    redraw=true;
  } else if (volumeMode) {
    lastAction=now;
    int ch=channelAt(selected);
    if(ch>=0){float requested=constrain(liveSend[ch]+0.02f*steps,0.0f,1.0f);
      if(changeLiveSend(ch,requested))
        Serial.printf("CH%02d %s SEND = %.6f\n",ch+1,liveName[ch],(double)liveSend[ch]);}
  } else {
    selected=constrain(selected+steps,0,shownCount-1);
    const int previousWindow=windowStart;
    if(selected<windowStart)windowStart=selected;
    if(selected>=windowStart+CHANNELS)windowStart=selected-CHANNELS+1;
    lastAction=now;
    redraw=true;
    if(windowStart!=previousWindow && x32Identified){
      // Laad alleen de zichtbare zes kanalen: maximaal 13 OSC-subscriptions.
      // Eerder geladen kanalen blijven in de lokale cache beschikbaar.
      if(!allLiveReceived())setLink(LINK_SYNCING);
      requestMixerState();
      subscribeMixerState();
      lastLivePoll=now;
      Serial.printf("SCROLL VENSTER: CH%02d-CH%02d\n",windowStart+1,windowStart+CHANNELS);
    }
    int ch=channelAt(selected);if(ch>=0)Serial.printf("SELECTIE CH%02d %s\n",ch+1,liveName[ch]);
  }
}
static void setupSingleClick(){
  if(wifiEditor!=EDIT_NONE){keyboardActivate(keyboardFocus);return;}
  if(setupTabFocus){openSetupTab(focusedTab);return;}
  if(setupPage==3){
    updateMenuAction();
    return;
  }
  if(setupPage==2){wifiEncoderAction();return;}
  if(setupPage==1){busEncoderAction();return;}
  channelEncoderAction();
}
void pollButton() {
  uint32_t now=millis();
  bool raw=(digitalRead(SW_PIN)==LOW);
  if(raw!=lastRawPressed){lastRawPressed=raw;lastRawChange=now;}
  if(now-lastRawChange>=DEBOUNCE_MS && raw!=stablePressed){
    stablePressed=raw;
    if(stablePressed){pressStart=now;longHandled=false;}
    else if(!longHandled){
      if(diagnostics){diagnostics=false;redraw=true;lastAction=now;}
      else if(setupMode)setupSingleClick();
      else {volumeMode=!volumeMode;lastAction=now;redraw=true;
        Serial.println(volumeMode?"VOLUME MODUS":"SELECTIE MODUS");}
    }
  }
  if(!diagnostics && stablePressed && !longHandled && now-pressStart>=LONG_MS){
    longHandled=true;
    // Eén consistente terug-actie in alle editors en setupschermen.
    if(wifiEditor!=EDIT_NONE){cancelWifiEditor();return;}
    if(otaInstalling)return;
    dragChannel=-1;dragPending=false;
    if(!setupMode){enterSetupUI();return;}
    if(setupTabFocus){leaveSetupUI();return;}
    if(otaAwaitConfirm||rollbackConfirm){otaAwaitConfirm=false;rollbackConfirm=false;otaLastStatus="ACTIE GEANNULEERD";}
    if(busActionFocus){busActionFocus=false;redraw=true;return;}
    if(wifiActionFocus){wifiActionFocus=false;redraw=true;return;}
    if(channelActionFocus){channelActionFocus=false;redraw=true;return;}
    setupTabFocus=true;focusedTab=setupPage;redraw=true;
  }
}
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("BOOT %s: setup gestart\n",GB_VERSION);
  pinMode(CLK_PIN,INPUT_PULLUP);
  pinMode(DT_PIN,INPUT_PULLUP);
  pinMode(SW_PIN,INPUT); // Externe ca. 6.7k pull-up bevestigd
  encoderPreviousAB=(digitalRead(CLK_PIN)==HIGH?1u:0u)|
                    (digitalRead(DT_PIN)==HIGH?2u:0u);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT_PIN), onEncoderChange, CHANGE);
  Serial.println("=== PSRAM DIAGNOSE ===");
  Serial.printf("PSRAM aanwezig: %u bytes\n", (unsigned)ESP.getPsramSize());
  Serial.printf("PSRAM vrij: %u bytes\n", (unsigned)ESP.getFreePsram());
  Serial.printf("Vrij heapgeheugen: %u bytes\n", (unsigned)ESP.getFreeHeap());
  if (ESP.getPsramSize() == 0) {
    Serial.println("WAARSCHUWING: geen PSRAM gedetecteerd.");
    Serial.println("De Freenove ST77922-driver gebruikt ps_malloc voor rotatie 1.");
  }
  Serial.println("BOOT: display rotatie start");
  lcd.Set_Rotation(3);
  Serial.println("BOOT: display rotatie OK"); // 180 graden t.o.v. werkende v1.2 (landscape)
  // FNK0104N: touchcontroller (0x55) moet eerst uit hardware-reset.
  // Freenove ST77922_TOUCH::init() doet dezelfde reset, maar onze
  // oude v0.7 probeerde het adres voordat reset was uitgevoerd.
  pinMode(TOUCH_RST, OUTPUT);   // GPIO48, volgens Freenove-driver
  digitalWrite(TOUCH_RST, LOW);
  delay(120);
  digitalWrite(TOUCH_RST, HIGH);
  delay(200);
  pinMode(TOUCH_INT, INPUT);   // GPIO47

  // Wire.begin(38,39) wordt door ST77922_TOUCH-constructor uitgevoerd.
  Wire.beginTransmission(TOUCH_ADDR);
  uint8_t touchError = Wire.endTransmission();
  Serial.printf("I2C touch 0x%02X na reset: foutcode %u\n",
                (unsigned)TOUCH_ADDR, (unsigned)touchError);
  if (touchError == 0) {
    Serial.println("Touchcontroller antwoordt; Freenove-init starten...");
    touchPanel.init();         // Herhaalt hardware-reset + leest max_points
    touchPanel.Set_Rotation(3); // touch volgt 180 graden schermrotatie
    touchReady = (touchPanel.max_points > 0 &&
                  touchPanel.max_points <= MAX_TOUCH_POINTS);
    Serial.printf("Touchpunten max: %u; touch gereed: %s\n",
                  (unsigned)touchPanel.max_points,
                  touchReady ? "JA" : "NEE");
  } else {
    Serial.println("Geen I2C-antwoord na hardware-reset; touch blijft uit.");
    Serial.println("Controleer dan de GPIO38/39-bus en GPIO48 reset.");
  }
  Serial.printf("Na rotatie - PSRAM vrij: %u bytes\n", (unsigned)ESP.getFreePsram());
  randomSeed(micros());
  lastAction=millis();
  Serial.println("GOED BEDOELD UI TEST v0.7b TOUCH RESET");
  Serial.println("Start direct met de zes kanaalstroken.");
  Serial.printf("Display: %d x %d\n",lcd.Get_Width(),lcd.Get_Height());
  Serial.println("BOOT: bus NVS start");
  loadBusSettings();
  Serial.println("BOOT: bus NVS OK");
  Serial.println("BOOT: OSC paden start");
  initLivePaths();
  Serial.println("BOOT: OSC paden OK");
  Serial.println("BOOT: kanaal NVS start");
  loadChannels();
  Serial.println("BOOT: kanaal NVS OK");
  Serial.println("UI tekenen...");
  render();
  Serial.printf("UI getekend, PSRAM vrij: %u bytes\n", (unsigned)ESP.getFreePsram());
  Serial.println("BOOT: netwerk NVS start");
  loadNetworkSettings();
  Serial.println("BOOT: netwerk NVS OK");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.println("BOOT: wifi verbinden start");
  connectSavedWifi();
  Serial.println("BOOT: wifi verbinden OK");
  lastReconnectTry=millis();
  Serial.println("BOOT: partitiecontrole start");
  otaInspectPartitions(true);
  Serial.println("BOOT: partitiecontrole OK");
  Serial.printf("IEMC firmware %s: SETUP > UPDATE; ota partitions; github check\n",OTA_BASE_VERSION);
  Serial.println("v1.2p: TV-menu encoder: draai = navigeren, druk = kies, houd = terug");
  Serial.println("Typ status, probe, getsend of setsend 0.65");
}
void loop() {
  static bool firstLoop=true;
  if(firstLoop){firstLoop=false;Serial.println("BOOT: eerste loop gestart");}
  pollEncoder(); // Encoder eerst; interrupt verzamelt pulsen tijdens LCD/UDP.
  netLoop();
  serviceWifiScan();
  netSerial();
  pollButton();
  pollTouch(); // Touch actief in MIX en SETUP
  uint32_t now=millis();
  if(!diagnostics && volumeMode && !setupMode && now-lastAction>=TIMEOUT_MS) {
    volumeMode=false; redraw=true;
    Serial.println("5 SEC TIMEOUT -> SELECTIE");
  }
  // De daadwerkelijke inputmeter volgt OSC /meters/1.
  if(!diagnostics && !setupMode && now-lastVu>=METER_RENDER_MS){
    lastVu=now;
    serviceMeterAnimation();
  }
  if(redraw) render();
  delay(2);
}
