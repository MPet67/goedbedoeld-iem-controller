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
// Legacy setup AP is disabled. Configure Wi-Fi on the device via SETUP > WI-FI.
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
  // No preconfigured network or shared password. Configure a profile on device.
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
static constexpr const char* GB_VERSION="v2.0 RC";

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
  for(int i=0;i<LIVE_CHANNELS;++i){liveHasName[i]=false;liveHasSend[i]=false;liveHasColor[i]=false;liveColor[i]=0;}
  if(dropIdentity){x32Identified=false;lastXinfoReply=0;}
  lastSyncRequest=0;lastSubscribe=0;
  redraw=true;
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
static void netSerial(){
  while(Serial.available()){
    char ch=Serial.read();
    if(ch=='\n'||ch=='\r'){
      netCommand.trim();
      if(netCommand.length()){
        if(netCommand=="wifi setup"){
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
        } else Serial.println("Commando's: wifi setup | wifi list | status | probe | getsend | setsend 0.65 | colors | buses | bus 3 | scanbuses");
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
  text(205,12,busTitle,C_TEXT,1);
  text(398,12,linkLabel(),linkState==LINK_READY?C_GREEN:C_AMBER,1);
  for (int i=0;i<CHANNELS;++i) {
    const int ch=channelAt(windowStart+i);
    if(ch<0)continue;
    int x=5+i*79;
    rect(x,40,74,243,C_PANEL);
    border(x,40,74,243,(windowStart+i==selected?3:1),
           (windowStart+i==selected?(volumeMode?C_GREEN:C_BLUE):C_EDGE));
    // Kleurlabel komt uit X32 OSC, niet uit de kanaalnaam.
    uint8_t code=liveHasColor[ch]?liveColor[ch]:0;
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
  text(310,26,range,C_MUTED,1);
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
  static const int xs[3]={108,228,352};
  static const int ws[3]={114,117,119};
  static const char* names[3]={"KANALEN","IEM BUS","WI-FI"};
  for(int i=0;i<3;i++){
    rect(xs[i],2,ws[i],32,i==setupPage?C_PANEL:C_HEAD);
    focusOutline(xs[i],2,ws[i],32,setupTabFocus && focusedTab==i);
    centerText(xs[i]+ws[i]/2,12,names[i],C_TEXT,1);
  }
}
void paintSetup() {
  rect(0,0,W,H,C_BG);
  rect(0,0,W,37,C_HEAD);
  text(12,10,"SETUP",C_TEXT,2);
  paintSetupTabs();
  text(14,47,"KANAAL",C_MUTED,1);
  text(215,47,"ZICHTBAAR",C_MUTED,1);
  for(int row=0;row<7;row++){
    int ix=setupTop+row;if(ix>=TOTAL_CHANNELS)break;
    int y=64+row*30;
    rect(10,y,460,28,C_PANEL);focusOutline(10,y,460,28,ix==setupSelected,!(!channelActionFocus)&&ix==setupSelected);
    char info[48];int ch=userOrder[ix];
    snprintf(info,sizeof(info),"%02d CH%02d %-12.12s",ix+1,ch+1,liveName[ch]);
    text(18,y+10,info,C_TEXT,1);
    rect(260,y+4,84,20,C_HEAD);focusOutline(260,y+4,84,20,ix==setupSelected&&channelActionFocus&&channelAction==0,true);
    centerText(302,y+11,userVisible[ch]?"AAN":"UIT",userVisible[ch]?C_GREEN:C_MUTED,1);
    rect(359,y+4,45,20,C_HEAD);focusOutline(359,y+4,45,20,ix==setupSelected&&channelActionFocus&&channelAction==1,true);centerText(381,y+11,"UP",C_TEXT,1);
    rect(414,y+4,45,20,C_HEAD);focusOutline(414,y+4,45,20,ix==setupSelected&&channelActionFocus&&channelAction==2,true);centerText(436,y+11,"DN",C_TEXT,1);
  }
  rect(0,281,W,39,C_HEAD);
  if(!channelActionFocus){
    text(12,292,"DRAAI: KANAAL  DRUK: ACTIES",C_MUTED,1);
    text(12,305,setupTabFocus?"DRAAI RECHTS: OPTIES  LINKS: VORIG TAB":"EINDE LIJST: VOLGEND TAB",C_MUTED,1);
  }else{
    const char* actions[]={"AAN/UIT","OMHOOG","OMLAAG","VOLGEND TAB"};
    char nav[72];snprintf(nav,sizeof(nav),"ACTIE: %s",actions[channelAction]);
    text(12,292,nav,C_AMBER,1);
    text(12,305,"DRAAI: ACTIE  DRUK: UITVOEREN  HOUD: TERUG",C_MUTED,1);
  }
}

static void paintSetupBuses(){
  rect(0,0,W,H,C_BG);rect(0,0,W,37,C_HEAD);
  text(12,10,"SETUP",C_TEXT,2);
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
    const char* options[]={"AAN/UIT","KIES BUS","FILTER","VOLGEND TAB"};
    char footer[52];snprintf(footer,sizeof(footer),"BUS B%02d  ACTIE: %s",busCursor+1,options[busAction]);
    text(9,291,footer,C_AMBER,1);
    text(9,305,"DRAAI: ACTIE  DRUK: UITVOER  HOUD: BUS",C_MUTED,1);
  }else if(setupTabFocus){
    text(9,291,"TAB FOCUS: DRAAI  DRUK: OPEN",C_AMBER,1);
    text(9,305,"RECHTSOM: OPTIES",C_MUTED,1);
  }else{
    text(9,291,"DRAAI: BUS  DRUK: ACTIES",C_MUTED,1);
    text(9,305,"EINDE LIJST: VOLGEND TAB",C_MUTED,1);
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
static void openSetupTab(uint8_t tab){
  if(tab>2)return;
  setupPage=tab;focusedTab=tab;setupTabFocus=true;
  channelActionFocus=false;busActionFocus=false;wifiActionFocus=false;
  if(tab==0){setupSelected=0;setupTop=0;}
  else if(tab==1){busCursor=selectedBus-1;busTop=constrain(busCursor-3,0,9);requestBusNames();}
  else {wifiCursor=activeWifi;wifiTop=0;beginWifiScan();}
  redraw=true;
  Serial.printf("SETUP TAB %u (focus)\n",(unsigned)setupPage);
}
static void focusAdjacentTab(int direction){
  int next=constrain((int)setupPage+direction,0,2);
  if(next!=setupPage)openSetupTab((uint8_t)next);
  else {focusedTab=setupPage;setupTabFocus=true;redraw=true;}
}
static void setupNextPage(){focusAdjacentTab(+1);}
static void stepTabFocus(int steps){
  if(steps>0){setupTabFocus=false;redraw=true;return;}
  if(steps<0)focusAdjacentTab(-1);
}
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
  text(12,10,"SETUP",C_TEXT,2);
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
    text(9,291,"TAB: DRAAI RECHTS VOOR NETWERKEN",C_MUTED,1);
  }else if(wifiActionFocus){
    text(9,291,"NETWERK BEWERKEN - GROENE RAND",C_GREEN,1);
    text(9,305,"DRAAI: ACTIE  DRUK: UITVOER  HOUD: TERUG",C_MUTED,1);
  }else{
    text(9,291,"DRAAI: NETWERK  DRUK: SUBACTIES",C_MUTED,1);
    text(9,305,"NA LAATSTE NETWERK: VOLGEND TAB",C_MUTED,1);
  }
  if(wifiNotice.length())text(9,277,wifiNotice.c_str(),C_AMBER,1);
}

void pollTouch() {
  if(!touchReady) { touchWasDown=false; return; }
  uint32_t now=millis();if(now-lastTouchPoll<35)return;lastTouchPoll=now;
  bool down=touchPanel.Get_Touch();
  if(!down){touchWasDown=false;return;}
  uint16_t x=touchPanel.touch.x[0],y=touchPanel.touch.y[0];
  if(x>=W||y>=H||touchWasDown)return;
  touchWasDown=true;
  if(!setupMode){
    if(y>=40&&y<=282){
      int slot=(x-5)/79;
      if(x>=5&&slot>=0&&slot<CHANNELS){
        int index=windowStart+slot, ch=channelAt(index);
        if(ch>=0){
          selected=index;lastAction=now;redraw=true;
          int localX=x-(5+slot*79);
          if(localX>=38&&localX<=72&&y>=96&&y<=238&&linkState==LINK_READY){
            // Touch mirrors the existing fader, translating the visible -18dB scale.
            float p=constrain((238.0f-y)/(238.0f-96.0f),0.0f,1.0f);
            float value=p<=REF_DISPLAY?(p/REF_DISPLAY)*REF_OSC:
              REF_OSC+(p-REF_DISPLAY)/(1.0f-REF_DISPLAY)*(1.0f-REF_OSC);
            changeLiveSend(ch,value);lastAction=now;
          }
        }
      }
    }
    return;
  }
  if(wifiEditor!=EDIT_NONE){wifiEditorKey(x,y);return;}
  if(y<37){
    if(x>=108&&x<223)openSetupTab(0);
    else if(x>=228&&x<346)openSetupTab(1);
    else if(x>=352)openSetupTab(2);
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

void render() {
  for (stripTop=0; stripTop<H; stripTop+=TILE_H) {
    if(diagnostics) paintDiagnostics();
    else if(wifiEditor!=EDIT_NONE)paintWifiKeyboard();
    else if(setupMode){if(setupPage==2)paintSetupWifi();else if(setupPage==1)paintSetupBuses();else paintSetup();} else paintMix();
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
    if(wifiCursor==0 && steps<0){setupTabFocus=true;focusedTab=setupPage;redraw=true;return;}
    const int total=WIFI_PROFILES+scannedCount;
    if(steps>0 && wifiCursor>=total-1){setupNextPage();return;}
    wifiCursor=constrain(wifiCursor+steps,0,max(0,total-1));
    if(wifiCursor<wifiTop)wifiTop=wifiCursor;
    if(wifiCursor>=wifiTop+7)wifiTop=wifiCursor-6;
    redraw=true;
  } else if (setupMode && setupPage==1) {
    if(busActionFocus){busAction=constrain((int)busAction+steps,0,3);redraw=true;return;}
    if(busCursor==0 && steps<0){setupTabFocus=true;focusedTab=setupPage;redraw=true;return;}
    if(steps>0 && busCursor>=18){setupNextPage();return;}
    busCursor=constrain(busCursor+steps,0,18);
    if(busCursor>=0&&busCursor<16)manualBusTarget=busCursor;
    if(busCursor<busTop)busTop=busCursor;
    if(busCursor>=busTop+7&&busCursor<16)busTop=busCursor-6;
    redraw=true;
  } else if (setupMode) {
    if(!channelActionFocus && setupSelected==0 && steps<0){setupTabFocus=true;focusedTab=setupPage;redraw=true;return;}
    if(!channelActionFocus && setupSelected==TOTAL_CHANNELS-1 && steps>0){setupNextPage();return;}
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
    if(setupMode && setupTabFocus){setupTabFocus=false;redraw=true;return;}
    if(setupMode && setupPage==1 && busActionFocus){busActionFocus=false;redraw=true;return;}
    if(setupMode && setupPage==2 && wifiActionFocus){wifiActionFocus=false;redraw=true;return;}
    if(setupMode && setupPage==0 && channelActionFocus){
      channelActionFocus=false;redraw=true;return;
    }
    setupMode=!setupMode; volumeMode=false;
    if(setupMode){setupSelected=0;setupTop=0;setupPage=0;
      channelActionFocus=false;channelAction=0;busActionFocus=false;wifiActionFocus=false;setupTabFocus=true;focusedTab=0;}
    else if(x32Identified){if(!allLiveReceived())setLink(LINK_SYNCING);
      requestMixerState();subscribeMixerState();}
    touchWasDown=false;lastAction=now;redraw=true;
    Serial.println(setupMode?"SETUP OPEN":"SETUP GESLOTEN");
  }
}
void setup() {
  Serial.begin(115200);
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
  lcd.Set_Rotation(3); // 180 graden t.o.v. werkende v1.2 (landscape)
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
  loadBusSettings();
  initLivePaths();
  loadChannels();
  Serial.println("UI tekenen...");
  render();
  Serial.printf("UI getekend, PSRAM vrij: %u bytes\n", (unsigned)ESP.getFreePsram());
  loadNetworkSettings();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  connectSavedWifi();
  lastReconnectTry=millis();
  Serial.println("v1.2p: TV-menu encoder: draai = navigeren, druk = kies, houd = terug");
  Serial.println("Typ status, probe, getsend of setsend 0.65");
}
void loop() {
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
