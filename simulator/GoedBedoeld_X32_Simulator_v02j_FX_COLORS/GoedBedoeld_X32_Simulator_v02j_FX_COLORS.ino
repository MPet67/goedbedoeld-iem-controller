/* Goed Bedoeld X32 OSC Simulator v0.2j AUTO COLORS + FX RETURN - ESP32-S3 Mini
   Test-only: partial X32 emulator; not a full mixer or guaranteed MX-Q compatibility.
   Network default: ESP32 creates AP GB-X32-SIM with a locally configured password; IP 192.168.4.1
   OSC UDP 10023; serial 115200. Arduino ESP32 core 3.x.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// Choose false and supply credentials to join an existing Wi-Fi network.
static const bool USE_EXISTING_WIFI = false;
// Local credentials are intentionally NOT part of the public repository.
// Copy local_secrets.example.h to local_secrets.h and set a unique AP password.
#include "local_secrets.h"
static const char *WIFI_SSID = GB_WIFI_SSID;
static const char *WIFI_PASS = GB_WIFI_PASSWORD;
static const char *AP_SSID = "GB-X32-SIM";
static const char *AP_PASS = GB_SIM_AP_PASSWORD; // >=8 characters
static const uint16_t OSC_PORT = 10023;

WiFiUDP udp;
struct OscSubscriber { IPAddress ip; uint16_t port=0; uint32_t expires=0; bool used=false; };
OscSubscriber clients[4];
char channelNames[32][13];
char busNames[16][13];
float sends[32][16];
uint8_t sendOn[32][16];
float chanFaders[32];
uint8_t chanOn[32];
float busFaders[16];
float busPans[16];
// Stereo bus pairs 1-2, 3-4, ..., 15-16 (0 = unlinked).
uint8_t busLinked[8] = {0};
uint8_t busOn[16];
static uint8_t rx[1024];
static uint8_t tx[1024];
// Short-lived, bounded OSC subscriptions. Expiration after 10s.
struct PollSubscription {
  IPAddress ip; uint16_t port=0; uint32_t expires=0, nextSend=0;
  uint16_t intervalMs=50; bool active=false, meter=false;
  uint8_t meterGroup=0, meterChan=0, meterAux=0;
  char path[88]={0};
};
PollSubscription subscriptions[28];
uint8_t channelColors[32],channelIcons[32],busColors[16],busIcons[16];
float sendPans[32][16];
// X32 AUX inputs: 01..08 (six auxiliary inputs plus USB L/R).
char auxNames[8][13];
float auxFaders[8], auxSends[8][16], auxPans[8][16];
uint8_t auxOn[8], auxSendOn[8][16], auxColors[8], auxIcons[8];
// FX returns can also appear during MX-Q parameter synchronization.
char fxNames[8][13];
float fxFaders[8], fxSends[8][16], fxPans[8][16];
uint8_t fxOn[8], fxSendOn[8][16], fxColors[8], fxIcons[8];
static uint32_t requestCount=0,replyCount=0,unsupportedCount=0;
static uint32_t diagnosticCount=0;
static void traceRequest(const char* path,const char* tags,const char* firstStr) {
  // Trace at most 100 protocol requests to avoid swamping the serial monitor.
  if(diagnosticCount>=100) return;
  if(strcmp(path,"/node")==0 || strcmp(path,"/subscribe")==0 ||
     strcmp(path,"/unsubscribe")==0 || strcmp(path,"/renew")==0 ||
     strncmp(path,"/ch/0/",6)==0) {
     Serial.printf("OSC RX %lu: %s tags=%s arg=%s\n",
                   (unsigned long)++diagnosticCount,path,tags,
                   firstStr[0]?firstStr:"<geen string>");
  }
}


// --- OSC codec: 4-byte padded strings, 32-bit network byte order ---
static size_t putString(uint8_t *dst, size_t max, size_t pos, const char *s) {
  size_t n = strlen(s) + 1;
  size_t aligned = (n + 3) & ~size_t(3);
  if (pos + aligned > max) return max + 1;
  memcpy(dst+pos, s, n);
  memset(dst+pos+n, 0, aligned-n);
  return pos+aligned;
}
static size_t putU32(uint8_t *dst, size_t max, size_t pos, uint32_t v) {
  if(pos+4>max) return max+1;
  dst[pos]=(uint8_t)(v>>24); dst[pos+1]=(uint8_t)(v>>16);
  dst[pos+2]=(uint8_t)(v>>8); dst[pos+3]=(uint8_t)v;
  return pos+4;
}
static uint32_t getU32(const uint8_t *src) {
  return ((uint32_t)src[0]<<24)|((uint32_t)src[1]<<16)|((uint32_t)src[2]<<8)|src[3];
}
static size_t getString(const uint8_t *buf,size_t len,size_t at,char *out,size_t outSize) {
  if(at>=len || !outSize) return len+1;
  size_t begin=at;
  while(at<len && buf[at]) at++;
  if(at>=len) return len+1;
  size_t n=min(at-begin,outSize-1);
  memcpy(out,buf+begin,n); out[n]=0;
  return (at+4)&~size_t(3);
}
static void sendPacket(IPAddress ip, uint16_t port, size_t len) {
  if(len<=sizeof(tx)) replyCount++;
  if(!port || len>sizeof(tx)) return;
  if(udp.beginPacket(ip,port)) {udp.write(tx,len); udp.endPacket();}
}
static size_t startMsg(const char *path, const char *tags) {
  size_t p=putString(tx,sizeof(tx),0,path);
  return p<=sizeof(tx)?putString(tx,sizeof(tx),p,tags):sizeof(tx)+1;
}
static void replyStr(IPAddress ip,uint16_t port,const char *path,const char *s) {
  size_t p=startMsg(path,",s"); p=putString(tx,sizeof(tx),p,s); sendPacket(ip,port,p);
}
static void replyFloat(IPAddress ip,uint16_t port,const char *path,float val) {
  size_t p=startMsg(path,",f"); uint32_t u; memcpy(&u,&val,4);
  p=putU32(tx,sizeof(tx),p,u); sendPacket(ip,port,p);
}
static void replyInt(IPAddress ip,uint16_t port,const char *path,int val) {
  size_t p=startMsg(path,",i"); p=putU32(tx,sizeof(tx),p,(uint32_t)val); sendPacket(ip,port,p);
}
static void replyInfo(IPAddress ip,uint16_t port,bool extended) {
  char ipText[20]; snprintf(ipText,sizeof(ipText),"%s",(USE_EXISTING_WIFI?WiFi.localIP():WiFi.softAPIP()).toString().c_str());
  size_t p=startMsg(extended?"/xinfo":"/info",",ssss");
  if(extended) {
    p=putString(tx,sizeof(tx),p,ipText);
    p=putString(tx,sizeof(tx),p,"GB-X32-SIM");
    p=putString(tx,sizeof(tx),p,"X32RACK");
    p=putString(tx,sizeof(tx),p,"4.06");
  } else {
    p=putString(tx,sizeof(tx),p,"V2.05");
    p=putString(tx,sizeof(tx),p,"osc-server");
    p=putString(tx,sizeof(tx),p,"X32RACK");
    p=putString(tx,sizeof(tx),p,"4.06");
  }
  sendPacket(ip,port,p);
}
static void replyStatus(IPAddress ip,uint16_t port) {
  String ipString=(USE_EXISTING_WIFI?WiFi.localIP():WiFi.softAPIP()).toString();
  size_t p=startMsg("/status",",sss");
  p=putString(tx,sizeof(tx),p,"active");
  p=putString(tx,sizeof(tx),p,ipString.c_str());
  p=putString(tx,sizeof(tx),p,"osc-server");
  sendPacket(ip,port,p);
}
static void registerClient(IPAddress ip,uint16_t port) {
  int slot=-1;
  for(int i=0;i<4;i++) if(clients[i].used && clients[i].ip==ip && clients[i].port==port) slot=i;
  if(slot<0) for(int i=0;i<4;i++) if(!clients[i].used || (int32_t)(millis()-clients[i].expires)>=0) {slot=i;break;}
  if(slot<0) {Serial.println("XREMOTE: alle 4 slots bezet");return;}
  clients[slot]={ip,port,millis()+10000,true};
}
static void broadcastStr(IPAddress origin,uint16_t port,const char *path,const char *value) {
  for(auto &c:clients) if(c.used && (int32_t)(millis()-c.expires)<0 && !(c.ip==origin&&c.port==port)) replyStr(c.ip,c.port,path,value);
}
static void broadcastFloat(IPAddress origin,uint16_t port,const char *path,float value) {
  for(auto &c:clients) if(c.used && (int32_t)(millis()-c.expires)<0 && !(c.ip==origin&&c.port==port)) replyFloat(c.ip,c.port,path,value);
}
static void broadcastInt(IPAddress origin,uint16_t port,const char *path,int value) {
  for(auto &c:clients) if(c.used && (int32_t)(millis()-c.expires)<0 && !(c.ip==origin&&c.port==port)) replyInt(c.ip,c.port,path,value);
}
// Auto-classification by channel label. X32 OSC color codes:
// VOCALS = BLUE (4), INSTRUMENTS = GREEN (2), EFFECTS = MAGENTA (5).
// White lettering is a UI rendering choice on the Freenove; OSC only carries a color code.
static uint8_t classifyChannelColor(const char *label) {
  String upper(label); upper.toUpperCase();
  // Effects first so e.g. FX VOC is treated as an effect.
  const char *effects[]={"FX", "REVERB", "DELAY", "ECHO"};
  for(const char *term:effects) if(upper.indexOf(term)>=0) return 5;
  const char *vocals[]={"VOC", "ZANG", "LEAD", "BACKING", "BACK VOC", "BV"};
  for(const char *term:vocals) if(upper.indexOf(term)>=0) return 4;
  return 2; // All other channels = instruments.
}
static void updateAutoChannelColor(int channel) {
  int i=channel-1;
  uint8_t code=classifyChannelColor(channelNames[i]);
  if(channelColors[i]==code)return;
  channelColors[i]=code;
  char path[44]; snprintf(path,sizeof(path),"/ch/%02d/config/color",channel);
  broadcastInt(IPAddress(0,0,0,0),0,path,code);
  for(auto &sub:subscriptions) if(sub.active && !sub.meter && strcmp(sub.path,path)==0 &&
      (int32_t)(millis()-sub.expires)<0) replyInt(sub.ip,sub.port,path,code);
  Serial.printf("AUTO COLOR CH%02d (%s): %u\n",channel,channelNames[i],code);
}
static void seedState() {
  const char *initChannels[]={"KICK","SNARE","HIHAT","BASS","GITAAR","KEYS","ZANG","BACKING","CLICK","FX REVERB"};
  const char *initBuses[]={"FX1","FX2","IEM MAURICE","IEM STEIJN","IEM MICHEL","IEM JULIET","IEM MATZ","IEM LUC"};
  for(int ch=0;ch<32;ch++) {
    snprintf(channelNames[ch],13,"CH %02d",ch+1);
    if(ch<10) snprintf(channelNames[ch],13,"%s",initChannels[ch]);
    chanFaders[ch]=0.7f; chanOn[ch]=1; channelColors[ch]=classifyChannelColor(channelNames[ch]);channelIcons[ch]=1;
    for(int bus=0;bus<16;bus++) { sends[ch][bus]=0.5f; sendOn[ch][bus]=1;sendPans[ch][bus]=0.5f; }
  }
  for(int i=0;i<8;i++) {
    snprintf(auxNames[i],13,i<6?"AUX %02d":"USB %02d",i<6?i+1:i-5);
    snprintf(fxNames[i],13,"FX RTN %02d",i+1);
    auxFaders[i]=0.7f; fxFaders[i]=0.7f;
    auxOn[i]=fxOn[i]=1;
    auxColors[i]=0; fxColors[i]=5; // FX returns = MAGENTA, ongeacht naam
    auxIcons[i]=fxIcons[i]=1;
    for(int b=0;b<16;b++) {
      auxSends[i][b]=fxSends[i][b]=0.5f;
      auxPans[i][b]=fxPans[i][b]=0.5f;
      auxSendOn[i][b]=fxSendOn[i][b]=1;
    }
  }
  for(int b=0;b<16;b++) {
    snprintf(busNames[b],13,"BUS %02d",b+1);
    if(b<8) snprintf(busNames[b],13,"%s",initBuses[b]);
    busFaders[b]=0.75f; busPans[b]=0.5f; busOn[b]=1;busColors[b]=0;busIcons[b]=1;
  }
}
// Recognized paths: /ch/NN/config/name; /ch/NN/mix/BB/{level,on};
//                  /ch/NN/mix/{fader,on}; /bus/BB/config/name;
//                  /bus/BB/mix/{fader,on}.
static void handleParam(const char *path,char type,const char *strVal,float fval,int ival,
                        IPAddress ip,uint16_t port) {
  // MX-Q 2.2 sometimes subscribes to /ch/0/config/name during startup.
  // X32 channels are 01..32: do NOT alias zero to channel 01.
  // This explicit diagnostic response tests whether MX-Q needs a response
  // before requesting the real channels.
  if(strcmp(path,"/ch/0/config/name")==0) {
    static uint32_t probeReplies=0;
    replyStr(ip,port,path,"");
    if(probeReplies++ < 5) Serial.println("COMPAT PROBE: /ch/0/config/name => empty string");
    return;
  }
  // X32 bus stereo-link configuration is an OSC enum (integer 0 or 1).
  // Explicitly restrict to the eight valid odd-even pairs.
  unsigned left=0,right=0;
  if(sscanf(path,"/config/buslink/%u-%u",&left,&right)==2 &&
     left>=1 && left<=15 && (left%2)==1 && right==left+1) {
    int pair=(left-1)/2;
    if(type=='i') {
      busLinked[pair]=(ival!=0)?1:0;
      broadcastInt(ip,port,path,busLinked[pair]);
      Serial.printf("BUSLINK %u-%u = %s\n",left,right,busLinked[pair]?"ON":"OFF");
    }
    replyInt(ip,port,path,busLinked[pair]);
    return;
  }
  unsigned ch=0,bus=0; char tail[64]={0};
  char canonical[100]; snprintf(canonical,sizeof(canonical),"%s",path);
  if(sscanf(path,"/ch/%u/mix/%u/%63s",&ch,&bus,tail)==3 && ch>=1&&ch<=32&&bus>=1&&bus<=16) {
    const int c=ch-1,b=bus-1;
    if(strcmp(tail,"level")==0) {
      if(type=='f') {sends[c][b]=constrain(fval,0.0f,1.0f);broadcastFloat(ip,port,path,sends[c][b]);}
      replyFloat(ip,port,path,sends[c][b]);return;
    }
    if(strcmp(tail,"pan")==0) {
      if(type=='f') {sendPans[c][b]=constrain(fval,0.0f,1.0f);broadcastFloat(ip,port,path,sendPans[c][b]);}
      replyFloat(ip,port,path,sendPans[c][b]);return;
    }
    if(strcmp(tail,"on")==0) {
      if(type=='i') {sendOn[c][b]=ival?1:0;broadcastInt(ip,port,path,sendOn[c][b]);}
      replyInt(ip,port,path,sendOn[c][b]);return;
    }
  }
  if(sscanf(path,"/ch/%u/config/%63s",&ch,tail)==2 && ch>=1&&ch<=32) {
    if(strcmp(tail,"name")==0) {
      if(type=='s') {snprintf(channelNames[ch-1],13,"%.12s",strVal);broadcastStr(ip,port,path,channelNames[ch-1]);updateAutoChannelColor((int)ch);}
      replyStr(ip,port,path,channelNames[ch-1]);return;
    }
    if(strcmp(tail,"color")==0 || strcmp(tail,"icon")==0 || strcmp(tail,"source")==0) {
      uint8_t *p=strcmp(tail,"color")==0?&channelColors[ch-1]:&channelIcons[ch-1];
      if(strcmp(tail,"source")==0){replyInt(ip,port,path,ch-1);return;}
      // Channel color is derived from name, not manually configured.
      if(type=='i' && strcmp(tail,"color")!=0){*p=(uint8_t)ival;broadcastInt(ip,port,path,*p);}
      replyInt(ip,port,path,*p);return;
    }
  }
  if(sscanf(path,"/bus/%u/config/%63s",&bus,tail)==2 && bus>=1&&bus<=16) {
    if(strcmp(tail,"name")==0) {
      if(type=='s') {snprintf(busNames[bus-1],13,"%.12s",strVal);broadcastStr(ip,port,path,busNames[bus-1]);}
      replyStr(ip,port,path,busNames[bus-1]);return;
    }
    if(strcmp(tail,"color")==0 || strcmp(tail,"icon")==0) {
      uint8_t *p=strcmp(tail,"color")==0?&busColors[bus-1]:&busIcons[bus-1];
      if(type=='i'){*p=(uint8_t)ival;broadcastInt(ip,port,path,*p);}
      replyInt(ip,port,path,*p);return;
    }
  }
  if(sscanf(path,"/ch/%u/mix/%63s",&ch,tail)==2 && ch>=1&&ch<=32) {
    if(strcmp(tail,"fader")==0) {if(type=='f') {chanFaders[ch-1]=constrain(fval,0.0f,1.0f);broadcastFloat(ip,port,path,chanFaders[ch-1]);}replyFloat(ip,port,path,chanFaders[ch-1]);return;}
    if(strcmp(tail,"on")==0) {if(type=='i') {chanOn[ch-1]=ival?1:0;broadcastInt(ip,port,path,chanOn[ch-1]);}replyInt(ip,port,path,chanOn[ch-1]);return;}
  }
  if(sscanf(path,"/bus/%u/mix/%63s",&bus,tail)==2 && bus>=1&&bus<=16) {
    // X32 mix bus master panorama; 0.5=center. Read and write OSC float.
    if(strcmp(tail,"pan")==0) {
      if(type=='f') {busPans[bus-1]=constrain(fval,0.0f,1.0f);broadcastFloat(ip,port,path,busPans[bus-1]);}
      replyFloat(ip,port,path,busPans[bus-1]);return;
    }
    if(strcmp(tail,"fader")==0) {if(type=='f') {busFaders[bus-1]=constrain(fval,0.0f,1.0f);broadcastFloat(ip,port,path,busFaders[bus-1]);}replyFloat(ip,port,path,busFaders[bus-1]);return;}
    if(strcmp(tail,"on")==0) {if(type=='i') {busOn[bus-1]=ival?1:0;broadcastInt(ip,port,path,busOn[bus-1]);}replyInt(ip,port,path,busOn[bus-1]);return;}
  }
  // AUX inputs and FX returns are actual X32 input strips and are requested
  // by MX-Q during its full parameter transfer, not just during IEM mixing.
  // Keep separate channel state; never alias an AUX strip to input CH01.
  unsigned idx=0, sendBus=0;
  char prop[64]={0};
  const bool isAux = strncmp(path,"/auxin/",7)==0;
  const bool isFx  = strncmp(path,"/fxrtn/",7)==0;
  if(isAux || isFx) {
    char (*names)[13]=isAux?auxNames:fxNames;
    float *faders=isAux?auxFaders:fxFaders;
    uint8_t *on=isAux?auxOn:fxOn;
    uint8_t *colors=isAux?auxColors:fxColors;
    uint8_t *icons=isAux?auxIcons:fxIcons;
    float (*levels)[16]=isAux?auxSends:fxSends;
    float (*pans)[16]=isAux?auxPans:fxPans;
    uint8_t (*sendEnabled)[16]=isAux?auxSendOn:fxSendOn;
    const char *rest=path+7;
    // Require a two-digit index: /auxin/01 ... /fxrtn/08.
    if(rest[0]>='0'&&rest[0]<='9'&&rest[1]>='0'&&rest[1]<='9'&&rest[2]=='/') {
      idx=(rest[0]-'0')*10+(rest[1]-'0');
      if(idx>=1&&idx<=8) {
        const int i=idx-1;
        const char *suffix=rest+3;
        if(strcmp(suffix,"config/name")==0) {
          if(type=='s') {snprintf(names[i],13,"%.12s",strVal);broadcastStr(ip,port,path,names[i]);}
          replyStr(ip,port,path,names[i]);return;
        }
        if(strcmp(suffix,"config/color")==0 || strcmp(suffix,"config/icon")==0 || strcmp(suffix,"config/source")==0) {
          if(strcmp(suffix,"config/source")==0) {replyInt(ip,port,path,i);return;}
          uint8_t &v=(strcmp(suffix,"config/color")==0)?colors[i]:icons[i];
          if(type=='i') {v=(uint8_t)ival;broadcastInt(ip,port,path,v);}
          replyInt(ip,port,path,v);return;
        }
        if(strcmp(suffix,"mix/fader")==0) {
          if(type=='f'){faders[i]=constrain(fval,0.0f,1.0f);broadcastFloat(ip,port,path,faders[i]);}
          replyFloat(ip,port,path,faders[i]);return;
        }
        if(strcmp(suffix,"mix/on")==0) {
          if(type=='i'){on[i]=ival?1:0;broadcastInt(ip,port,path,on[i]);}
          replyInt(ip,port,path,on[i]);return;
        }
        if(strcmp(suffix,"mix/pan")==0) {replyFloat(ip,port,path,0.5f);return;}
        if(sscanf(suffix,"mix/%u/%63s",&sendBus,prop)==2 && sendBus>=1&&sendBus<=16) {
          const int b=sendBus-1;
          if(strcmp(prop,"level")==0) {
            if(type=='f'){levels[i][b]=constrain(fval,0.0f,1.0f);broadcastFloat(ip,port,path,levels[i][b]);}
            replyFloat(ip,port,path,levels[i][b]);return;
          }
          if(strcmp(prop,"on")==0) {
            if(type=='i'){sendEnabled[i][b]=ival?1:0;broadcastInt(ip,port,path,sendEnabled[i][b]);}
            replyInt(ip,port,path,sendEnabled[i][b]);return;
          }
          if(strcmp(prop,"pan")==0) {
            if(type=='f'){pans[i][b]=constrain(fval,0.0f,1.0f);broadcastFloat(ip,port,path,pans[i][b]);}
            replyFloat(ip,port,path,pans[i][b]);return;
          }
        }
      }
    }
  }
  static uint16_t unknownCount=0;
  if(unknownCount++<50) Serial.printf("Niet ondersteund: %s (%c)\n",path,type?type:'?');
}
// OSC strings have zero padding, and numeric arguments are big-endian.
// Meter payload is exceptional: little-endian count + little-endian float32.
static size_t putLE32(uint8_t *dst,size_t max,size_t pos,uint32_t v) {
  if(pos+4>max)return max+1;
  for(int i=0;i<4;i++) dst[pos+i]=(uint8_t)(v>>(8*i));
  return pos+4;
}
static void replyNode(IPAddress ip,uint16_t port,const char *query) {
  static unsigned traceNodeCount=0;
  char q[100];snprintf(q,sizeof(q),"%s",query);
  while(q[0]=='/') memmove(q,q+1,strlen(q));
  char value[300];value[0]=0;
  int idx=0,sub=0;
  unsigned linkA=0,linkB=0;
  if(sscanf(q,"config/buslink/%u-%u",&linkA,&linkB)==2 &&
     linkA>=1 && linkA<=15 && (linkA%2)==1 && linkB==linkA+1) {
    snprintf(value,sizeof(value),"/config/buslink/%u-%u %s\n",linkA,linkB,
             busLinked[(linkA-1)/2]?"ON":"OFF");
  } else if(sscanf(q,"ch/%d/config",&idx)==1 && strstr(q,"/config") && idx>=1&&idx<=32) {
    snprintf(value,sizeof(value),"/ch/%02d/config %s %u %u %d\n",idx,channelNames[idx-1],channelIcons[idx-1],channelColors[idx-1],idx-1);
  } else if(sscanf(q,"bus/%d/config",&idx)==1 && strstr(q,"/config") && idx>=1&&idx<=16) {
    snprintf(value,sizeof(value),"/bus/%02d/config %s %u %u\n",idx,busNames[idx-1],busIcons[idx-1],busColors[idx-1]);
  } else if(sscanf(q,"ch/%d/mix/%d",&idx,&sub)==2 && idx>=1&&idx<=32&&sub>=1&&sub<=16) {
    snprintf(value,sizeof(value),"/ch/%02d/mix/%02d %s %.5f %.5f\n",idx,sub,sendOn[idx-1][sub-1]?"ON":"OFF",sends[idx-1][sub-1],sendPans[idx-1][sub-1]);
  } else if(sscanf(q,"ch/%d/mix",&idx)==1 && idx>=1&&idx<=32&&strstr(q,"/mix")) {
    snprintf(value,sizeof(value),"/ch/%02d/mix %s %.5f OFF 0.50000 0.50000\n",idx,chanOn[idx-1]?"ON":"OFF",chanFaders[idx-1]);
  } else if((strncmp(q,"auxin/",6)==0 || strncmp(q,"fxrtn/",6)==0) &&
            sscanf(q+6,"%d/config",&idx)==1 && idx>=1&&idx<=8 && strstr(q,"/config")) {
    const bool a=strncmp(q,"auxin/",6)==0;
    snprintf(value,sizeof(value),"/%s/%02d/config %s %u %u %d\n",
             a?"auxin":"fxrtn",idx,a?auxNames[idx-1]:fxNames[idx-1],
             a?auxIcons[idx-1]:fxIcons[idx-1],
             a?auxColors[idx-1]:fxColors[idx-1],idx-1);
  } else if(sscanf(q,"bus/%d/mix",&idx)==1 && idx>=1&&idx<=16&&strstr(q,"/mix")) {
    snprintf(value,sizeof(value),"/bus/%02d/mix %s %.5f %.5f\n",idx,busOn[idx-1]?"ON":"OFF",busFaders[idx-1],busPans[idx-1]);
  } else {
    // A diagnostic placeholder is not enough for MX-Q's full state sync.
    // We return the queried path and a newline, and log the gap.
    snprintf(value,sizeof(value),"/%s\n",q);
    static unsigned unknownNodes=0;
    if(unknownNodes++<45)Serial.printf("NODE nog niet gesimuleerd: %s\n",q);
  }
  if(traceNodeCount++<35) Serial.printf("NODE TX %s => %s",query,value);
  size_t p=startMsg("node",",s");p=putString(tx,sizeof(tx),p,value);sendPacket(ip,port,p);
}
static int meterCount(int group) {
  switch(group){case 0:return 70;case 1:return 96;case 2:return 49;case 3:return 22;
  case 4:return 82;case 5:return 27;case 6:return 4;case 7:return 16;
  case 8:return 6;case 9:return 32;case 10:return 32;case 11:return 5;
  case 12:return 4;case 13:return 48;default:return 0;}
}
static void sendMeters(const PollSubscription &sub) {
  int n=meterCount(sub.meterGroup);if(!n)return;
  char path[30];snprintf(path,sizeof(path),"/meters/%u",sub.meterGroup);
  size_t p=startMsg(path,",b");
  size_t blobBytes=4+4*n;
  p=putU32(tx,sizeof(tx),p,(uint32_t)blobBytes);
  p=putLE32(tx,sizeof(tx),p,(uint32_t)n);
  for(int i=0;i<n && p<sizeof(tx);i++) {
    // Simulated level 0.01 to ~0.7 linear amplitude.
    float t=(millis()/220.0f+i*1.63f);
    float x=0.06f+0.43f*fabsf(sinf(t));
    if(sub.meterGroup==6 && (i==1||i==2))x=0;
    uint32_t bits;memcpy(&bits,&x,4);
    p=putLE32(tx,sizeof(tx),p,bits);
  }
  sendPacket(sub.ip,sub.port,p);
}
static void scheduleSubscription(IPAddress ip,uint16_t port,const char *path,uint16_t factor,
                                 bool meter,uint8_t group=0,uint8_t channel=0,uint8_t aux=0) {
  if(!path[0])return;
  int slot=-1;
  for(int i=0;i<28;i++)if(subscriptions[i].active && subscriptions[i].ip==ip &&
      subscriptions[i].port==port && subscriptions[i].meter==meter && strcmp(subscriptions[i].path,path)==0){slot=i;break;}
  if(slot<0)for(int i=0;i<28;i++)if(!subscriptions[i].active ||
      (int32_t)(millis()-subscriptions[i].expires)>=0){slot=i;break;}
  if(slot<0) {static uint32_t lastWarn=0;if(millis()-lastWarn>5000){Serial.println("SUBSCRIPTIONS vol");lastWarn=millis();}return;}
  PollSubscription &p=subscriptions[slot];
  p.active=true;p.ip=ip;p.port=port;p.expires=millis()+10000;
  p.intervalMs=(factor>=1&&factor<=99?factor:1)*50;
  p.nextSend=millis()+50; p.meter=meter; p.meterGroup=group;p.meterChan=channel;p.meterAux=aux;
  snprintf(p.path,sizeof(p.path),"%s",path);
}
static void removeSubscriptions(IPAddress ip,uint16_t port,const char *key) {
  for(auto &s:subscriptions)if(s.active&&s.ip==ip&&s.port==port &&
      (!key[0]||strcmp(s.path,key)==0))s.active=false;
}
static void renewSubscriptions(IPAddress ip,uint16_t port,const char *key) {
  for(auto &s:subscriptions)if(s.active&&s.ip==ip&&s.port==port &&
      (!key[0]||strcmp(s.path,key)==0))s.expires=millis()+10000;
}
static void serviceSubscriptions() {
  uint32_t now=millis();int quota=5;
  for(auto &s:subscriptions) {
    if(!s.active)continue;
    if((int32_t)(now-s.expires)>=0){s.active=false;continue;}
    if(quota<=0|| (int32_t)(now-s.nextSend)<0)continue;
    s.nextSend=now+s.intervalMs;
    if(s.meter)sendMeters(s);
    else handleParam(s.path,0,"",0,0,s.ip,s.port);
    quota--;
  }
}
static bool getArg(const uint8_t *buf,size_t len,size_t &at,char type,
                   char *stringOut,size_t stringMax,int32_t &number) {
  if(type=='s'){at=getString(buf,len,at,stringOut,stringMax);return at<=len;}
  if(type=='i'||type=='f'){if(at+4>len)return false;number=(int32_t)getU32(buf+at);at+=4;return true;}
  return false;
}
static void serviceOSC() {
  int count=udp.parsePacket();if(count<=0)return;
  IPAddress ip=udp.remoteIP();uint16_t port=udp.remotePort();requestCount++;
  int n=udp.read(rx,min((int)sizeof(rx),count));
  while(udp.available())udp.read();if(n<=0)return;
  char path[120],tags[36]={0};
  size_t at=getString(rx,n,0,path,sizeof(path));if(at>(size_t)n||path[0]!='/')return;
  if(strcmp(path,"/xinfo")==0){replyInfo(ip,port,true);Serial.printf("XINFO -> %s:%u\n",ip.toString().c_str(),port);return;}
  if(strcmp(path,"/info")==0){replyInfo(ip,port,false);return;}
  if(strcmp(path,"/status")==0){replyStatus(ip,port);return;}
  if(strcmp(path,"/xremote")==0||strcmp(path,"/xremotenfb")==0){registerClient(ip,port);return;}
  if(at<(size_t)n && rx[at]==','){
    at=getString(rx,n,at,tags,sizeof(tags));if(at>(size_t)n)return;
  }
  char firstStr[128]={0};int32_t firstInt=0;
  size_t next=at;
  if(tags[1] && !getArg(rx,n,next,tags[1],firstStr,sizeof(firstStr),firstInt))return;
  traceRequest(path,tags,firstStr);
  // Show which real parameters MX-Q starts requesting after the probe.
  static unsigned otherTrace=0;
  if(otherTrace < 80 && strcmp(path,"/subscribe")!=0 &&
     strcmp(path,"/unsubscribe")!=0 && strcmp(path,"/renew")!=0 &&
     strcmp(path,"/meters")!=0) {
    Serial.printf("OTHER OSC %u: %s tags=%s arg=%s\n",++otherTrace,path,tags,
                  firstStr[0]?firstStr:"<none>");
  }

  if(strcmp(path,"/subscribe")==0){
    int32_t interval=1; char ignore[128]={0};
    if(tags[2] == 'i' && !getArg(rx,n,next,'i',ignore,sizeof(ignore),interval))return;
    scheduleSubscription(ip,port,firstStr,(uint16_t)max(1,(int)interval),false);
    return;
  }
  if(strcmp(path,"/unsubscribe")==0){removeSubscriptions(ip,port,firstStr);return;}
  if(strcmp(path,"/renew")==0){renewSubscriptions(ip,port,firstStr);return;}
  if(strcmp(path,"/meters")==0){
    int group=-1;
    if(sscanf(firstStr,"/meters/%d",&group)==1 && group>=0 && meterCount(group)>0){
      int32_t a=0,b=0,factor=1;char dummy[16]={0};
      // /meters/5 and /meters/6 have additional group/channel arguments.
      int tagPos=2;
      if(tags[tagPos]=='i') {getArg(rx,n,next,'i',dummy,sizeof(dummy),a);tagPos++;}
      if(tags[tagPos]=='i') {getArg(rx,n,next,'i',dummy,sizeof(dummy),b);tagPos++;}
      if(tags[tagPos]=='i') {getArg(rx,n,next,'i',dummy,sizeof(dummy),factor);}
      // When one int is supplied for common groups it denotes time_factor.
      if(group!=5&&group!=6)factor=a?a:1;
      else if(group==6)factor=b?b:1;
      else factor=1;
      scheduleSubscription(ip,port,firstStr,(uint16_t)max(1,(int)factor),true,group,a,b);
    }
    return;
  }
  if(strcmp(path,"/node")==0){if(tags[1]=='s')replyNode(ip,port,firstStr);return;}
  char typ=tags[1];float fv=0;int iv=0;
  if(typ=='f')memcpy(&fv,&firstInt,4);
  if(typ=='i')iv=firstInt;
  handleParam(path,typ,firstStr,fv,iv,ip,port);
}
// Diagnostics only; colors are assigned automatically from channel names.
static const char* colorName(int value) {
  if(value==4) return "BLUE (VOCALS)";
  if(value==2) return "GREEN (INSTRUMENTS)";
  if(value==5) return "MAGENTA (EFFECTS)";
  return "OTHER";
}
static void serviceSerial() {
  if(!Serial.available())return;
  String s=Serial.readStringUntil('\n');s.trim();
  if(s=="help") Serial.println("Commands: help, status, ch <1-32> <NAME>, bus <1-16> <NAME>, send <ch> <bus> <0.0-1.0>, getsend <ch> <bus>, getbuslink <pair 1-8>, getcolor <ch 1-32>");
  else if(s.startsWith("getcolor ")) {
    int ch=0;char extra=0;
    if(sscanf(s.c_str(),"getcolor %d %c",&ch,&extra)==1 && ch>=1 && ch<=32)
      Serial.printf("COLOR CH%02d = %d (%s)\n",ch,channelColors[ch-1],colorName(channelColors[ch-1]));
    else Serial.println("Gebruik: getcolor <kanaal 1-32>");
  }
  else if(s.startsWith("getsend ")) {
    int c=0,b=0; char extra=0;
    if(sscanf(s.c_str(),"getsend %d %d %c",&c,&b,&extra)==2 && c>=1&&c<=32&&b>=1&&b<=16) {
      // Read ONLY: do not change mixer state or notify subscribers.
      Serial.printf("GETSEND CH%02d -> BUS%02d = %.6f (normalized 0..1) | %s\n",
                    c,b,(double)sends[c-1][b-1],sendOn[c-1][b-1]?"ON":"OFF");
    } else Serial.println("Gebruik: getsend <kanaal 1-32> <bus 1-16>");
  }
  else if(s.startsWith("getbuslink ")) {
    int pair=0; char extra=0;
    if(sscanf(s.c_str(),"getbuslink %d %c",&pair,&extra)==1 && pair>=1&&pair<=8) {
      Serial.printf("BUSLINK %d-%d = %s\n",2*pair-1,2*pair,
                    busLinked[pair-1]?"ON":"OFF");
    } else Serial.println("Gebruik: getbuslink <paar 1-8>");
  }
  else if(s=="status") {Serial.printf("IP %s | UDP %u | free heap %u\n",(USE_EXISTING_WIFI?WiFi.localIP():WiFi.softAPIP()).toString().c_str(),OSC_PORT,ESP.getFreeHeap());Serial.printf("OSC ontvangen: %lu | antwoorden: %lu\n",(unsigned long)requestCount,(unsigned long)replyCount);}
  else if(s.startsWith("ch ")||s.startsWith("bus ")) {
    int p=s.indexOf(' '),q=s.indexOf(' ',p+1);if(q<0)return;
    int id=s.substring(p+1,q).toInt();String name=s.substring(q+1);name.trim();
    if(s.startsWith("ch ")&&id>=1&&id<=32) {snprintf(channelNames[id-1],13,"%.12s",name.c_str());char path[36];snprintf(path,sizeof(path),"/ch/%02d/config/name",id);broadcastStr(IPAddress(0,0,0,0),0,path,channelNames[id-1]);updateAutoChannelColor(id);}
    else if(s.startsWith("bus ")&&id>=1&&id<=16) {snprintf(busNames[id-1],13,"%.12s",name.c_str());char path[36];snprintf(path,sizeof(path),"/bus/%02d/config/name",id);broadcastStr(IPAddress(0,0,0,0),0,path,busNames[id-1]);}
    Serial.println("Bijgewerkt");
  } else if(s.startsWith("send ")) {
    int c,b;float x;if(sscanf(s.c_str(),"send %d %d %f",&c,&b,&x)==3&&c>=1&&c<=32&&b>=1&&b<=16) {
      sends[c-1][b-1]=constrain(x,0.0f,1.0f);
      char path[50];snprintf(path,sizeof(path),"/ch/%02d/mix/%02d/level",c,b);
      broadcastFloat(IPAddress(0,0,0,0),0,path,sends[c-1][b-1]);Serial.println("Send bijgewerkt");
    }
  }
}
void setup() {
  Serial.begin(115200); delay(600); Serial.setTimeout(40); seedState();
  WiFi.mode(USE_EXISTING_WIFI?WIFI_STA:WIFI_AP);
  if(USE_EXISTING_WIFI) {
    WiFi.begin(WIFI_SSID,WIFI_PASS);uint32_t start=millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-start<15000) {delay(250);Serial.print('.');}
    if(WiFi.status()!=WL_CONNECTED) {Serial.println("\nWi-Fi niet verbonden. Controleer credentials.");return;}
  } else if(!WiFi.softAP(AP_SSID,AP_PASS)) {Serial.println("AP starten mislukt");return;}
  if(!udp.begin(OSC_PORT)) {Serial.println("UDP poort openen mislukt");return;}
  Serial.println("\nGOED BEDOELD X32 SIMULATOR v0.2j AUTO COLORS + FX RETURN");
  Serial.printf("SSID: %s\n",USE_EXISTING_WIFI?WIFI_SSID:AP_SSID);
  Serial.printf("IP: %s | UDP: %u\n",(USE_EXISTING_WIFI?WiFi.localIP():WiFi.softAPIP()).toString().c_str(),OSC_PORT);
  Serial.println("Model: X32RACK (SIMULATOR, gedeeltelijke OSC-implementatie)");
  Serial.println("Typ help in de seriele monitor voor commando's.");
}
void loop() {for(int i=0;i<4;i++)serviceOSC();serviceSubscriptions();serviceSerial();delay(1);}
