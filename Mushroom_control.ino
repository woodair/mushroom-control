#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>

// SH1107 128x128 I2C, 페이지버퍼(저RAM)
U8G2_SH1107_SEEED_128X128_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define PIR_PIN   5
#define DHTPIN    6
#define DHTTYPE   DHT22
DHT dht(DHTPIN, DHTTYPE);

// 릴레이(하드웨어 트리거에 맞춰 조정)
#define RELAY_COOL  7
#define RELAY_HEAT  8
#define RELAY_FOG   3
#define RELAY_CYC   4
#define RELAY_ON    LOW
#define RELAY_OFF   HIGH

const float T_COOL_ON=26.0, T_COOL_OFF=18.0, T_HEAT_ON=10.0, T_HEAT_OFF=15.0;
const float H_FOG_ON=85.0, H_FOG_OFF=95.0;

const unsigned long TIMEOUT_MS=300000UL, LOOP_MS=1000UL;
unsigned long lastMotionMs=0, lastTick=0, lastAnyActiveMs=0, cycStartMs=0;
const unsigned long CYC_IDLE_MS=600000UL, CYC_RUN_MS=300000UL;

bool screenOn=true, coolOn=false, heatOn=false, fogOn=false, cycOn=false, cycForced=false;

inline bool motionDetected(){ return digitalRead(PIR_PIN)==HIGH; }
inline void setRelay(uint8_t pin, bool on){ digitalWrite(pin, on?RELAY_ON:RELAY_OFF); }
inline void applyOutputs(){ setRelay(RELAY_COOL,coolOn); setRelay(RELAY_HEAT,heatOn); setRelay(RELAY_FOG,fogOn); setRelay(RELAY_CYC,cycOn); }

void drawDot(int x,int y,bool on){ if(on) u8g2.drawDisc(x,y,6); else u8g2.drawCircle(x,y,6); }

void drawScreen(float t,float h,int remainSec){
  if(!screenOn) return;
  u8g2.firstPage();
  do{
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_6x12_tf);

    u8g2.setCursor(0,12);  u8g2.print(F("Mushroom Incubator"));
    u8g2.setCursor(0,36);
    if(isnan(t)) u8g2.print(F("--.-")); else u8g2.print(t,1);
    u8g2.print(F("C  "));
    if(isnan(h)) u8g2.print(F("--.-")); else u8g2.print(h,1);
    u8g2.print(F("%"));

    u8g2.setCursor(0,56);  u8g2.print(F("Cool        Fog"));
    drawDot(45,54,coolOn); drawDot(115,54,fogOn);

    u8g2.setCursor(0,76);  u8g2.print(F("Heat        Vent"));
    drawDot(45,74,heatOn); drawDot(115,74,cycOn);

    u8g2.setCursor(0,96);  u8g2.print(F("Cyc         PIR"));
    drawDot(45,94,cycOn);  drawDot(115,94,motionDetected());

    char buf[10]; int mm=remainSec/60, ss=remainSec%60;
    snprintf(buf,sizeof(buf),"%02d:%02d",mm,ss);
    u8g2.setCursor(0,116); u8g2.print(F("OLED OFF ")); u8g2.print(buf);
  }while(u8g2.nextPage());
}

int secondsUntilOff(unsigned long now){
  if(!screenOn) return 0;
  unsigned long elapsed=now-lastMotionMs;
  if(elapsed>=TIMEOUT_MS) return 0;
  unsigned long remain_ms=TIMEOUT_MS-elapsed;
  return (int)((remain_ms+999UL)/1000UL);
}

void autoControl(float t,float h){
  if(!isnan(t)){
    if(t>=T_COOL_ON) coolOn=true; else if(t<=T_COOL_OFF) coolOn=false;
    if(t<=T_HEAT_ON) heatOn=true; else if(t>=T_HEAT_OFF) heatOn=false;
    if(coolOn&&heatOn){ if(t>=(T_COOL_ON+T_HEAT_OFF)/2.0) heatOn=false; else coolOn=false; }
  }
  if(!isnan(h)){
    if(h<=H_FOG_ON) fogOn=true; else if(h>=H_FOG_OFF) fogOn=false;
  }
}

void updateVentilation(unsigned long now){
  bool anyMain=coolOn||heatOn||fogOn;
  if(anyMain){ cycOn=true; cycForced=false; lastAnyActiveMs=now; return; }
  if(cycForced){
    if(now-cycStartMs>=CYC_RUN_MS){ cycForced=false; cycOn=false; lastAnyActiveMs=now; }
    else cycOn=true;
  }else{
    if(now-lastAnyActiveMs>=CYC_IDLE_MS){ cycForced=true; cycStartMs=now; cycOn=true; }
    else cycOn=false;
  }
}

void sendStatus(float t,float h){
  Serial.print(F("{\"t\":"));   if(isnan(t)) Serial.print(F("null")); else Serial.print(t,1);
  Serial.print(F(",\"h\":"));   if(isnan(h)) Serial.print(F("null")); else Serial.print(h,1);
  Serial.print(F(",\"cool\":")); Serial.print(coolOn?1:0);
  Serial.print(F(",\"heat\":")); Serial.print(heatOn?1:0);
  Serial.print(F(",\"fog\":"));  Serial.print(fogOn?1:0);
  Serial.print(F(",\"cyc\":"));  Serial.print(cycOn?1:0);
  Serial.println('}');
}

void setup(){
  Serial.begin(115200);
  pinMode(PIR_PIN,INPUT);

  // 초기 레벨 설정 후 OUTPUT
  digitalWrite(RELAY_COOL,RELAY_OFF);
  digitalWrite(RELAY_HEAT,RELAY_OFF);
  digitalWrite(RELAY_FOG, RELAY_OFF);
  digitalWrite(RELAY_CYC, RELAY_OFF);
  pinMode(RELAY_COOL,OUTPUT);
  pinMode(RELAY_HEAT,OUTPUT);
  pinMode(RELAY_FOG, OUTPUT);
  pinMode(RELAY_CYC, OUTPUT);

  dht.begin();
  u8g2.begin();

  unsigned long now=millis();
  lastMotionMs=now; lastAnyActiveMs=now;
  float t=dht.readTemperature(), h=dht.readHumidity();
  drawScreen(t,h,secondsUntilOff(now));
}

void loop(){
  unsigned long now=millis();

  if(motionDetected()){
    lastMotionMs=now;
    if(!screenOn){ screenOn=true; float t=dht.readTemperature(), h=dht.readHumidity(); drawScreen(t,h,secondsUntilOff(now)); }
  }
  if(screenOn && (now-lastMotionMs>=TIMEOUT_MS)){ screenOn=false; }

  if(now-lastTick>=LOOP_MS){
    lastTick=now;
    float t=dht.readTemperature(), h=dht.readHumidity();
    autoControl(t,h);
    updateVentilation(now);
    applyOutputs();
    if(screenOn) drawScreen(t,h,secondsUntilOff(now));
    sendStatus(t,h);
  }
}