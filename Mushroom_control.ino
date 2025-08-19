#include <Wire.h>
#include <U8g2lib.h>          // ★ 변경: u8g2 사용
#include <DHT.h>
#include <ArduinoJson.h>

#define I2C_SDA D2
#define I2C_SCL D1
#define OLED_ADDR 0x3C        // SH1107 기본 주소(보드에 따라 0x3D일 수 있다)

// ★ 변경: u8g2 디스플레이 객체(128x128, HW I2C)
U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); 
// 주소가 0x3D이면 아래처럼 생성한다.
// U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 0x3D);

// 센서/입력
#define PIR_PIN  D5
const bool LOGIC_ACTIVE_HIGH = true;

#define DHTPIN   D6
#define DHTTYPE  DHT22
DHT dht(DHTPIN, DHTTYPE);

// 릴레이 출력 (LOW=ON 가정)
#define RELAY_COOL  D7
#define RELAY_HEAT  D8
#define RELAY_FOG   D3
#define RELAY_CYC   D4   // 환기 릴레이(GPIO16). 부트 영향 적어 권장이다.
#define RELAY_ON    LOW
#define RELAY_OFF   HIGH

// 제어 임계 (히스테리시스)
const float T_COOL_ON  = 26.0;
const float T_COOL_OFF = 24.0;
const float T_HEAT_ON  = 20.0;
const float T_HEAT_OFF = 21.0;
const float H_FOG_ON   = 85.0;
const float H_FOG_OFF  = 95.0;

// 화면 절전 타이머(5분)
const unsigned long TIMEOUT_MS = 5UL * 60UL * 1000UL;
unsigned long lastMotionMs = 0;
bool screenOn = true;

// 주기
unsigned long lastTick = 0;
const unsigned long LOOP_MS = 1000;

bool coolOn=false, heatOn=false, fogOn=false, cycOn=false;

// 환기 정책
const unsigned long CYC_IDLE_MS = 10UL * 60UL * 1000UL; // 10분
const unsigned long CYC_RUN_MS  = 5UL  * 60UL * 1000UL; // 5분
unsigned long lastAnyActiveMs = 0;  // 마지막으로 메인 릴레이가 하나라도 ON이었던 시각
unsigned long cycStartMs = 0;      // 강제 환기 시작 시각
bool cycForced = false;            // 강제 환기 중 여부

// 유틸
bool motionDetected() {
  int v = digitalRead(PIR_PIN);
  return LOGIC_ACTIVE_HIGH ? (v==HIGH) : (v==LOW);
}
void setRelay(int pin, bool on) { 
  digitalWrite(pin, on?RELAY_ON:RELAY_OFF); 
}
void applyOutputs() {
  setRelay(RELAY_COOL, coolOn);
  setRelay(RELAY_HEAT, heatOn);
  setRelay(RELAY_FOG,  fogOn);
  setRelay(RELAY_CYC, cycOn);
}

// ★ 변경: u8g2 버전의 점 그리기
void drawDot(int x, int y, bool on) {
  if (on) u8g2.drawDisc(x, y, 6);
  else    u8g2.drawCircle(x, y, 6);
}

// ★ 변경: u8g2 버전 화면 렌더링
void drawScreen(float t, float h) {
  if (!screenOn) return;
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.enableUTF8Print();

  // 헤더
  //u8g2.setFont(u8g2_font_6x12_tf);
  //u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.setFont(u8g2_font_unifont_t_korean2);
  u8g2.setCursor(0, 14);
  //u8g2.print("Mushroom  PIR:");
  u8g2.print("실험실버섯재배기");
  //u8g2.print(motionDetected()? "ON":"OFF");

  // 온습도
  u8g2.setCursor(0, 40);
  //u8g2.print("T: ");
  u8g2.print("  ");
  if (isnan(t)) u8g2.print("--.-");
  else          u8g2.print(t,1);
  //u8g2.print("°C  H: ");
  u8g2.print("°C  ");
  if (isnan(h)) u8g2.print("--.-");
  else          u8g2.print(h,1);
  //else          u8g2.print((int)h);
  u8g2.print("%");

  // 라벨
  u8g2.setCursor(0, 65);
  u8g2.print("Cool    Fog");

  // 상태 점(128x128 기준 위치)
  drawDot(45,  62, coolOn);
  drawDot(115, 62, fogOn);

  u8g2.setCursor(0, 85);
  u8g2.print("Heat    Vent");

  // 상태 점(128x128 기준 위치)
  drawDot(45, 82, heatOn);
  drawDot(115,82, cycOn);

  u8g2.setCursor(0, 105);
  u8g2.print("Cyc     PIR");

  // 상태 점(128x128 기준 위치)
  drawDot(45, 102, cycOn);
  drawDot(115,102, motionDetected());

  u8g2.setCursor(0, 125);
  u8g2.print("           Jini");

  u8g2.sendBuffer();
}

// ★ 변경: 전원 온/오프 제어
void turnDisplayOn(float t, float h) {
  u8g2.setPowerSave(0);   // ON
  screenOn = true;
  drawScreen(t, h);
}
void turnDisplayOff() {
  u8g2.clearBuffer(); 
  u8g2.sendBuffer();
  u8g2.setPowerSave(1);   // OFF
  screenOn = false;
}

void autoControl(float t, float h) {
  if (!isnan(t)) {
    if (t >= T_COOL_ON) coolOn = true;
    else if (t <= T_COOL_OFF) coolOn = false;

    if (t <= T_HEAT_ON) heatOn = true;
    else if (t >= T_HEAT_OFF) heatOn = false;

    if (coolOn && heatOn) {
      if (t >= (T_COOL_ON + T_HEAT_OFF)/2.0) heatOn = false;
      else coolOn = false;
    }
  }
  if (!isnan(h)) {
    if (h <= H_FOG_ON) fogOn = true;
    else if (h >= H_FOG_OFF) fogOn = false;
  }
}

// 환기 로직
void updateVentilation(unsigned long now) {
  bool anyMain = coolOn || heatOn || fogOn;

  if (anyMain) {
    cycOn = true;                // 메인 동작 중엔 항상 환기 켜기
    cycForced = false;           // 강제 환기 상태 해제
    lastAnyActiveMs = now;        // 마지막 활동 시각 갱신
    return;
  }

  // 메인 모두 OFF인 구간
  if (cycForced) {
    // 강제 환기 진행 중
    if (now - cycStartMs >= CYC_RUN_MS) {
      cycForced = false;
      cycOn = false;
      lastAnyActiveMs = now;      // 강제 환기 종료 시각을 마지막 활동으로 간주
    } else {
      cycOn = true;              // 강제 환기 유지
    }
  } else {
    // 대기 시간 경과 시 강제 환기 시작
    if (now - lastAnyActiveMs >= CYC_IDLE_MS) {
      cycForced = true;
      cycStartMs = now;
      cycOn = true;
    } else {
      cycOn = false;
    }
  }
}

void sendStatus(float t, float h) {
  StaticJsonDocument<160> doc;
  if (!isnan(t)) doc["t"]=t;
  if (!isnan(h)) doc["h"]=h;
  doc["fan"]=coolOn?1:0;
  doc["heat"]=heatOn?1:0;
  doc["fog"]=fogOn?1:0;
  doc["cyc"]=cycOn?1:0;   // 환기 상태 포함
  serializeJson(doc, Serial);
  Serial.print('\n');
}

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);

  pinMode(RELAY_COOL, OUTPUT);
  pinMode(RELAY_HEAT, OUTPUT);
  pinMode(RELAY_FOG, OUTPUT);
  pinMode(RELAY_CYC, OUTPUT);
  applyOutputs();

  dht.begin();
  //Wire.begin(I2C_SDA, I2C_SCL);

  // ★ 변경: u8g2 초기화
  u8g2.begin();
  u8g2.setPowerSave(0);   // 화면 ON

  // 부팅 시 화면 강제 ON
  lastMotionMs = millis();
  lastAnyActiveMs = millis();   // 부팅 시점을 마지막 활동 시각으로 초기화
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  turnDisplayOn(t, h);
}

void loop() {
  unsigned long now = millis();

  // PIR 처리
  if (motionDetected()) {
    lastMotionMs = now;
    if (!screenOn) {
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      turnDisplayOn(t, h);
    }
  }
  if (screenOn && (now - lastMotionMs >= TIMEOUT_MS)) {
    turnDisplayOff();
  }

  // 1초 주기 센싱/제어/표시
  if (now - lastTick >= LOOP_MS) {
    lastTick = now;

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    autoControl(t, h);          // 메인 릴레이 결정
    updateVentilation(now);     // 환기 릴레이 결정
    applyOutputs();             // 모든 릴레이 적용
    drawScreen(t, h);           // 화면 표시
    sendStatus(t, h);           // 상태 출력(JSON)
  }

  delay(20);
}