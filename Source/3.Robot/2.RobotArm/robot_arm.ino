/*******************************************************
 * 5-DOF Robot Arm — Sequence Runner (Fruit Pick path)
 * (Button trigger + Strong Grip + Strong Open + Anti-buzz)
 *
 * Fixes:
 *  - No motion on boot (no setAll in setup)
 *  - Ignore button for 1s after boot (debounce/glitch guard)
 *  - FP3 pose == FP2 (S1~S4), so no dip before gripping
 *  - Start without forcing TUCK (avoid bumping fruit)
 *
 * Serial: 115200  (Serial Monitor: No line ending)
 *******************************************************/
#include <Servo.h>

// ===== Feature Switches =====
#define S5_DETACH_AFTER_GRIP 1   // 1=잡은 뒤 집게 PWM 완전 끊기(무음)
#define USE_BUTTON_TRIGGER   1   // 1=물리 버튼으로 Z 시퀀스 실행
#define GRIP_INVERT          0   // 0=기존, 1=열림/닫힘 각도 반전(하드웨어 방향 반대)
#define BTN_ACTIVE_LOW       1   // 버튼이 GND로 떨어지는 구조면 1

// ===== Pins =====
#define PIN_SERVO_S1  9
#define PIN_SERVO_S2  6
#define PIN_SERVO_S3  5
#define PIN_SERVO_S4  3
#define PIN_SERVO_S5  11
#if USE_BUTTON_TRIGGER
#define PIN_BTN_Z     4          // D4 ↔ BUTTON ↔ GND
#endif

Servo s1, s2, s3, s4, s5;

// ===== Per-joint Limits =====
const int S1_MIN=0,   S1_MAX=180;
const int S2_MIN=5,   S2_MAX=175;
const int S3_MIN=5,   S3_MAX=175;
const int S4_MIN=0,   S4_MAX=180;
const int GRIP_MIN=10, GRIP_MAX=170;

// ===== Speed / Timing =====
int STEP_DEG   = 1;
int STEP_DELAY = 18;          // 느리게: 22~28
const int GRIP_STEP_DELAY   = 20;
const unsigned POSE_HOLD_MS = 300; // 느리게: 450~600

// ===== Gripper Angles =====
const int GRIP_OPEN  = 30;
const int GRIP_CLOSE = 160;

// ===== PWM range (S5 only) =====
const int PWM_MIN_US = 540;
const int PWM_MAX_US = 2320;

// ===== Strong Grip (오버드라이브/시팅/백오프) =====
const int GRIP_OVERDRIVE_DEG = 6;
const int RELAX_DEG          = 1;
const int GRIP_FINAL_HOLD_MS = 300;
const int SIT_PULSE_DEG      = 2;
const int SIT_PULSE_MS       = 90;
const int SIT_PULSE_REPEAT   = 2;

// ===== Strong Open (오버오픈 + 펄스) =====
const int OPEN_OVER_DEG      = 12;
const int OPEN_PULSE_REPEAT  = 3;
const int OPEN_PULSE_MS      = 90;

// ===== Default "TUCK(앉은 자세)" =====
const int TUCK_S1 = 90;
const int TUCK_S2 = 60;
const int TUCK_S3 = 30;
const int TUCK_S4 = 90;

// ===== Preset poses(참고) =====
const int LIFT_S2=43, LIFT_S3=16, LIFT_S4=85;
const int ROT_S1=166, ROT_S2=43, ROT_S3=16, ROT_S4=85;
const int PLACE_S1=166, PLACE_S2=47, PLACE_S3=117, PLACE_S4=85;

// ===== Fruit-pick 시퀀스 포즈(네가 준 각도) =====
// 1) 기본자세
const int FP1_S1=76,  FP1_S2=104, FP1_S3=166, FP1_S4=86;  const int FP1_S5=88;
// 2) 열매 앞(오픈)
const int FP2_S1=76,  FP2_S2=69,  FP2_S3=139, FP2_S4=86;  const int FP2_S5=44;
// 3) 열매를 잡음(팔 자세는 FP2와 동일! ↓↓↓ 내려가지 않도록)
const int FP3_S1=76,  FP3_S2=69,  FP3_S3=139, FP3_S4=86;  const int FP3_S5=72; // (표시용)
// 4) 내려서 ‘따기’(S5 패시브 유지)
const int FP4_S1=76,  FP4_S2=155, FP4_S3=166, FP4_S4=82;  const int FP4_S5=72;
// 5) 이동 1 (패시브)
const int FP5_S1=166, FP5_S2=105, FP5_S3=166, FP5_S4=82;  const int FP5_S5=72;
// 6) 이동 2 (패시브)
const int FP6_S1=166, FP6_S2=60,  FP6_S3=119, FP6_S4=82;  const int FP6_S5=72;
// 7) 바구니에서 놓기(attach → 강력열기)
const int FP7_S1=166, FP7_S2=60,  FP7_S3=119, FP7_S4=82;  const int FP7_S5=44;
// 8) 기본자세 복귀
const int FP8_S1=76,  FP8_S2=170, FP8_S3=166, FP8_S4=86;  const int FP8_S5=88;

// ===== State =====
struct Pose { int s1=-1, s2=-1, s3=-1, s4=-1, s5=-1; };
Pose BASIC, GRIP;

int ang1=76, ang2=170, ang3=166, ang4=86, ang5=GRIP_OPEN;

#if USE_BUTTON_TRIGGER
unsigned long lastBtnChangeMs = 0;
bool btnPrev = (BTN_ACTIVE_LOW? true:false); // PULLUP 유휴=HIGH(true)
const unsigned BTN_DEBOUNCE_MS = 30;
unsigned long bootMs = 0; // 부팅 후 버튼 무시 타이머
#endif

// ===== S5 Passive Hold Guard =====
bool S5_passiveHold = false;   // true면 S5는 attach/출력 금지(놓기 직전까지)

// ===== Utils =====
inline int clamp(int x,int lo,int hi){ return x<lo?lo:(x>hi?hi:x); }
inline int clampS1(int v){ return clamp(v, S1_MIN, S1_MAX); }
inline int clampS2(int v){ return clamp(v, S2_MIN, S2_MAX); }
inline int clampS3(int v){ return clamp(v, S3_MIN, S3_MAX); }
inline int clampS4(int v){ return clamp(v, S4_MIN, S4_MAX); }
inline int clampS5(int v){ return clamp(v, GRIP_MIN, GRIP_MAX); }

inline int applyGripInvert(int deg){
#if GRIP_INVERT
  return 180 - deg;
#else
  return deg;
#endif
}

inline void writeXYZWOnly(){ s1.write(ang1); s2.write(ang2); s3.write(ang3); s4.write(ang4); }
inline void writeAll(){
  writeXYZWOnly();
  if (!S5_passiveHold && s5.attached()){
    s5.write(applyGripInvert(ang5));
  }
}

inline void setAll(int a1,int a2,int a3,int a4,int a5){
  ang1=clampS1(a1); ang2=clampS2(a2); ang3=clampS3(a3); ang4=clampS4(a4); ang5=clampS5(a5);
  writeAll();
}

inline void forceS5DetachPassive(){
#if S5_DETACH_AFTER_GRIP
  if (s5.attached()) s5.detach();
  S5_passiveHold = true;
#endif
}

inline void ensureS5Attached(){
#if S5_DETACH_AFTER_GRIP
  if(!s5.attached()) s5.attach(PIN_SERVO_S5, PWM_MIN_US, PWM_MAX_US);
  S5_passiveHold = false;
#endif
}

inline void writeS5Deg(int deg){
  deg = clampS5(deg);
  if (!S5_passiveHold){ s5.write(applyGripInvert(deg)); }
  ang5 = deg;
}

inline void writeS5USFromDeg(int deg){
  deg = clampS5(deg);
  long us = map(applyGripInvert(deg), 0, 180, PWM_MIN_US, PWM_MAX_US);
  if (!S5_passiveHold){ s5.writeMicroseconds(us); }
  ang5 = deg;
}

void moveToBlocking(int t1,int t2,int t3,int t4,int t5){
  t1=clampS1(t1);
  t2=clampS2(t2);
  t3=clampS3(t3);
  t4=clampS4(t4);

  if (S5_passiveHold) {
    t5 = ang5; // 패시브 중 S5 불변
  } else {
    t5=clampS5(t5);
    if (t5 != ang5) ensureS5Attached();
  }

  while(ang1!=t1 || ang2!=t2 || ang3!=t3 || ang4!=t4 || ang5!=t5){
    if(ang1<t1) ang1=min(ang1+STEP_DEG,t1); else if(ang1>t1) ang1=max(ang1-STEP_DEG,t1);
    if(ang2<t2) ang2=min(ang2+STEP_DEG,t2); else if(ang2>t2) ang2=max(ang2-STEP_DEG,t2);
    if(ang3<t3) ang3=min(ang3+STEP_DEG,t3); else if(ang3>t3) ang3=max(ang3-STEP_DEG,t3);
    if(ang4<t4) ang4=min(ang4+STEP_DEG,t4); else if(ang4>t4) ang4=max(ang4-STEP_DEG,t4);

    if (!S5_passiveHold){
      if(ang5<t5) ang5=min(ang5+STEP_DEG,t5); else if(ang5>t5) ang5=max(ang5-STEP_DEG,t5);
    }

    writeAll();

    int d1=abs(t1-ang1), d2=abs(t2-ang2), d3=abs(t3-ang3), d4=abs(t4-ang4);
    int d5 = S5_passiveHold ? 0 : abs(t5-ang5);
    int dmax = max(max(d1,d2), max(d3, max(d4, d5)));
    int baseDelay = (!S5_passiveHold && (t5!=ang5))
                    ? ((d5<=8)? max(GRIP_STEP_DELAY, STEP_DELAY): STEP_DELAY)
                    : STEP_DELAY;
    if (dmax <= 8) delay(baseDelay + 6);
    else           delay(baseDelay);
  }
}

void moveAndHold(int t1,int t2,int t3,int t4,int t5, unsigned holdMs){
  moveToBlocking(t1,t2,t3,t4,t5);
  delay(holdMs);
}

void settleAt(int t1,int t2,int t3,int t4,int t5, unsigned holdMs=180, int repeats=1){
  for(int i=0;i<repeats; ++i){
    moveToBlocking(t1,t2,t3,t4,t5);
    delay(holdMs);
  }
}

// ===== TUCK(앉은 자세) =====
void goTuck(unsigned hold=150){
  moveToBlocking(TUCK_S1, TUCK_S2, TUCK_S3, TUCK_S4, ang5);
  delay(hold);
}

// ===== Grip Open/Close =====
void openGrip(unsigned holdOpenMs=250){
  ensureS5Attached();

  moveToBlocking(ang1, ang2, ang3, ang4, GRIP_OPEN);
  delay(holdOpenMs);

  int overOpen = clampS5(GRIP_OPEN - OPEN_OVER_DEG);
  s5.write(applyGripInvert(overOpen));  ang5 = overOpen;  delay(140);
  s5.write(applyGripInvert(GRIP_OPEN)); ang5 = GRIP_OPEN; delay(120);

  long usOpen = map(applyGripInvert(GRIP_OPEN), 0, 180, PWM_MIN_US, PWM_MAX_US);
  for(int i=0;i<OPEN_PULSE_REPEAT;i++){
    s5.writeMicroseconds(usOpen - 20); delay(OPEN_PULSE_MS);
    s5.writeMicroseconds(usOpen + 20); delay(OPEN_PULSE_MS);
    s5.writeMicroseconds(usOpen);      delay(OPEN_PULSE_MS);
  }
  delay(180);
}

void gripCloseWithRelief(){
  ensureS5Attached();

  int base = clampS5(GRIP_CLOSE);
  moveToBlocking(ang1, ang2, ang3, ang4, base);
  delay(120);

  int over = clampS5(base + GRIP_OVERDRIVE_DEG);
  writeS5Deg(over); delay(140);

  for(int k=0;k<SIT_PULSE_REPEAT;k++){
    writeS5Deg(over - SIT_PULSE_DEG); delay(SIT_PULSE_MS);
    writeS5Deg(over + SIT_PULSE_DEG); delay(SIT_PULSE_MS);
    writeS5Deg(over);                 delay(SIT_PULSE_MS);
  }

  int relax = clampS5(base - RELAX_DEG);
  writeS5Deg(relax);
  delay(GRIP_FINAL_HOLD_MS);

#if S5_DETACH_AFTER_GRIP
  forceS5DetachPassive();   // 이후 놓기 전까지 S5 금지
#endif
}

void printHelp(){
  Serial.println(F("\n=== Robot Arm (Strong Grip + Strong Open + Button trigger) ==="));
  Serial.println(F("Z : Run sequence (same as Button)"));
  Serial.println(F("C : Print current angles (S1..S5)"));
  Serial.println(F("B : Save BASIC pose (current)"));
  Serial.println(F("J : Save GRIP  pose (current)"));
  Serial.println(F("O : Open gripper"));
  Serial.println(F("P : Close gripper (strong)"));
  Serial.println(F("H : Help"));
}

void printCurrent(){
  Serial.print(F("CUR S1..S5 = "));
  Serial.print(ang1); Serial.print(',');
  Serial.print(ang2); Serial.print(',');
  Serial.print(ang3); Serial.print(',');
  Serial.print(ang4); Serial.print(',');
  Serial.println(ang5);
}

// ===== 메인 시퀀스(Z) =====
void runSequenceZ(){
  Serial.println(F("Running sequence (Z) — Fruit pick path..."));

  // (중요) 시작 시 TUCK으로 강제 이동하지 않음
  // goTuck(120);

  // 1) 기본자세(오픈 유지)
  moveAndHold(FP1_S1, FP1_S2, FP1_S3, FP1_S4, FP1_S5, POSE_HOLD_MS);

  // 2) 열매 앞(오픈)
  moveAndHold(FP2_S1, FP2_S2, FP2_S3, FP2_S4, FP2_S5, POSE_HOLD_MS);

  // 3) 열매를 잡음(포지셔닝만; 아직 안 잡음 — 오픈 유지)
  moveAndHold(FP3_S1, FP3_S2, FP3_S3, FP3_S4, ang5 /*keep open*/, POSE_HOLD_MS);

  // → 여기서 뒤에 실제로 잡기 수행 (강력 닫기 + DETACH 시작)
  gripCloseWithRelief();       // S5 DETACH + S5_passiveHold=true
  delay(POSE_HOLD_MS);         // 그립 안정화

  // 4) 내려서 ‘따기’ (S5 패시브 유지)
  moveAndHold(FP4_S1, FP4_S2, FP4_S3, FP4_S4, ang5/*ignored*/, POSE_HOLD_MS + 200);

  // 5) 이동 1 (패시브)
  moveAndHold(FP5_S1, FP5_S2, FP5_S3, FP5_S4, ang5/*ignored*/, POSE_HOLD_MS);

  // 6) 이동 2 (패시브)
  moveAndHold(FP6_S1, FP6_S2, FP6_S3, FP6_S4, ang5/*ignored*/, POSE_HOLD_MS);

  // 7) 바구니 위치 → attach 보장 → 강력 열기(놓기)
  moveToBlocking(FP7_S1, FP7_S2, FP7_S3, FP7_S4, ang5/*ignored*/);
  ensureS5Attached();          // 패시브 해제
  openGrip(300);               // 강력 열기(오버오픈+펄스 포함)

  // 8) 기본자세 복귀
  moveAndHold(FP8_S1, FP8_S2, FP8_S3, FP8_S4, FP8_S5, POSE_HOLD_MS);

  Serial.println(F("Done."));
}

// ===== Setup / Loop =====
void setup(){
  Serial.begin(115200);
  unsigned long t0=millis(); while(!Serial && millis()-t0<2000){}

#if USE_BUTTON_TRIGGER
  pinMode(PIN_BTN_Z, INPUT_PULLUP); // D4 ↔ 버튼 ↔ GND
#endif

  s1.attach(PIN_SERVO_S1);
  s2.attach(PIN_SERVO_S2);
  s3.attach(PIN_SERVO_S3);
  s4.attach(PIN_SERVO_S4);
  s5.attach(PIN_SERVO_S5, PWM_MIN_US, PWM_MAX_US); // attach만, 이동 없음

  // ★ 부팅 시 절대 이동하지 않음 (setAll 삭제)
  // 필요하면 현재 자세를 읽어 ang1~ang5 변수만 보정해도 OK

#if USE_BUTTON_TRIGGER
  bootMs = millis(); // 부팅 후 1초 버튼 무시
#endif

  Serial.println(F("Ready. (115200 / No line ending)"));
  printHelp();
}

void loop(){
#if USE_BUTTON_TRIGGER
  // ★ 부팅 후 1초 동안 버튼 무시 (글리치 방지)
  if (millis() - bootMs < 1000) return;

  bool btnNow = digitalRead(PIN_BTN_Z);
  bool pressed = BTN_ACTIVE_LOW ? (btnNow == LOW) : (btnNow == HIGH);
  if ((btnNow != btnPrev) && (millis() - lastBtnChangeMs > BTN_DEBOUNCE_MS)){
    lastBtnChangeMs = millis();
    btnPrev = btnNow;
    if (pressed){
      Serial.println(F("[BUTTON] Z trigger"));
      runSequenceZ();
    }
  }
#endif
}
