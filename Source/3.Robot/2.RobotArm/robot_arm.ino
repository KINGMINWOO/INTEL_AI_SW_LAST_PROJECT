/*******************************************************
 * 5-DOF Robot Arm — Serial-only Sequence Runner
 * Order: BASIC → GRIP(접근·정착→닫기) → LIFT → ROT → PLACE(접근·정착→열기) → RETURN
 * 이동은 항상 열린 상태로(또는 현재 상태 유지) 진행, 집을 때만 닫고 운반, 놓을 때 다시 연다.
 * Baud: 115200  (Serial Monitor: No line ending)
 *******************************************************/
#include <Servo.h>

// ===== Pins =====
#define PIN_SERVO_S1  9
#define PIN_SERVO_S2  6
#define PIN_SERVO_S3  5
#define PIN_SERVO_S4  3
#define PIN_SERVO_S5  11

Servo s1, s2, s3, s4, s5;

// ===== Limits =====
const int ANG_MIN=10, ANG_MAX=170;
const int GRIP_MIN=40, GRIP_MAX=140;

// ===== Speed (천천히 추천) =====
int STEP_DEG   = 1;
int STEP_DELAY = 18;

// ===== Pose hold =====
const unsigned POSE_HOLD_MS = 600; // 0.6s 멈춤

// ===== Gripper 각도 (하드웨어 기준) =====
const int GRIP_OPEN  = 60;   // 벌리기(디폴트)
const int GRIP_CLOSE = 130;  // 닫기(운반시)

// ===== 현재 각도 =====
int ang1=90, ang2=90, ang3=90, ang4=90, ang5=GRIP_OPEN;  // 기본 열림

// ===== 하드코딩 포즈(팔 관절용) =====
// LIFT
const int LIFT_S2   = 43;
const int LIFT_S3   = 16;
const int LIFT_S4   = 85;
// ROT
const int ROT_S1    = 166;
const int ROT_S2    = 43;
const int ROT_S3    = 16;
const int ROT_S4    = 85;
// PLACE
const int PLACE_S1  = 166;
const int PLACE_S2  = 47;
const int PLACE_S3  = 117;
const int PLACE_S4  = 85;

// ===== 사용자 저장 포즈 =====
struct Pose { int s1=-1, s2=-1, s3=-1, s4=-1, s5=-1; };
Pose BASIC, GRIP;

// ===== Utils =====
inline int clamp(int x,int lo,int hi){ return x<lo?lo:(x>hi?hi:x); }
void writeAll(){ s1.write(ang1); s2.write(ang2); s3.write(ang3); s4.write(ang4); s5.write(ang5); }
void setAll(int a1,int a2,int a3,int a4,int a5){ ang1=a1; ang2=a2; ang3=a3; ang4=a4; ang5=a5; writeAll(); }

// 부드럽게 목표까지 이동 (접근 말미 감속)
void moveToBlocking(int t1,int t2,int t3,int t4,int t5){
  t1=clamp(t1,ANG_MIN,ANG_MAX);
  t2=clamp(t2,ANG_MIN,ANG_MAX);
  t3=clamp(t3,ANG_MIN,ANG_MAX);
  t4=clamp(t4,ANG_MIN,ANG_MAX);
  t5=clamp(t5,GRIP_MIN,GRIP_MAX);

  while(ang1!=t1 || ang2!=t2 || ang3!=t3 || ang4!=t4 || ang5!=t5){
    if(ang1<t1) ang1=min(ang1+STEP_DEG,t1); else if(ang1>t1) ang1=max(ang1-STEP_DEG,t1);
    if(ang2<t2) ang2=min(ang2+STEP_DEG,t2); else if(ang2>t2) ang2=max(ang2-STEP_DEG,t2);
    if(ang3<t3) ang3=min(ang3+STEP_DEG,t3); else if(ang3>t3) ang3=max(ang3-STEP_DEG,t3);
    if(ang4<t4) ang4=min(ang4+STEP_DEG,t4); else if(ang4>t4) ang4=max(ang4-STEP_DEG,t4);
    if(ang5<t5) ang5=min(ang5+STEP_DEG,t5); else if(ang5>t5) ang5=max(ang5-STEP_DEG,t5);

    writeAll();

    int d1=abs(t1-ang1), d2=abs(t2-ang2), d3=abs(t3-ang3), d4=abs(t4-ang4), d5=abs(t5-ang5);
    int dmax = max(d1, max(d2, max(d3, max(d4, d5))));
    if (dmax <= 8) delay(STEP_DELAY + 8);
    else           delay(STEP_DELAY);
  }
}

// 포즈로 이동 후 hold
void moveAndHold(int t1,int t2,int t3,int t4,int t5, unsigned holdMs){
  moveToBlocking(t1,t2,t3,t4,t5);
  delay(holdMs);
}

// 목표에 도착한 뒤, 같은 목표로 재수렴 + 홀드 (정착)
void settleAt(int t1,int t2,int t3,int t4,int t5, unsigned holdMs=250, int repeats=2){
  for(int i=0;i<repeats; ++i){
    moveToBlocking(t1,t2,t3,t4,t5);
    delay(holdMs);
  }
}

// 단독 열기/닫기 (현재 팔 자세 유지)
void openGrip(unsigned holdOpenMs=600){
  moveToBlocking(ang1, ang2, ang3, ang4, GRIP_OPEN);
  delay(holdOpenMs);
}
void closeGrip(unsigned holdCloseMs=600){
  moveToBlocking(ang1, ang2, ang3, ang4, GRIP_CLOSE);
  delay(holdCloseMs);
}

void printHelp(){
  Serial.println(F("\n=== Robot Arm Sequence ==="));
  Serial.println(F("Z : BASIC→GRIP(접근·정착→닫기)→LIFT→ROT→PLACE(접근·정착→열기)→RETURN"));
  Serial.println(F("C : Print current angles (S1..S5)"));
  Serial.println(F("B : Save BASIC pose   (use current angles)"));
  Serial.println(F("J : Save GRIP  pose   (use current angles)"));
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

void setup(){
  Serial.begin(115200);
  unsigned long t0=millis(); while(!Serial && millis()-t0<2000){} // (레오나르도류) 연결 대기

  s1.attach(PIN_SERVO_S1);
  s2.attach(PIN_SERVO_S2);
  s3.attach(PIN_SERVO_S3);
  s4.attach(PIN_SERVO_S4);
  s5.attach(PIN_SERVO_S5);

  setAll(90,90,90,90,GRIP_OPEN);  // 전원/리셋 시 디폴트: 열린 상태
  Serial.println(F("Ready. (115200 / No line ending)"));
  printHelp();
}

void loop(){
  if(Serial.available()){
    int c = Serial.read();
    if(c=='\r' || c=='\n') return;

    Serial.print((char)c); Serial.println(F(" ✓"));

    switch(c){
      case 'H': printHelp(); break;
      case 'C': printCurrent(); break;

      case 'B':
        BASIC.s1 = ang1; BASIC.s2 = ang2; BASIC.s3 = ang3; BASIC.s4 = ang4; BASIC.s5 = ang5;
        Serial.println(F("Saved BASIC."));
        break;

      case 'J':
        GRIP.s1  = ang1; GRIP.s2  = ang2; GRIP.s3  = ang3; GRIP.s4  = ang4; GRIP.s5  = ang5;
        Serial.println(F("Saved GRIP."));
        break;

      case 'Z': {
        Serial.println(F("Running sequence..."));

        // 1) BASIC (또는 현재) — 시작 포즈 기억, 이동은 '열림'으로
        int b1= (BASIC.s1>=0)? BASIC.s1 : ang1;
        int b2= (BASIC.s2>=0)? BASIC.s2 : ang2;
        int b3= (BASIC.s3>=0)? BASIC.s3 : ang3;
        int b4= (BASIC.s4>=0)? BASIC.s4 : ang4;
        // int b5= (BASIC.s5>=0)? BASIC.s5 : ang5;  // 시작시엔 열고 이동할 거라 b5는 고정 안 씀
        moveAndHold(b1,b2,b3,b4, GRIP_OPEN, POSE_HOLD_MS);

        // 2) GRIP — S1은 유지, S2~S4는 PLACE와 동일로 접근(열림), 정착 후 '닫기'
        int g1 = (GRIP.s1>=0)? GRIP.s1 : b1;   // 맨 밑 모터(베이스) 그대로
        int g2 = PLACE_S2;
        int g3 = PLACE_S3;
        int g4 = PLACE_S4;
        moveToBlocking(g1,g2,g3,g4, GRIP_OPEN);  // 열린 상태로 접근
        delay(300);
        settleAt(g1,g2,g3,g4, GRIP_OPEN, 250, 2); // 목표로 재수렴(열림 상태 유지)
        closeGrip(700);                            // 여기서만 닫음(운반 시작)

        // 3) LIFT — 그립 각도 유지(=닫힌 상태로 운반)
        moveAndHold(ang1, LIFT_S2, LIFT_S3, LIFT_S4, ang5, POSE_HOLD_MS);

        // 4) ROT — 그립 각도 유지(닫힘 유지)
        moveToBlocking(ROT_S1, ROT_S2, ROT_S3, ROT_S4, ang5);
        delay(300);
        settleAt(ROT_S1, ROT_S2, ROT_S3, ROT_S4, ang5, 250, 2);

        // 5) PLACE — 닫힌 상태로 접근 후, 정착하고 '열기'로 내려놓기
        moveToBlocking(PLACE_S1, PLACE_S2, PLACE_S3, PLACE_S4, ang5);
        delay(300);
        settleAt(PLACE_S1, PLACE_S2, PLACE_S3, PLACE_S4, ang5, 250, 2);
        openGrip(700);   // 놓으면서 열기

        // 6) RETURN — 처음 자리로 복귀(열림 유지)
        moveToBlocking(b1, b2, b3, b4, GRIP_OPEN);

        Serial.println(F("Done."));
      } break;

      default:
        Serial.println(F(" (unknown key — H for help)"));
        break;
    }
  }
}
