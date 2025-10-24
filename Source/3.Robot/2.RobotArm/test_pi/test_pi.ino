#include <Servo.h>

// ===== 핀 정의 =====
#define PIN_SERVO_S1  9
#define PIN_SERVO_S2  6
#define PIN_SERVO_S3  5
#define PIN_SERVO_S4  3
#define PIN_SERVO_S5  11
#define PIN_SERVO_CR  10

// ===== Servo 객체 =====
Servo S1, S2, S3, S4, S5;
Servo crServo;

// ===== 현재 각도 저장 =====
int currentAngles[5] = {90,90,90,90,90};

// ===== 시퀀스 정의 =====
struct SequenceStep { int angles[5]; const char* label; };

// high 시퀀스
SequenceStep highSeq[] = {
  {{155,149,176,90,61}, "기본자세"},
  {{156,103,175,82,44}, "세팅 자세"},
  {{150,38,70,86,44}, ""}, {{150,38,70,86,88}, ""},
  {{150,38,88,86,88}, ""}, {{157,109,176,77,89}, "돌아옴"},
  {{59,109,176,77,89}, "돌림"}, {{59,77,176,77,89}, "놓기전"},
  {{59,77,176,76,44}, "놓음"}, {{59,147,176,76,92}, "돌아오기전 자세"},
  {{155,149,176,90,61}, "기본자세 복귀"}
};

// mid 시퀀스
SequenceStep midSeq[] = {
  {{155,149,176,90,61}, "기본자세"},
  {{156,103,175,82,44}, "세팅 자세"},
  {{156,50,121,82,44}, ""}, {{156,49,121,82,89}, ""},
  {{156,49,144,84,89}, ""}, {{157,109,176,77,89}, "돌아옴"},
  {{59,109,176,77,89}, "돌림"}, {{59,77,176,77,89}, "놓기전"},
  {{59,77,176,76,44}, "놓음"}, {{59,147,176,76,92}, "돌아오기전 자세"},
  {{155,149,176,90,61}, "기본자세 복귀"}
};

// low 시퀀스
SequenceStep lowSeq[] = {
  {{155,149,176,90,61}, "기본자세"},
  {{156,103,175,82,44}, "세팅 자세"},
  {{155,32,125,90,47}, "토마토 잡기 직전"},
  {{158,32,126,90,91}, "토마토 잡음"},
  {{158,32,143,90,91}, "내림"},
  {{157,109,176,77,89}, "돌아옴"},
  {{59,109,176,77,89}, "돌림"},
  {{59,77,176,77,89}, "놓음"},
  {{59,77,176,76,44}, "닫기"},
  {{59,147,176,76,92}, "돌아오기전 자세"},
  {{155,149,176,90,61}, "기본자세 복귀"}
};

// ===== D10 서보 CCW→STOP→CW→STOP 한 사이클 =====
bool crActive = false;
int crStep = 0;
unsigned long crStartMs = 0;
const unsigned long DURATION_CCW   = 1000;
const unsigned long DURATION_STOP1 = 1000;
const unsigned long DURATION_CW    = 1000;
const unsigned long DURATION_STOP2 = 1000;
const int SERVO_STOP_US = 1500;
const int SERVO_CW_US   = 1600;
const int SERVO_CCW_US  = 1375;

// ===== 시퀀스 실행 횟수 카운터 =====
int seqCount = 0;

// ==========================
// 부드럽게 서보 이동 함수
// ==========================
void moveServosSmooth(int s1,int s2,int s3,int s4,int s5,const char* label){
  int target[5] = {s1,s2,s3,s4,s5};
  int maxDiff = 0;
  for(int i=0;i<5;i++){ int diff=abs(target[i]-currentAngles[i]); if(diff>maxDiff) maxDiff=diff; }

  for(int step=0;step<=maxDiff;step++){
    for(int i=0;i<5;i++){
      int newAngle = currentAngles[i] + (target[i]-currentAngles[i])*step/maxDiff;
      switch(i){
        case 0: S1.write(newAngle); break;
        case 1: S2.write(newAngle); break;
        case 2: S3.write(newAngle); break;
        case 3: S4.write(newAngle); break;
        case 4: S5.write(newAngle); break;
      }
    }
    delay(15);
  }

  for(int i=0;i<5;i++) currentAngles[i]=target[i];
  delay(500);
  S2.detach(); delay(200); S2.attach(PIN_SERVO_S2);
}

// ==========================
// 시퀀스 실행 함수
// ==========================
void runSequence(SequenceStep* seq,int len){
  for(int i=0;i<len;i++)
    moveServosSmooth(seq[i].angles[0],seq[i].angles[1],seq[i].angles[2],seq[i].angles[3],seq[i].angles[4],seq[i].label);
  seqCount++;
  Serial.println("robot@done"); // 시퀀스 완료 신호
}

// ==========================
// setup
// ==========================
void setup(){
  Serial.begin(9600);

  S1.attach(PIN_SERVO_S1); S2.attach(PIN_SERVO_S2);
  S3.attach(PIN_SERVO_S3); S4.attach(PIN_SERVO_S4);
  S5.attach(PIN_SERVO_S5);

  crServo.attach(PIN_SERVO_CR);
  crServo.writeMicroseconds(SERVO_STOP_US);
  delay(300);

  moveServosSmooth(155,149,176,90,61,"기본자세");
}

// ==========================
// loop
// ==========================
void loop(){
  // UART 명령 처리
  if(Serial.available() > 0){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if(cmd.equalsIgnoreCase("robot@high")) runSequence(highSeq,sizeof(highSeq)/sizeof(highSeq[0]));
    else if(cmd.equalsIgnoreCase("robot@mid")) runSequence(midSeq,sizeof(midSeq)/sizeof(midSeq[0]));
    else if(cmd.equalsIgnoreCase("robot@low")) runSequence(lowSeq,sizeof(lowSeq)/sizeof(lowSeq[0]));
    else if(cmd.equalsIgnoreCase("dump@start")) {
      seqCount = 0;
      crActive = true;
      crStep = 0;
      crStartMs = millis();
    }
  }

  if(crActive){
    unsigned long now = millis();
    switch(crStep){
      case 0:
        crServo.writeMicroseconds(SERVO_CCW_US);
        crStartMs = now;
        crStep++;
        break;

      case 1:
        if(now - crStartMs >= DURATION_CCW){
          crServo.writeMicroseconds(SERVO_STOP_US);
          crStartMs = now;
          crStep++;
        }
        break;

      case 2:
        if(now - crStartMs >= DURATION_STOP1){
          crServo.writeMicroseconds(SERVO_CW_US);
          crStartMs = now;
          crStep++;
        }
        break;

      case 3:
        if(now - crStartMs >= DURATION_CW){
          crServo.writeMicroseconds(SERVO_STOP_US);
          crActive = false;
          Serial.println("dump@finish"); // D10 사이클 완료 신호
        }
        break;
    }
  }
}
