#include <Servo.h>

Servo servo;

// 핀/펄스 설정
const int SERVO_PIN     = 9;
const int SERVO_STOP_US = 1500;
const int SERVO_CW_US   = 1600;
const int SERVO_CCW_US  = 1375;

// 상태 정의
enum State { CW, STOP1, CCW, STOP2 };
State state = STOP2;  // 시작 전 잠깐 정지 상태로 두고 시작

// 각 상태별 지속 시간(ms)
const unsigned long DURATION_CW    = 1000;
const unsigned long DURATION_STOP1 = 1000;
const unsigned long DURATION_CCW   = 1000;
const unsigned long DURATION_STOP2 = 1000;

// 타이머
unsigned long stateStartMs = 0;

void printState(const char* name, unsigned long duration_ms) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] 상태: ");
  Serial.print(name);
  Serial.print(" (");
  Serial.print(duration_ms / 1000.0, 1);
  Serial.println(" s)");
}

void enterState(State s) {
  state = s;
  switch (state) {
    case CW:
      servo.writeMicroseconds(SERVO_CW_US);
      printState("시계방향(CW)", DURATION_CW);
      break;
    case STOP1:
      servo.writeMicroseconds(SERVO_STOP_US);
      printState("정지", DURATION_STOP1);
      break;
    case CCW:
      servo.writeMicroseconds(SERVO_CCW_US);
      printState("반시계방향(CCW)", DURATION_CCW);
      break;
    case STOP2:
      servo.writeMicroseconds(SERVO_STOP_US);
      printState("정지", DURATION_STOP2);
      break;
  }
  stateStartMs = millis();
}

unsigned long stateDuration(State s) {
  switch (s) {
    case CW:    return DURATION_CW;
    case STOP1: return DURATION_STOP1;
    case CCW:   return DURATION_CCW;
    case STOP2: return DURATION_STOP2;
  }
  return 0;
}

State nextState(State s) {
  switch (s) {
    case CW:    return STOP1;
    case STOP1: return CCW;
    case CCW:   return STOP2;
    case STOP2: return CW;
  }
  return CW;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }  // (Leonardo/Micro 대응)
  Serial.println("연속회전 서보 millis 타이머 테스트 시작");

  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(SERVO_STOP_US);
  delay(300);

  // 첫 상태 진입
  enterState(CW);
}

void loop() {
  // 경과 시간 확인 후 상태 전환
  if (millis() - stateStartMs >= stateDuration(state)) {
    enterState(nextState(state));
  }

  // 여기에 나중에 다른 작업(센서 읽기 등) 넣어도 타이밍에 영향 없음
}
