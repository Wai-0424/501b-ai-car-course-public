// 車輛實際馬達 PWM duty -> RPM 掃描測試，對應目前車上實際接線狀態
// （改寫自 hall_motor_pwm_sweep.ino，那份是給另一顆測試馬達用的，
//  腳位/TICKS_PER_REV 跟車輛不一樣，不能直接套用）。
//
// 左右輪依序掃描，各自輸出 CSV，方便之後回歸算 feedforward 常數
// （FF_DEADBAND_*/FF_SLOPE_*，見 bmduino_car_control.ino）。
//
// 腳位對應（跟 bmduino_car_control.ino 目前實際狀態一致）：
//   M1（左輪）：編碼器 D6/D7（現行接線）、
//              PCA9685 CH1(IN1)/CH2(IN2)、PWM 腳 D9
//   M2（右輪）：編碼器 D4/D5、PCA9685 CH4(IN1)/CH3(IN2)、PWM 腳 D8
//
// 測試前務必確認：車輪離地/固定，馬達空轉不會讓車子移動。
// 輸出格式：CSV,<motor>,<duty>,<rpm>
// 測完記得把 bmduino_car_control.ino 燒回去。

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ---- M1（左輪）----
const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 9;
const uint8_t PIN_M1_A = 6;
const uint8_t PIN_M1_B = 7;

// ---- M2（右輪）----
const uint8_t CH_M2_IN1 = 4;
const uint8_t CH_M2_IN2 = 3;
const uint8_t M2_PWM_PIN = 8;
const uint8_t PIN_M2_A = 4;
const uint8_t PIN_M2_B = 5;

const uint32_t PWM_PERIOD_US = 200;  // 5kHz，跟車輛現有設定一致
const float TICKS_PER_REV = 1550.0;  // 車輛實際編碼器估計值（非測試馬達的48）
const uint32_t SETTLE_MS = 700;
const uint32_t MEASURE_MS = 1000;
const uint8_t DUTY_STEP = 10;

volatile int32_t posM1 = 0;
volatile int32_t posM2 = 0;
volatile uint8_t stateM1 = 0;
volatile uint8_t stateM2 = 0;

static const int8_t QDEC_TABLE[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

void updateQuadM1() {
  uint8_t a = digitalRead(PIN_M1_A);
  uint8_t b = digitalRead(PIN_M1_B);
  uint8_t ns = (a << 1) | b;
  posM1 += QDEC_TABLE[((stateM1 << 2) | ns) & 0x0F];
  stateM1 = ns;
}
void updateQuadM2() {
  uint8_t a = digitalRead(PIN_M2_A);
  uint8_t b = digitalRead(PIN_M2_B);
  uint8_t ns = (a << 1) | b;
  posM2 += QDEC_TABLE[((stateM2 << 2) | ns) & 0x0F];
  stateM2 = ns;
}
void isrM1A() { updateQuadM1(); }
void isrM1B() { updateQuadM1(); }
void isrM2A() { updateQuadM2(); }
void isrM2B() { updateQuadM2(); }

uint8_t m1Duty = 0;
uint8_t m2Duty = 0;

void setDirection(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  if (dir > 0) {
    pwm.setPin(in1Ch, 4095);
    pwm.setPin(in2Ch, 0);
  } else if (dir < 0) {
    pwm.setPin(in1Ch, 0);
    pwm.setPin(in2Ch, 4095);
  } else {
    pwm.setPin(in1Ch, 0);
    pwm.setPin(in2Ch, 0);
  }
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t m1OnUs = (uint32_t)m1Duty * PWM_PERIOD_US / 255;
  uint32_t m2OnUs = (uint32_t)m2Duty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < m1OnUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < m2OnUs ? HIGH : LOW);
}

void busyWaitPWM(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    updateMotorPWM();
  }
}

void sweepMotor(const char* label, uint8_t* dutyVar, volatile int32_t* pos,
                uint8_t in1Ch, uint8_t in2Ch) {
  setDirection(in1Ch, in2Ch, 1);
  for (uint16_t duty = 0; duty <= 255; duty += DUTY_STEP) {
    *dutyVar = (uint8_t)duty;
    busyWaitPWM(SETTLE_MS);
    int32_t before = *pos;
    busyWaitPWM(MEASURE_MS);
    int32_t after = *pos;
    int32_t deltaTicks = after - before;
    float rpm = (deltaTicks / TICKS_PER_REV) * (60000.0f / MEASURE_MS);
    Serial.print(F("CSV,"));
    Serial.print(label);
    Serial.print(F(","));
    Serial.print(duty);
    Serial.print(F(","));
    Serial.println(rpm, 2);
  }
  *dutyVar = 0;
  setDirection(in1Ch, in2Ch, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  setDirection(CH_M1_IN1, CH_M1_IN2, 0);
  setDirection(CH_M2_IN1, CH_M2_IN2, 0);

  delay(1000);
  Serial.println(F("=== M1(左輪) sweep start ==="));
  sweepMotor("M1", &m1Duty, &posM1, CH_M1_IN1, CH_M1_IN2);
  Serial.println(F("=== M1 done, M2(右輪) sweep start ==="));
  sweepMotor("M2", &m2Duty, &posM2, CH_M2_IN1, CH_M2_IN2);
  Serial.println(F("=== all sweep done ==="));
}

void loop() {
}
