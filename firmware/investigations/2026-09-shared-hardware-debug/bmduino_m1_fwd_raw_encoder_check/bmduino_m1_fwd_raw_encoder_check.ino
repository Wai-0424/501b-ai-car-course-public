// 排除「難轉方向duty→RPM sweep量到0，是不是編碼器解碼邏輯(QDEC_TABLE)
// 正反轉寫錯，把真正有轉動的訊號吃掉」這個假設。
//
// 只測 M1 正轉（CH1=HIGH,CH2=LOW，今天sweep顯示duty<=230全部是0.00那個
// 方向），在duty=100/150/200/230各跑3秒，同時印：
//   1. posM1：正常quadrature解碼後的位置（原本sweep用的邏輯）
//   2. cntD4/cntD5：D4、D5兩根線各自原始toggle次數，完全不經過
//      QDEC_TABLE/方向判斷，純粹「這根線有沒有在動」
// 如果 posM1 全程貼著0、但cntD4/cntD5也全程是0，代表D4/D5兩根線真的
// 沒收到任何訊號變化，不是解碼邏輯把訊號吃掉，是電氣/機械上這個方向
// 真的沒有讓馬達產生（可偵測的）轉動。如果cntD4/cntD5有跳動、posM1卻
// 沒有累積，才是解碼邏輯的問題。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 9;
const uint8_t PIN_M1_A = 2;  // 2026-09-08改接：原本D4接觸不良，改到D2
const uint8_t PIN_M1_B = 3;  // 原本D5有雜訊，改到D3

const uint32_t PWM_PERIOD_US = 200;

volatile int32_t posM1 = 0;
volatile uint8_t stateM1 = 0;
volatile uint32_t cntD4 = 0, cntD5 = 0;

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
void isrA() { cntD4++; updateQuadM1(); }
void isrB() { cntD5++; updateQuadM1(); }

uint8_t m1Duty = 0;

void setDirection(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  if (dir > 0) {
    pwm.setPin(in1Ch, 4095);
    pwm.setPin(in2Ch, 0);
  } else {
    pwm.setPin(in1Ch, 4095);
    pwm.setPin(in2Ch, 4095);  // coast
  }
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)m1Duty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < onUs ? HIGH : LOW);
}

void holdAtDuty(uint8_t duty, uint32_t ms) {
  m1Duty = duty;
  Serial.print(F(">>> duty="));
  Serial.println(duty);
  uint32_t start = millis();
  uint32_t lastPrint = start;
  while (millis() - start < ms) {
    updateMotorPWM();
    if (millis() - lastPrint >= 300) {
      lastPrint = millis();
      Serial.print(F("  posM1="));
      Serial.print(posM1);
      Serial.print(F("  cntD4="));
      Serial.print(cntD4);
      Serial.print(F("  cntD5="));
      Serial.println(cntD5);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrB, CHANGE);

  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  setDirection(CH_M1_IN1, CH_M1_IN2, 1);  // M1 forward，維持整個測試

  Serial.println(F("=== M1 forward raw encoder check (D4/D5 raw toggle, no decode) ==="));
  delay(500);

  uint8_t testDuties[] = {100, 150, 200, 230};
  for (uint8_t i = 0; i < 4; i++) {
    holdAtDuty(testDuties[i], 3000);
  }

  m1Duty = 0;
  pwm.setPin(CH_M1_IN1, 4095);
  pwm.setPin(CH_M1_IN2, 4095);
  Serial.println(F("=== done, motor coast ==="));
}

void loop() {
  updateMotorPWM();
}
