// 兩輪同時輸出50% duty（128/255），持續運轉不自動停止，方便用電表/
// 示波器量測D8/D9實際PWM波形（5kHz）、或直接觀察兩輪同時轉動的狀態。
//
// 目前接線是交換過的狀態（CH1<->CH2、CH3<->CH4互換，今天驗證用），
// 所以這裡用交換後測出來確實正常出力的方向：M1正轉、M2反轉，避免卡在
// 驅動板那個弱腳位（見今天的正反轉對照測試結果）。
//
// PWM_PERIOD_US=200us -> 5kHz，跟bmduino_car_control.ino/所有今天的
// 測試韌體一致，沒有改動過。
//
// 安全提醒：這份韌體會持續轉動兩輪、不會自動停止，測完記得手動斷電
// 或重新燒別的韌體停止。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t M1_PWM_PIN = 9;

const uint8_t CH_M2_IN1 = 4;
const uint8_t CH_M2_IN2 = 3;
const uint8_t M2_PWM_PIN = 8;

const uint32_t PWM_PERIOD_US = 1000;  // 1000us = 1kHz
const uint8_t DUTY = 77;              // 30%

uint8_t m1Duty = DUTY;
uint8_t m2Duty = DUTY;

void setDir(uint8_t in1Ch, uint8_t in2Ch, int dir) {
  if (dir > 0) { pwm.setPin(in1Ch, 4095); pwm.setPin(in2Ch, 0); }
  else if (dir < 0) { pwm.setPin(in1Ch, 0); pwm.setPin(in2Ch, 4095); }
  else { pwm.setPin(in1Ch, 4095); pwm.setPin(in2Ch, 4095); }  // coast
}

void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  digitalWrite(M1_PWM_PIN, phase < (uint32_t)m1Duty * PWM_PERIOD_US / 255 ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < (uint32_t)m2Duty * PWM_PERIOD_US / 255 ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);

  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  setDir(CH_M1_IN1, CH_M1_IN2, -1);  // M1反轉（交換線後的弱方向，這次故意反過來試）
  setDir(CH_M2_IN1, CH_M2_IN2, 1);   // M2正轉（交換線後的弱方向）

  Serial.println(F("=== Both wheels @ 30% duty (77/255), D8/D9 @ 1kHz, continuous ==="));
  Serial.println(F("M1=reverse(WEAK dir post-swap)  M2=forward(WEAK dir post-swap)"));
  Serial.println(F("Will NOT auto-stop. Reflash or power off to stop."));
}

uint32_t lastPrint = 0;

void loop() {
  updateMotorPWM();
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.println(F("running: M1 duty=77(30%) rev | M2 duty=77(30%) fwd | PWM=1kHz"));
  }
}
