// 承接 bmduino_motor_static_high_debug：方向腳（CH1~CH4）維持在已驗證過的
// 安全靜態狀態（IN1=IN2=HIGH，兩顆馬達都是懸空/coast，不會被真的驅動），
// 但這次 D8/D9(ENA/ENB) 改成真正的 5kHz 軟體PWM、50% duty，而不是靜態HIGH，
// 方便直接量測/示波器看 PWM 訊號本身有沒有正確產生、有沒有正確穿過74LS14。
//
// 因為 IN1=IN2=HIGH 是 coast 狀態，依真值表這個狀態下 ENA/ENB 是什麼都不會
// 讓馬達轉動，所以這個測試「不會轉」是預期中的正常現象，純粹是拿來確認
// PWM 訊號本身波形/電壓正確，跟馬達會不會轉是分開的兩件事。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_STEER = 0;
const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t CH_M2_IN1 = 4;  // 實接驅動板 IN3
const uint8_t CH_M2_IN2 = 3;  // 實接驅動板 IN4

const uint8_t M1_PWM_PIN = 9;  // D9 -> 74LS14 -> M1 ENA
const uint8_t M2_PWM_PIN = 8;  // D8 -> 74LS14 -> M2 ENB

const uint16_t STEER_CENTER_US = 1500;
const uint32_t PWM_PERIOD_US = 200;  // 5kHz，跟正式韌體一致
const uint8_t TEST_DUTY = 128;       // 50%

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);

  pwm.setPin(CH_M1_IN1, 4095);  // IN1=IN2=HIGH -> coast，安全，不會真的驅動
  pwm.setPin(CH_M1_IN2, 4095);
  pwm.setPin(CH_M2_IN1, 4095);
  pwm.setPin(CH_M2_IN2, 4095);

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);

  Serial.println(F("=== DEBUG: CH1-4 HIGH (coast, safe) + D8/D9 real 5kHz 50% PWM ==="));
  Serial.println(F("量測D9/D8或74LS14輸出端：應該看到方波，或電表量到約一半電壓的平均值"));
}

void loop() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)TEST_DUTY * PWM_PERIOD_US / 255;
  bool high = phase < onUs;
  digitalWrite(M1_PWM_PIN, high ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, high ? HIGH : LOW);
}
