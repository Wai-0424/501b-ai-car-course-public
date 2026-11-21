// 承接 bmduino_motor_coast_pwm50_debug：把 CH1、CH3 拉低（對應驅動板
// IN1、IN3），CH2、CH4 維持 HIGH，D8/D9(ENA/ENB) 維持 5kHz 50% duty，
// 讓兩顆馬達這次真的進入「有方向」的驅動狀態（不再是 IN1=IN2 的懸空/coast）。
//
// 注意：這裡直接照 CH 編號對應 IN 編號（CH1->IN1, CH2->IN2, CH3->IN3,
// CH4->IN4），跟 bmduino_car_control.ino 裡car1那組「CH3/CH4 對調」的
// 常數不一樣——這台車(car2)的 CH3/CH4 是否也對調過，正是這次要測試驗證
// 的東西之一，先照單純的編號對應測，之後照實測結果再決定要不要對調。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_STEER = 0;
const uint8_t M1_PWM_PIN = 9;  // D9 -> 74LS14 -> M1 ENA
const uint8_t M2_PWM_PIN = 8;  // D8 -> 74LS14 -> M2 ENB

const uint16_t STEER_CENTER_US = 1500;
const uint32_t PWM_PERIOD_US = 200;  // 5kHz
const uint8_t TEST_DUTY = 128;       // 50%

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);

  pwm.setPin(1, 0);      // CH1(IN1) = LOW
  pwm.setPin(2, 4095);   // CH2(IN2) = HIGH
  pwm.setPin(3, 0);      // CH3(IN3) = LOW
  pwm.setPin(4, 4095);   // CH4(IN4) = HIGH

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);

  Serial.println(F("=== CH1=LOW,CH2=HIGH,CH3=LOW,CH4=HIGH + D8/D9 50% PWM ==="));
}

void loop() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)TEST_DUTY * PWM_PERIOD_US / 255;
  bool high = phase < onUs;
  digitalWrite(M1_PWM_PIN, high ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, high ? HIGH : LOW);
}
