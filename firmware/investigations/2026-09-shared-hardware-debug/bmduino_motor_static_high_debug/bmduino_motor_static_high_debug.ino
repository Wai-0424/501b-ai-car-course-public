// 承接 bmduino_motor_dir_test 完全沒反應的排查：把 IN1~IN4、ENA/ENB 全部
// 固定在 HIGH（懸空/coast 狀態，馬達不會被驅動，安全），而且不用軟體PWM
// 切換、單純靜態 HIGH，方便直接拿電表逐點量測，抓訊號斷在哪一段。
//
// 狀態對照（wiring_plan.md 第6節真值表）：IN1=IN2=HIGH 時，不管 ENA
// 是什麼都是懸空/自由停止，所以這個組合本身就是安全的靜態測試點，
// 不會意外全速轉動。

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

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);

  pwm.setPin(CH_M1_IN1, 4095);  // HIGH
  pwm.setPin(CH_M1_IN2, 4095);  // HIGH
  pwm.setPin(CH_M2_IN1, 4095);  // HIGH
  pwm.setPin(CH_M2_IN2, 4095);  // HIGH

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  digitalWrite(M1_PWM_PIN, HIGH);  // 靜態HIGH，非PWM
  digitalWrite(M2_PWM_PIN, HIGH);

  Serial.println(F("=== DEBUG static HIGH: IN1/IN2/IN3/IN4 + ENA/ENB all HIGH (coast, safe) ==="));
  Serial.println(F("CH1(M1 IN1)=HIGH  CH2(M1 IN2)=HIGH  CH4(M2 IN1)=HIGH  CH3(M2 IN2)=HIGH"));
  Serial.println(F("D9(M1 ENA before 74LS14)=HIGH   D8(M2 ENB before 74LS14)=HIGH"));
  Serial.println(F("All static (no PWM toggling), safe to probe with a multimeter."));
}

void loop() {
  Serial.println(F("holding: IN1-4=HIGH, D8/D9=HIGH"));
  delay(2000);
}
