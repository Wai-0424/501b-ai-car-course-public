// 排除「M2的ENB(D8)訊號到底有沒有送到驅動板」的疑慮：只操作D8這根線，
// 在全開(duty=255，等於一直HIGH)跟全關(duty=0，等於一直LOW)之間切換，
// 4秒開、1.5秒關，方便直接用電表量兩個點：
//   1. BMduino D8腳本身（74LS14之前）——確認BMduino真的有輸出訊號
//   2. 驅動板ENB接腳（74LS14之後）——確認訊號有沒有真的送達
// M2方向固定設為正轉（讓驅動板方向端是確定狀態，不影響這個測試的重點）。
// M1完全不驅動（duty=0，coast），避免干擾判斷。
//
// 注意：M2目前已知的bug是「不管duty多少都全速轉」，所以這次測試預期
// 馬達可能全程都在轉，不用意外——這次要看的是D8/ENB這條線本身的電氣
// 訊號，不是馬達轉不轉。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M2_IN1 = 4;
const uint8_t CH_M2_IN2 = 3;
const uint8_t M1_PWM_PIN = 9;
const uint8_t M2_PWM_PIN = 8;

const uint32_t ON_MS = 10000;
const uint32_t OFF_MS = 5000;

void setup() {
  Serial.begin(115200);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  digitalWrite(M1_PWM_PIN, LOW);

  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  pwm.setPin(CH_M2_IN1, 4095);
  pwm.setPin(CH_M2_IN2, 0);  // M2方向固定正轉

  Serial.println(F("=== M2 PWM-only probe: D8 toggling HIGH(10s)/LOW(5s), continuous ==="));
  Serial.println(F("Probe point 1: BMduino D8 pin (before 74LS14)"));
  Serial.println(F("Probe point 2: driver board ENB pin (after 74LS14)"));
  Serial.println(F("M1 untouched (LOW). M2 known bug: may spin full speed regardless of D8 state."));
}

void loop() {
  Serial.println(F(">>> D8 = HIGH"));
  digitalWrite(M2_PWM_PIN, HIGH);
  delay(ON_MS);

  Serial.println(F(">>> D8 = LOW"));
  digitalWrite(M2_PWM_PIN, LOW);
  delay(OFF_MS);
}
