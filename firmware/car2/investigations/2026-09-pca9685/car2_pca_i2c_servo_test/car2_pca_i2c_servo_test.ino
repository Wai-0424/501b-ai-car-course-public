// 排查用：先掃Wire1上的I2C裝置確認PCA9685真的有回應，再讓CH0(轉向servo)
// 來回擺動，用servo有沒有動來區分「PCA9685/I2C整體有沒有問題」還是
// 「只有馬達驅動那段(方向腳/PWM/驅動板)有問題」。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_STEER = 0;
const uint16_t STEER_MIN_US = 500;
const uint16_t STEER_MAX_US = 2500;
const uint16_t STEER_CENTER_US = 1500;

void scanI2C() {
  Serial.println(F("=== Wire1 I2C scan ==="));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    uint8_t err = Wire1.endTransmission();
    if (err == 0) {
      Serial.print(F("Found device at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("No I2C device found on Wire1!"));
  } else {
    Serial.print(found);
    Serial.println(F(" device(s) found."));
  }
  Serial.println(F("======================="));
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Wire1.begin();
  scanI2C();

  pwm.begin();
  pwm.setPWMFreq(50);
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);
  Serial.println(F("=== servo已置中，2秒後開始來回擺動 ==="));
  delay(2000);
}

void loop() {
  Serial.println(F("servo -> 最左(500us)"));
  pwm.writeMicroseconds(CH_STEER, STEER_MIN_US);
  delay(1500);

  Serial.println(F("servo -> 最右(2500us)"));
  pwm.writeMicroseconds(CH_STEER, STEER_MAX_US);
  delay(1500);

  Serial.println(F("servo -> 置中(1500us)"));
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);
  delay(1500);
}
