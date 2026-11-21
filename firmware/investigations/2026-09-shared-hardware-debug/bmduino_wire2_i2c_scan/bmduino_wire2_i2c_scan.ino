// 確認 PCA9685 換接到 BMCOM2(Wire2)之後，I2C 掃描抓不抓得到。

#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire2.begin();
  delay(200);
  Serial.println(F("=== BMCOM2 (Wire2) I2C scan ==="));
}

void loop() {
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire2.beginTransmission(addr);
    uint8_t err = Wire2.endTransmission();
    if (err == 0) {
      Serial.print(F("Found device at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("No I2C device found on Wire2 (BMCOM2)."));
  } else {
    Serial.print(found);
    Serial.println(F(" device(s) found."));
  }
  Serial.println(F("---"));
  delay(1500);
}
