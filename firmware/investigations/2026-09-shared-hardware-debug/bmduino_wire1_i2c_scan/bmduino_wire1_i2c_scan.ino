// 快速驗證用：掃描 BMCOM1（Wire1，SDA1=D20/SCL1=D21）上有沒有 I2C 裝置回應。
// 用來確認把 PCA9685 的 I2C 從主 Wire(A4/A5) 換接到 BMCOM1 之後，
// BMduino 端 Wire1 這條匯流排上抓不抓得到它（預設位址 0x40）。
// 純診斷用，不驅動任何馬達/伺服，跟 bmduino_car_control.ino 無關、不會互相影響。

#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  delay(200);
  Serial.println(F("=== BMCOM1 (Wire1) I2C scan ==="));
}

void loop() {
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
    Serial.println(F("No I2C device found on Wire1 (BMCOM1)."));
  } else {
    Serial.print(found);
    Serial.println(F(" device(s) found."));
  }
  Serial.println(F("---"));
  delay(1500);
}
