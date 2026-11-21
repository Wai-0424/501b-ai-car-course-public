// 銜接 bmduino_wire1_i2c_scan 的下一步驗證：確認 PCA9685 換接到 BMCOM1(Wire1)
// 之後，實際輸出 PWM 也正常、servo 真的會動。只驅動 CH0(前輪轉向 servo)，
// 其餘 CH1~CH4(馬達方向)、D8/D9(馬達轉速 PWM)完全不設定、不輸出，
// 避免還沒確認馬達驅動板接線前就誤動作。
//
// 動作方式：CH0 在 944us~2056us 之間緩慢來回掃動，這個範圍對應
// bmduino_car_control.ino 目前的轉向安全上限 ±50°（同一個 map()，
// STEER_MIN_US/MAX_US=500/2500 對應 -90~90，50°處算出來就是這個值），
// 不會超出實車轉向連桿目前驗證過的安全角度。每一步只移動1us、間隔15ms，
// 讓動作看起來平滑、也比較不會對還沒確認妥當的連桿/機構造成衝擊。
//
// 確認流程都沒問題後，才把這裡的 Wire1 建構子搬回正式版
// bmduino_car_control.ino。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_STEER = 0;
const uint16_t STEER_CENTER_US = 1500;  // 中立點，跟 bmduino_car_control.ino 一致
const uint16_t STEER_MIN_US = 944;      // 對應 -50°（安全上限）
const uint16_t STEER_MAX_US = 2056;     // 對應 +50°（安全上限）
const uint16_t STEP_US = 1;
const uint16_t STEP_DELAY_MS = 15;

uint16_t currentUs = STEER_CENTER_US;
int8_t direction = 1;  // 1=往MAX走，-1=往MIN走

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  pwm.writeMicroseconds(CH_STEER, currentUs);

  Serial.println(F("CH0 servo sweep test via Wire1 (BMCOM1)."));
  Serial.println(F("CH1~CH4 / D8 / D9 untouched, no output."));
  Serial.println(F("Sweeping CH0 between 944us(-50deg) and 2056us(+50deg)."));
}

void loop() {
  currentUs += direction * STEP_US;
  if (currentUs >= STEER_MAX_US) {
    currentUs = STEER_MAX_US;
    direction = -1;
  } else if (currentUs <= STEER_MIN_US) {
    currentUs = STEER_MIN_US;
    direction = 1;
  }

  pwm.writeMicroseconds(CH_STEER, currentUs);

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print(F("CH0="));
    Serial.print(currentUs);
    Serial.println(F("us"));
  }

  delay(STEP_DELAY_MS);
}
