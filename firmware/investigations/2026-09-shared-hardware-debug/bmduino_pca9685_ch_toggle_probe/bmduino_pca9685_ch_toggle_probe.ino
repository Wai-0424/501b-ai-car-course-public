// 排除「是程式邏輯寫錯，不是硬體斷路」的疑慮：這份韌體完全不呼叫
// setDir()/M1/M2/forward/reverse 那套抽象，只是最單純地把 CH1~CH4
// 逐一輪流拉 HIGH 幾秒、拉 LOW 幾秒，不斷循環。用電表（或LED+電阻）
// 直接量：
//   1. PCA9685 輸出腳本身有沒有真的在切換（排除PCA9685晶片/I2C本身問題）
//   2. 驅動板對應的 IN 腳有沒有收到一樣的切換（排除PCA9685到驅動板的接線）
// 如果 CH1 在 PCA9685 端量得到切換、但驅動板端量不到，就是接線斷了，
// 不是程式碼問題。全程不驅動 D8/D9（馬達轉速PWM），馬達不會轉動，
// 純粹電氣量測用，安全。
//
// CH1 = M1 IN1　CH2 = M1 IN2　CH3 = M2 IN2(實接驅動板IN4)　CH4 = M2 IN1(實接驅動板IN3)
// 跟 bmduino_car_control.ino / bmduino_motor_dir_test.ino 常數一致。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_STEER = 0;
const uint16_t STEER_CENTER_US = 1500;

const uint8_t CHANNELS[4] = { 1, 2, 3, 4 };
const uint32_t HIGH_MS = 4000;
const uint32_t LOW_MS = 1500;

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  pwm.writeMicroseconds(CH_STEER, STEER_CENTER_US);

  // 確保四個方向腳起始都是 LOW，一次只讓一個 channel 進入 HIGH 狀態，
  // 避免同時多個 channel HIGH 造成驅動板進入非預期狀態。
  for (uint8_t i = 0; i < 4; i++) {
    pwm.setPin(CHANNELS[i], 0);
  }

  Serial.println(F("=== PCA9685 CH1~CH4 toggle probe (Wire1/BMCOM1) ==="));
  Serial.println(F("D8/D9 (motor speed PWM) 全程不驅動，馬達不會轉，純電氣量測。"));
  Serial.println(F("拿電表/LED量對應PCA9685輸出腳 + 驅動板IN腳，確認訊號是否送達。"));
}

void loop() {
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t ch = CHANNELS[i];
    Serial.print(F(">>> CH"));
    Serial.print(ch);
    Serial.println(F(" = HIGH now"));
    pwm.setPin(ch, 4095);
    delay(HIGH_MS);

    Serial.print(F("    CH"));
    Serial.print(ch);
    Serial.println(F(" = LOW"));
    pwm.setPin(ch, 0);
    delay(LOW_MS);
  }
  Serial.println(F("--- cycle done, repeating ---"));
}
