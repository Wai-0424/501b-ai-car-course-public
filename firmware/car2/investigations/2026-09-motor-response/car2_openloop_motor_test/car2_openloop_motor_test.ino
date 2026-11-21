// 排查用：完全繞開CMD協定/PID/編碼器，開機直接固定duty正轉兩顆馬達，
// 純粹確認「訊號真的送到驅動板時，馬達到底會不會動」。腳位常數直接
// 沿用car2_speed_control_test.ino目前確認正確的版本，不是舊的猜測值。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M1_IN1 = 2;
const uint8_t CH_M1_IN2 = 1;
const uint8_t CH_M2_IN1 = 4;
const uint8_t CH_M2_IN2 = 3;

const uint8_t M1_PWM_PIN = 8;
const uint8_t M2_PWM_PIN = 9;

const uint32_t PWM_PERIOD_US = 200;  // 5kHz
const uint8_t TEST_DUTY = 180;       // 固定duty，明顯高於死區(約19~45)，正常應該會轉

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  // 正轉方向（跟driveDirection()的dir>0一致）
  pwm.setPin(CH_M1_IN1, 4095);
  pwm.setPin(CH_M1_IN2, 0);
  pwm.setPin(CH_M2_IN1, 4095);
  pwm.setPin(CH_M2_IN2, 0);

  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);

  Serial.println(F("=== 開迴路測試：M1/M2固定duty=180正轉，開機直接開始 ==="));
}

void loop() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t onUs = (uint32_t)TEST_DUTY * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < onUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < onUs ? HIGH : LOW);

  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.println(F("driving: M1/M2 duty=180 forward"));
  }
}
