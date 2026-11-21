// BMduino 車輛控制韌體 — 版本1（BM GPIO 方向 + 單級 74LS14 反相緩衝）
// 對應 protocol.md v0.1 + wiring_plan.md 第6節版本1。
// 已實機驗證可動（正反轉、0~80% 調速、servo 同步動作都正常）。
// 跟版本2（bmduino_car_control/bmduino_car_control.ino）的差別：
// 這版方向腳也是走 BMduino GPIO，經過 74LS14 才變成驅動板要的 5V；
// 版本2 方向腳改走 PCA9685 直接輸出數位 0/1，不需要 74LS14。
//
// 馬達驅動板方向腳（IN1~IN4）、轉速腳（ENA/ENB）都需要接近 5V 的 TTL
// 準位，BMduino GPIO 直接輸出只有 3.3V 吃不到門檻。這裡用 BMduino
// 輸出訊號 → 74LS14（TTL 反相緩衝器，5V 供電）→ 驅動板，
// 用 7414 把訊號整形成乾淨、真正的 5V。
//
// 74LS14 是「反相器」，只用一級的話，BMduino 這邊要送出「跟驅動板端
// 想要的邏輯相反」的值，7414 出來才會是正確的邏輯，韌體裡已經處理好
// 這個反相對應，接線時 74LS14 中間不用再加第二級反相。
//   方向腳：digitalWrite 的 HIGH/LOW 已經是反相後的值
//   轉速腳：實測發現這片 BMduino(HT32核心) 的 D8/D9 用內建
//           analogWrite() 送出來的 PWM 沒有正常作用（驅動板收不到有
//           效訊號，馬達不轉），改成自己用 digitalWrite + micros() 寫
//           軟體 PWM（updateMotorPWM()，1kHz），實測 digitalWrite 在
//           這兩支腳上是正常的，所以繞開 analogWrite 這個問題。
//
// 有一個好處：driveMotor() 遇到 speed==0（煞車、也是失效保護觸發後的
// 預設狀態）時，BMduino 端輸出的是 HIGH/HIGH/duty=0（也就是全部拉高），
// 這樣即使開機瞬間 BMduino 還沒跑到 setup()、GPIO 還沒被驅動之前，只
// 要在 74LS14 每個輸入腳外加一顆上拉電阻到 5V（建議加，避免浮接時
// 7414 輸出不確定），開機的預設狀態就會落在「安全」的煞車/ENA=LOW，
// 不會亂衝。
//
// BMduino 腳位／功能對應（接到 74LS14 輸入端）：
//   D2 → 74LS14 → M1 驅動板 IN1
//   D3 → 74LS14 → M1 驅動板 IN2
//   D4 → 74LS14 → M2 驅動板 IN1(IN3)
//   D5 → 74LS14 → M2 驅動板 IN2(IN4)
//   D9 → 74LS14 → M1 驅動板 ENA（轉速 PWM）
//   D8 → 74LS14 → M2 驅動板 ENB（轉速 PWM）
// 實際接線測試結果：協定裡的 M1（bwl）實際接的是「車體右輪」，
// M2（bwr）實際接的是「車體左輪」，跟命名剛好左右相反，功能上沒有
// 影響，只是命名對調，先記錄下來備查。
// 前輪轉向 servo 維持走 PCA9685 CH0，不受這次改動影響。

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SerialBM Serial1   // BM_COM1 對應到別的硬體序列埠的話，改這裡就好

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ---- PCA9685 channel 配置（只剩 servo）----
const uint8_t CH_STEER = 0;

// ---- 馬達轉速 PWM（BMduino 軟體 PWM，經 74LS14 單級反相緩衝）----
const uint8_t M1_PWM_PIN = 9;  // D9 → 74LS14 → M1 ENA
const uint8_t M2_PWM_PIN = 8;  // D8 → 74LS14 → M2 ENB

// ---- 馬達方向 GPIO（經 74LS14 單級反相緩衝）----
const uint8_t M1_IN1 = 2;  // D2 → 74LS14 → M1 驅動板 IN1
const uint8_t M1_IN2 = 3;  // D3 → 74LS14 → M1 驅動板 IN2
const uint8_t M2_IN1 = 4;  // D4 → 74LS14 → M2 驅動板 IN1(IN3)
const uint8_t M2_IN2 = 5;  // D5 → 74LS14 → M2 驅動板 IN2(IN4)

// 實測 servo（TBSK20）datasheet：500~2500us 對應 0~180°，中立點 1500us
const uint16_t STEER_MIN_US = 500;
const uint16_t STEER_MAX_US = 2500;

const uint32_t CMD_TIMEOUT_MS = 500;    // 失效保護：超過這麼久沒收到CMD就煞車
const uint32_t STATUS_PERIOD_MS = 100;  // 狀態回報週期

const uint8_t NUM_SENSORS = 1;  // 目前只有1個佔位感測器，之後要加TOF就把這個數字改大

const uint32_t PWM_PERIOD_US = 1000;  // 軟體 PWM 週期，1000us = 1kHz

// ---- 目前狀態 ----
int steerAngle = 0;  // -90~90，0=直行、負=左轉、正=右轉
int leftSpeed = 0;
int rightSpeed = 0;
uint8_t m1DriverDuty = 0;  // 驅動板端 ENA 實際想要的 duty（0~255，0=關閉）
uint8_t m2DriverDuty = 0;  // 驅動板端 ENB 實際想要的 duty
uint32_t lastCmdMs = 0;
uint32_t lastStatusMs = 0;

// ---- 每個 serial 埠各自的收訊緩衝 ----
struct LinkState {
  Stream* port;
  char buf[80];
  uint8_t len;
  bool active;
  uint32_t byteCount;
};

LinkState linkUSB = { &Serial, {0}, 0, false, 0 };
LinkState linkBM = { &SerialBM, {0}, 0, false, 0 };

void applyCommand(const char* key, int value) {
  if (strcmp(key, "fwa") == 0) {
    steerAngle = constrain(value, -90, 90);  // 0=直行、負=左轉、正=右轉
  } else if (strcmp(key, "bwl") == 0) {
    leftSpeed = constrain(value, -100, 100);
  } else if (strcmp(key, "bwr") == 0) {
    rightSpeed = constrain(value, -100, 100);
  }
}

// speed: -100~100，正=正轉、負=反轉、0=剎車（驅動板端 IN1/IN2 皆 LOW，
// 對應 wiring_plan.md 4.2 控制邏輯表的「剎車(急停)」那一列）。
// 方向腳的 digitalWrite 送出的是「反相後」的值（因為要經過 74LS14
// 反相緩衝），驅動板端實際看到的才會是註解裡寫的邏輯。轉速改成只
// 記錄「驅動板端想要的 duty」，實際 PWM 波形交給 updateMotorPWM()
// 在 loop() 裡持續產生。
void driveMotor(uint8_t in1Pin, uint8_t in2Pin, int speed, uint8_t* driverDutyOut) {
  if (speed == 0) {
    // 想要驅動板端 IN1=LOW, IN2=LOW（煞車）→ BMduino 端送 HIGH, HIGH
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
    *driverDutyOut = 0;  // 驅動板端 ENA=0%（關閉）
    return;
  }
  if (speed > 0) {
    // 想要驅動板端 IN1=HIGH, IN2=LOW → BMduino 端送 LOW, HIGH
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    // 想要驅動板端 IN1=LOW, IN2=HIGH → BMduino 端送 HIGH, LOW
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  }
  *driverDutyOut = map(abs(speed), 0, 100, 0, 255);
}

void updateActuators() {
  uint16_t steerUs = map(steerAngle, -90, 90, STEER_MIN_US, STEER_MAX_US);
  pwm.writeMicroseconds(CH_STEER, steerUs);
  driveMotor(M1_IN1, M1_IN2, leftSpeed, &m1DriverDuty);
  driveMotor(M2_IN1, M2_IN2, rightSpeed, &m2DriverDuty);
}

// 軟體 PWM：每次 loop() 都呼叫，靠 micros() 算目前在週期內的相位，
// 決定 BMduino 端這一瞬間該輸出 HIGH 還是 LOW。因為要經過 74LS14
// 反相，驅動板端想要 HIGH（duty 內）的時間，BMduino 端要輸出 LOW。
void updateMotorPWM() {
  uint32_t phase = micros() % PWM_PERIOD_US;
  uint32_t m1OnUs = (uint32_t)m1DriverDuty * PWM_PERIOD_US / 255;
  uint32_t m2OnUs = (uint32_t)m2DriverDuty * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < m1OnUs ? LOW : HIGH);
  digitalWrite(M2_PWM_PIN, phase < m2OnUs ? LOW : HIGH);
}

// 感測器讀值佔位，之後接 TOF 等模組就把這裡換成實際讀取。
// 回傳 -1 代表這個感測器目前無效/未接。
int16_t readSensorMM(uint8_t index) {
  (void)index;
  return -1;
}

void handleFrame(char* frame) {
  char* tok = strtok(frame, ",");
  if (tok == nullptr || strcmp(tok, "CMD") != 0) return;

  while ((tok = strtok(nullptr, ",")) != nullptr) {
    char key[8];
    int value;
    if (sscanf(tok, "%7s %d", key, &value) == 2) {
      applyCommand(key, value);
    }
  }

  lastCmdMs = millis();
  updateActuators();
}

void pollLink(LinkState& ls) {
  while (ls.port->available()) {
    char c = ls.port->read();
    ls.byteCount++;
    if (c == '<') {
      ls.len = 0;
      ls.active = true;
      continue;
    }
    if (!ls.active) continue;
    if (c == '>') {
      ls.buf[ls.len] = '\0';
      handleFrame(ls.buf);
      ls.active = false;
      ls.len = 0;
      continue;
    }
    if (ls.len < sizeof(ls.buf) - 1) {
      ls.buf[ls.len++] = c;
    } else {
      // 緩衝溢位，放棄這個封包重新同步
      ls.active = false;
      ls.len = 0;
    }
  }
}

void sendStatus() {
  char line[80];
  int n = snprintf(line, sizeof(line), "<STA");
  for (uint8_t i = 1; i <= NUM_SENSORS; i++) {
    int16_t d = readSensorMM(i);
    n += snprintf(line + n, sizeof(line) - n, ",S%u %d", i, d);
  }
  // 除錯欄位：直接印出韌體內部算出來的值，確認 CMD -> speed -> duty 這條路有沒有正確運作
  n += snprintf(line + n, sizeof(line) - n, ",dbg_ls %d,dbg_rs %d,dbg_d1 %u,dbg_d2 %u",
                leftSpeed, rightSpeed, m1DriverDuty, m2DriverDuty);
  n += snprintf(line + n, sizeof(line) - n, ">\n");

  Serial.print(line);
  SerialBM.print(line);
}

void setup() {
  Serial.begin(115200);
  SerialBM.begin(115200);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  lastCmdMs = millis();
  updateActuators();
}

void loop() {
  updateMotorPWM();

  pollLink(linkUSB);
  pollLink(linkBM);

  uint32_t now = millis();

  if (now - lastCmdMs > CMD_TIMEOUT_MS) {
    if (leftSpeed != 0 || rightSpeed != 0) {
      leftSpeed = 0;
      rightSpeed = 0;
      updateActuators();
    }
  }

  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    sendStatus();
  }
}
