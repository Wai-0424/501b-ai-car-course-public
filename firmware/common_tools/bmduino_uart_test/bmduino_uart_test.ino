// BMduino UART 通訊測試 — 對應 protocol.md v0.1
// 協定: <TYPE,key value,key value,...>\n
//   CMD (收): <CMD,fwa 90,bwl -60,bwr 60>
//   STA (送): <STA,S1 235>
//
// 現階段可先只用 USB 排線接電腦測試（用 Arduino Serial Monitor 或
// host/pc_test_client.py 接 USB 那個 COM port），CMD/STA 在 USB(Serial)
// 和 BM_COM1(Serial1) 兩條線上都會處理，之後接上 Jetson 也不用改程式。
//
// 若手邊還沒接 PCA9685，先把下面 USE_PCA9685 設成 0，程式會改用
// USB Serial 印出解析結果，方便先驗證通訊本身是否正常。

#include <Arduino.h>

#define USE_PCA9685 0
#define SerialBM Serial1   // BMduino 若把 BM_COM1 對應到別的硬體序列埠，改這裡就好

#if USE_PCA9685
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#endif

// ---- PWM channel 配置 ----
const uint8_t CH_STEER = 0;
const uint8_t CH_LEFT  = 1;
const uint8_t CH_RIGHT = 2;

const uint16_t STEER_MIN_US = 1000;
const uint16_t STEER_MAX_US = 2000;
const uint16_t ESC_NEUTRAL_US    = 1500;
const uint16_t ESC_HALF_RANGE_US = 400;   // 1100~1900us

const uint32_t CMD_TIMEOUT_MS = 500;   // 失效保護：超過這麼久沒收到CMD就煞車
const uint32_t STATUS_PERIOD_MS = 100; // 狀態回報週期

const uint8_t NUM_SENSORS = 1; // 目前只有 1 個佔位感測器，之後要加TOF就把這個數字改大

// ---- 目前狀態 ----
int steerAngle = 90;
int leftSpeed  = 0;
int rightSpeed = 0;
uint32_t lastCmdMs = 0;
uint32_t lastStatusMs = 0;

// ---- 每個 serial 埠各自的收訊緩衝 ----
struct LinkState {
  Stream* port;
  char buf[80];
  uint8_t len;
  bool active;
  uint32_t byteCount; // 除錯用：不管有沒有解析成功，只要收到byte就算
};

LinkState linkUSB = { &Serial,   {0}, 0, false, 0 };
LinkState linkBM  = { &SerialBM, {0}, 0, false, 0 };

void applyCommand(const char* key, int value) {
  if (strcmp(key, "fwa") == 0) {
    steerAngle = constrain(value, 0, 180);
  } else if (strcmp(key, "bwl") == 0) {
    leftSpeed = constrain(value, -100, 100);
  } else if (strcmp(key, "bwr") == 0) {
    rightSpeed = constrain(value, -100, 100);
  }
}

void updateActuators() {
#if USE_PCA9685
  uint16_t steerUs = map(steerAngle, 0, 180, STEER_MIN_US, STEER_MAX_US);
  uint16_t leftUs  = ESC_NEUTRAL_US + (long)leftSpeed  * ESC_HALF_RANGE_US / 100;
  uint16_t rightUs = ESC_NEUTRAL_US + (long)rightSpeed * ESC_HALF_RANGE_US / 100;
  pwm.writeMicroseconds(CH_STEER, steerUs);
  pwm.writeMicroseconds(CH_LEFT, leftUs);
  pwm.writeMicroseconds(CH_RIGHT, rightUs);
#else
  Serial.print(F("[actuator] fwa="));
  Serial.print(steerAngle);
  Serial.print(F(" bwl="));
  Serial.print(leftSpeed);
  Serial.print(F(" bwr="));
  Serial.println(rightSpeed);
#endif
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
  n += snprintf(line + n, sizeof(line) - n, ">\n");

  Serial.print(line);
  SerialBM.print(line);

  Serial.print(F("[dbg] rxBytesUSB="));
  Serial.print(linkUSB.byteCount);
  Serial.print(F(" rxBytesBM="));
  Serial.print(linkBM.byteCount);
  Serial.print(F(" feBM="));
  Serial.print(gDbgFeCount[SERIAL1_BUFFER_INDEX]);
  Serial.print(F(" rxdrBM="));
  Serial.println(gDbgRxdrCount[SERIAL1_BUFFER_INDEX]);
}

void setup() {
  Serial.begin(115200);
  SerialBM.begin(115200);

#if USE_PCA9685
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
#endif

  lastCmdMs = millis();
  updateActuators();
}

void loop() {
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
