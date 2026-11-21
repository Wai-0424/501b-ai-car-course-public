/*
  bmduino_uart_loopback_diag.ino

  目的:不需要 Jetson,只靠 BMduino 自己驗證 UART TX/RX 損毀問題。
  三個「非 USB」的硬體 UART(Serial3=USART1, Serial1=UART1, Serial4=UART2)
  各自把 TX 腳位用一條杜邦線接回自己的 RX 腳位(self-loopback),
  由同一顆 MCU 送出去、也由同一顆 MCU 收回來,逐 byte 比對送出/收到是否一致。
  結果透過 Serial(USB)印出來給人看,Serial 本身不參與迴接。

  === 接線(依 variant.h 查到的實際腳位)===
    Serial3 (USART1):  TX = PA4   RX = PA5   → 這兩支腳用杜邦線互接
    Serial1 (UART1) :  TX = PC12  RX = PC13  → 這兩支腳用杜邦線互接
    Serial4 (UART2) :  TX = PB2   RX = PB3   → 這兩支腳用杜邦線互接
    Serial  (USART0):  PA8 / PA10 —— 不要接跳線!這兩支腳很可能同時接到板上
                        USB-UART 橋接晶片,迴接會造成訊號衝突。這個 port 只拿來
                        透過 USB 印測試結果,不參與比對。

  === 怎麼看結果 ===
  接好三條跳線、燒錄、打開 Serial Monitor(跟 begin() 設的 baud rate 一致),
  每個 port 會依序測試長度 1~32 byte,每個長度印一行 PASS/FAIL。
  FAIL 的話會印出第一個不一致的位置、預期值 vs 實際收到值,以及這次測試期間
  gDbgFeCount(framing error 計數)、gDbgRxdrCount(收資料事件計數)各自增加了多少
  ——這兩個數字可以直接拿去對照 docs/architecture/HT32_uart_bug_report.md
  第 4.5 節「RX 端雙重讀資料暫存器」那個假說是否成立。
  全部跑完最後會印一個總表,列出每個 port 第一次開始 FAIL 的長度。

  跑完把整個 Serial Monitor 的輸出複製起來就是完整的測試紀錄。
*/

#include <Arduino.h>

#define TEST_BAUD      115200   // 如果正式跟 Jetson 通訊用別的鮑率,這裡也可以改成一樣的來測
#define MAX_LEN        32
#define TIMEOUT_MS     200      // 每次收資料最多等這麼久
#define GAP_MS         20       // 每次測試之間的間隔

// HardwareSerial.cpp 裡宣告的除錯計數器(沒有 static,全域可見)
extern uint32_t gDbgFeCount[];
extern uint32_t gDbgRxdrCount[];

struct PortUnderTest {
  const char*    name;
  HardwareSerial* port;
  uint8_t         bufferIndex;   // 對應 gDbgFeCount / gDbgRxdrCount 的索引
};

PortUnderTest ports[] = {
  { "Serial3 (USART1, PA4/PA5)",  &Serial3, 3 },
  { "Serial1 (UART1,  PC12/PC13)", &Serial1, 1 },
  // Serial4 (UART2, PB2/PB3) 這次先不測,之後要測再取消註解
  // { "Serial4 (UART2,  PB2/PB3)",  &Serial4, 4 },
};
const int NUM_PORTS = sizeof(ports) / sizeof(ports[0]);

int firstFailLen[NUM_PORTS];   // -1 = 全部通過
int failCount[NUM_PORTS];

// 產生跟位置有關、方便辨識的測試內容(避免用 0x00 / 0xFF,才能分辨損毀 vs 正常內容)
uint8_t patternByte(int pos) {
  const char cycle[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  return (uint8_t)cycle[pos % (sizeof(cycle) - 1)];
}

void drainRx(HardwareSerial* port) {
  while (port->available()) port->read();
}

// 送 len byte、收回來比對,回傳 true=完全一致
bool runOneTrial(PortUnderTest& pt, int len, uint8_t* sentOut, uint8_t* recvOut, int* recvLenOut) {
  uint8_t sent[MAX_LEN];
  for (int i = 0; i < len; i++) sent[i] = patternByte(i);

  drainRx(pt.port);

  uint32_t fe0   = gDbgFeCount[pt.bufferIndex];
  uint32_t rxdr0 = gDbgRxdrCount[pt.bufferIndex];

  // 逐 byte 呼叫 write(),模擬 print()/write(buf,len) 實際會發生的呼叫模式
  for (int i = 0; i < len; i++) {
    pt.port->write(sent[i]);
  }

  uint8_t recv[MAX_LEN];
  int recvLen = 0;
  uint32_t t0 = millis();
  while (recvLen < len && (millis() - t0) < TIMEOUT_MS) {
    if (pt.port->available()) {
      recv[recvLen++] = (uint8_t)pt.port->read();
    }
  }
  // 額外多等一下,把可能跟在後面的多餘 byte(例如雙重讀出的幽靈 byte)也收進來
  uint32_t t1 = millis();
  while ((millis() - t1) < 10 && recvLen < MAX_LEN) {
    if (pt.port->available()) {
      recv[recvLen++] = (uint8_t)pt.port->read();
    } else {
      break;
    }
  }

  uint32_t fe1   = gDbgFeCount[pt.bufferIndex];
  uint32_t rxdr1 = gDbgRxdrCount[pt.bufferIndex];

  memcpy(sentOut, sent, len);
  memcpy(recvOut, recv, recvLen);
  *recvLenOut = recvLen;

  bool ok = (recvLen == len);
  if (ok) {
    for (int i = 0; i < len; i++) {
      if (recv[i] != sent[i]) { ok = false; break; }
    }
  }

  Serial.print(F("  len="));
  if (len < 10) Serial.print(' ');
  Serial.print(len);
  Serial.print(F("  recvLen="));
  if (recvLen < 10) Serial.print(' ');
  Serial.print(recvLen);
  Serial.print(F("  feCount+="));
  Serial.print(fe1 - fe0);
  Serial.print(F("  rxdrCount+="));
  Serial.print(rxdr1 - rxdr0);
  Serial.print(F("  -> "));

  if (ok) {
    Serial.println(F("PASS"));
  } else {
    Serial.println(F("FAIL"));
    int cmpLen = min(len, recvLen);
    int mismatches = 0;
    for (int i = 0; i < cmpLen; i++) {
      if (recv[i] != sent[i]) {
        if (mismatches < 5) {  // 最多印前 5 個不一致的位置,避免洗版
          Serial.print(F("      @pos "));
          Serial.print(i);
          Serial.print(F("  sent=0x"));
          if (sent[i] < 16) Serial.print('0');
          Serial.print(sent[i], HEX);
          Serial.print(F(" '"));
          Serial.print((char)sent[i]);
          Serial.print(F("'  recv=0x"));
          if (recv[i] < 16) Serial.print('0');
          Serial.print(recv[i], HEX);
          Serial.print(F(" '"));
          Serial.print((char)recv[i]);
          Serial.println(F("'"));
        }
        mismatches++;
      }
    }
    if (len != recvLen) {
      Serial.print(F("      [長度不一致] 送出 "));
      Serial.print(len);
      Serial.print(F(" byte,收到 "));
      Serial.print(recvLen);
      Serial.println(F(" byte"));
    }
    Serial.print(F("      總共 "));
    Serial.print(mismatches);
    Serial.println(F(" 個位置不一致(只列前5個)"));
  }

  return ok;
}

void setup() {
  Serial.begin(TEST_BAUD);
  delay(1000);
  Serial.println(F("=========================================="));
  Serial.println(F(" HT32 / BMduino UART self-loopback 診斷測試"));
  Serial.println(F("=========================================="));
  Serial.println(F("確認接線:Serial3 PA4<->PA5, Serial1 PC12<->PC13"));
  Serial.println(F("(Serial 本身 PA8/PA10 不要接跳線;Serial4 這次先不測)"));
  Serial.println();

  for (int p = 0; p < NUM_PORTS; p++) {
    ports[p].port->begin(TEST_BAUD);
    firstFailLen[p] = -1;
    failCount[p] = 0;
  }
  delay(200);

  for (int p = 0; p < NUM_PORTS; p++) {
    Serial.print(F("---- 測試 "));
    Serial.print(ports[p].name);
    Serial.println(F(" ----"));

    for (int len = 1; len <= MAX_LEN; len++) {
      uint8_t sentBuf[MAX_LEN], recvBuf[MAX_LEN];
      int recvLen = 0;
      bool ok = runOneTrial(ports[p], len, sentBuf, recvBuf, &recvLen);
      if (!ok) {
        failCount[p]++;
        if (firstFailLen[p] == -1) firstFailLen[p] = len;
      }
      delay(GAP_MS);
    }
    Serial.println();
  }

  Serial.println(F("=========================================="));
  Serial.println(F(" 總結"));
  Serial.println(F("=========================================="));
  for (int p = 0; p < NUM_PORTS; p++) {
    Serial.print(ports[p].name);
    Serial.print(F(" : "));
    if (firstFailLen[p] == -1) {
      Serial.println(F("全部長度(1~32)PASS,完全沒有損毀"));
    } else {
      Serial.print(F("第一次 FAIL 在 len="));
      Serial.print(firstFailLen[p]);
      Serial.print(F(" byte,總共 "));
      Serial.print(failCount[p]);
      Serial.print(F(" / "));
      Serial.print(MAX_LEN);
      Serial.println(F(" 個長度失敗"));
    }
  }
  Serial.println(F("=========================================="));
  Serial.println(F("測試結束。把上面全部輸出複製起來即為完整記錄。"));
}

void loop() {
  // 測試只在 setup() 跑一次,loop() 保持空白
}
