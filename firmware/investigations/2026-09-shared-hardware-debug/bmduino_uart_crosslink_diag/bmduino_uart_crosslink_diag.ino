/*
  bmduino_uart_crosslink_diag.ino

  真正跨裝置版本的 UART 診斷測試(對應 bmduino_uart_loopback_diag.ino 的自我迴接版本)。
  這次 Serial1(PC12=TX, PC13=RX)實際接到 Jetson Orin 的 /dev/ttyTHS1
  (Jetson UART1_TXD -> BM PC13/RX, Jetson UART1_RXD <- BM PC12/TX, GND 共地)。

  跟 Jetson 端的 jetson_uart_crosslink_diag.py 配合使用,協定極簡單、
  跟正式的 <TYPE,...> 封包協定無關,純粹是為了逐 byte 驗證用:
    [1 byte 長度 L][L byte 內容]  重複 L=1..32
  內容是跟位置有關的可辨識字元(不是 0x00/0xFF),兩邊各自用同一個公式算,
  不需要互相回報「應該收到什麼」。

  === 測試流程(單次燒錄跑完兩個方向) ===
  1. 開機延遲 3 秒(給 Jetson 腳本足夠時間先進入監聽狀態)
  2. Phase A:BM -> Jetson,送 1~32 byte,驗證交給 Jetson 端(Jetson 印結果)
  3. 間隔 2.5 秒
  4. Phase B:Jetson -> BM,送 1~32 byte,BM 自己驗證,結果透過 USB(Serial)印出來

  === 使用方式 ===
  跟 Jetson 那邊約好時間(Jetson 先背景執行 jetson_uart_crosslink_diag.py),
  燒錄後打開 Serial Monitor(115200)看 Phase B 結果;Phase A 的結果要看 Jetson
  那邊的輸出。
*/

#include <Arduino.h>

#define SerialBM Serial1        // PC12(TX)/PC13(RX),接 Jetson /dev/ttyTHS1
#define TEST_BAUD      115200
#define MAX_LEN        32
#define START_DELAY_MS 3000     // 開機後等這麼久才開始 Phase A,給 Jetson 腳本時間就緒
#define GAP_MS         100      // Phase A/B 每個長度之間的間隔
#define PHASE_GAP_MS   2500     // Phase A 結束到 Phase B 開始的間隔
#define RX_TIMEOUT_MS  400      // Phase B 每個長度等 Jetson 送資料的逾時

#define BM_BUFIDX 1              // Serial1 對應 gDbgFeCount/gDbgRxdrCount 的索引(SERIAL1_BUFFER_INDEX)

extern uint32_t gDbgFeCount[];
extern uint32_t gDbgRxdrCount[];

uint8_t patternByte(int pos) {
  const char cycle[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  return (uint8_t)cycle[pos % (sizeof(cycle) - 1)];
}

void phaseA_sendToJetson() {
  Serial.println(F("-- Phase A: BM -> Jetson (len 1..32) --"));
  Serial.println(F("   (驗證結果請看 Jetson 那邊的輸出)"));
  uint32_t fe0 = gDbgFeCount[BM_BUFIDX];
  uint32_t rxdr0 = gDbgRxdrCount[BM_BUFIDX];

  for (int len = 1; len <= MAX_LEN; len++) {
    SerialBM.write((uint8_t)len);
    for (int i = 0; i < len; i++) {
      SerialBM.write(patternByte(i));
    }
    Serial.print(F("  [TX] len="));
    Serial.println(len);
    delay(GAP_MS);
  }

  uint32_t fe1 = gDbgFeCount[BM_BUFIDX];
  uint32_t rxdr1 = gDbgRxdrCount[BM_BUFIDX];
  Serial.print(F("Phase A 傳送完畢。期間 BM 自己的 feCount+="));
  Serial.print(fe1 - fe0);
  Serial.print(F(" rxdrCount+="));
  Serial.print(rxdr1 - rxdr0);
  Serial.println(F("(理論上應該是0,因為這階段Jetson只收不送)"));
}

// 跟 Jetson 端 phase_a_listen() 邏輯對齊:不假設「第k次讀到的就是第k個長度」,
// 而是持續在一個夠長的絕對時間窗內掃描,只要讀到一個落在 1..MAX_LEN 範圍內的
// byte 就當作長度前綴嘗試組一個 frame,能容忍 Jetson 開始送的時間點沒完全對齊。
#define PHASE_B_DEADLINE_MS 20000

void phaseB_recvFromJetson() {
  Serial.println(F("-- Phase B: Jetson -> BM (len 1..32,用掃描方式收) --"));

  while (SerialBM.available()) SerialBM.read();  // 清掉可能殘留的東西

  int passCount = 0, failCount = 0, gotFrames = 0;
  uint32_t feTotal0 = gDbgFeCount[BM_BUFIDX];
  uint32_t rxdrTotal0 = gDbgRxdrCount[BM_BUFIDX];

  uint32_t deadline = millis() + PHASE_B_DEADLINE_MS;

  while (gotFrames < MAX_LEN && millis() < deadline) {
    if (!SerialBM.available()) continue;
    int len = SerialBM.read();
    if (len < 1 || len > MAX_LEN) continue;  // 不是合法長度前綴,當雜訊丟掉繼續掃

    uint32_t feL0 = gDbgFeCount[BM_BUFIDX];
    uint32_t rxdrL0 = gDbgRxdrCount[BM_BUFIDX];

    uint8_t recv[MAX_LEN];
    int recvLen = 0;
    uint32_t t1 = millis();
    while (recvLen < len && (millis() - t1) < RX_TIMEOUT_MS) {
      if (SerialBM.available()) {
        recv[recvLen++] = (uint8_t)SerialBM.read();
      }
    }

    uint32_t feL1 = gDbgFeCount[BM_BUFIDX];
    uint32_t rxdrL1 = gDbgRxdrCount[BM_BUFIDX];

    gotFrames++;

    bool ok = (recvLen == len);
    if (ok) {
      for (int i = 0; i < len; i++) {
        if (recv[i] != patternByte(i)) { ok = false; break; }
      }
    }

    Serial.print(F("  frame#"));
    if (gotFrames < 10) Serial.print(' ');
    Serial.print(gotFrames);
    Serial.print(F("  lenByte="));
    if (len < 10) Serial.print(' ');
    Serial.print(len);
    Serial.print(F("  recvLen="));
    if (recvLen < 10) Serial.print(' ');
    Serial.print(recvLen);
    Serial.print(F("  fe+="));
    Serial.print(feL1 - feL0);
    Serial.print(F("  rxdr+="));
    Serial.print(rxdrL1 - rxdrL0);
    Serial.print(F("  -> "));

    if (ok) {
      Serial.println(F("PASS"));
      passCount++;
    } else {
      Serial.println(F("FAIL"));
      failCount++;
      int cmpLen = min(len, recvLen);
      int mismatches = 0;
      for (int i = 0; i < cmpLen; i++) {
        if (recv[i] != patternByte(i)) {
          if (mismatches < 5) {
            Serial.print(F("      @pos "));
            Serial.print(i);
            Serial.print(F("  expected=0x"));
            uint8_t exp = patternByte(i);
            if (exp < 16) Serial.print('0');
            Serial.print(exp, HEX);
            Serial.print(F(" '"));
            Serial.print((char)exp);
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
      if (recvLen != len) {
        Serial.print(F("      [長度不一致] 長度前綴="));
        Serial.print(len);
        Serial.print(F(",實際收到 "));
        Serial.print(recvLen);
        Serial.println(F(" byte"));
      }
      Serial.print(F("      總共 "));
      Serial.print(mismatches);
      Serial.println(F(" 個位置不一致(只列前5個)"));
    }
  }

  uint32_t feTotal1 = gDbgFeCount[BM_BUFIDX];
  uint32_t rxdrTotal1 = gDbgRxdrCount[BM_BUFIDX];

  Serial.println(F("=========================================="));
  Serial.print(F("Phase B 總結: 收到 "));
  Serial.print(gotFrames);
  Serial.print(F(" / 32 筆   PASS="));
  Serial.print(passCount);
  Serial.print(F("   FAIL="));
  Serial.println(failCount);
  if (gotFrames < MAX_LEN) {
    Serial.println(F("注意:未滿32筆就到達監聽上限時間"));
  }
  Serial.print(F("整個 Phase B 期間 feCount+="));
  Serial.print(feTotal1 - feTotal0);
  Serial.print(F("  rxdrCount+="));
  Serial.println(rxdrTotal1 - rxdrTotal0);
  Serial.println(F("=========================================="));
}

void setup() {
  Serial.begin(TEST_BAUD);
  SerialBM.begin(TEST_BAUD);
  delay(1000);

  Serial.println(F("=========================================="));
  Serial.println(F(" BMduino <-> Jetson 跨裝置 UART 診斷測試"));
  Serial.println(F(" Serial1(PC12 TX / PC13 RX) <-> Jetson /dev/ttyTHS1"));
  Serial.println(F("=========================================="));
  Serial.print(F("開機延遲 "));
  Serial.print(START_DELAY_MS);
  Serial.println(F(" ms,等 Jetson 端腳本就緒..."));

  delay(START_DELAY_MS);

  phaseA_sendToJetson();
  delay(PHASE_GAP_MS);
  phaseB_recvFromJetson();

  Serial.println(F("測試結束。把上面全部輸出複製起來即為完整記錄。"));
}

void loop() {
  // 只在 setup() 跑一次
}
