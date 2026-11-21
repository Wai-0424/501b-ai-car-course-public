// 對照測試：驗證 Serial1 長訊息損毀是不是「只有 F() 巨集字串」才會出現，
// 車1的正式程式（bmduino_car_control.ino）從來只用 char[] RAM 陣列送 STA，
// 從沒用過 F() 巨集送到 SerialBM，卻回報「已大量測試穩定」；懷疑
// bmduino_serial1_length_test.ino 量到的損毀其實是 F() 巨集字串印出實作
// 的 bug，不是 UART 本身或核心通用限制。
//
// 這裡同一組長度，各送一次 F() 版本、一次 RAM char[] 版本，交錯送出，
// 方便直接比對兩者是否有差異。

#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(500);
}

char ramBuf[64];
uint32_t lastMs = 0;
uint8_t phase = 0;

void loop() {
  uint32_t now = millis();
  if (now - lastMs < 300) return;
  lastMs = now;

  switch (phase % 8) {
    case 0:
      Serial1.print(F("ABCDEFGH\n"));
      Serial.println(F("sent F(): ABCDEFGH (9 bytes)"));
      break;
    case 1:
      strcpy(ramBuf, "ABCDEFGH\n");
      Serial1.print(ramBuf);
      Serial.println(F("sent RAM: ABCDEFGH (9 bytes)"));
      break;
    case 2:
      Serial1.print(F("ABCDEFGHIJKLMNOP\n"));
      Serial.println(F("sent F(): ABCDEFGHIJKLMNOP (17 bytes)"));
      break;
    case 3:
      strcpy(ramBuf, "ABCDEFGHIJKLMNOP\n");
      Serial1.print(ramBuf);
      Serial.println(F("sent RAM: ABCDEFGHIJKLMNOP (17 bytes)"));
      break;
    case 4:
      Serial1.print(F("<STA,S1 -1>\n"));
      Serial.println(F("sent F(): <STA,S1 -1> (12 bytes)"));
      break;
    case 5:
      strcpy(ramBuf, "<STA,S1 -1>\n");
      Serial1.print(ramBuf);
      Serial.println(F("sent RAM: <STA,S1 -1> (12 bytes)"));
      break;
    case 6:
      Serial1.print(F("ABCDEFGHIJKLMNOPQRSTUVWX\n"));
      Serial.println(F("sent F(): 25 bytes"));
      break;
    case 7:
      strcpy(ramBuf, "ABCDEFGHIJKLMNOPQRSTUVWX\n");
      Serial1.print(ramBuf);
      Serial.println(F("sent RAM: 25 bytes"));
      break;
  }
  phase++;
}
