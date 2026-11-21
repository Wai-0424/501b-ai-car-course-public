// 測試 Serial1 在不同長度訊息下，傳送損毀的規律：從短到長各送一種
// 固定內容，方便比對「多短才會乾淨」。

#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(500);
}

uint32_t lastMs = 0;
uint8_t phase = 0;

void loop() {
  uint32_t now = millis();
  if (now - lastMs < 300) return;
  lastMs = now;

  switch (phase % 6) {
    case 0: Serial1.print(F("A\n")); Serial.println(F("sent: A\\n (2 bytes)")); break;
    case 1: Serial1.print(F("AB\n")); Serial.println(F("sent: AB\\n (3 bytes)")); break;
    case 2: Serial1.print(F("ABCD\n")); Serial.println(F("sent: ABCD\\n (5 bytes)")); break;
    case 3: Serial1.print(F("ABCDEFGH\n")); Serial.println(F("sent: ABCDEFGH\\n (9 bytes)")); break;
    case 4: Serial1.print(F("ABCDEFGHIJKLMNOP\n")); Serial.println(F("sent: ABCDEFGHIJKLMNOP\\n (17 bytes)")); break;
    case 5: Serial1.print(F("ABCDEFGHIJKLMNOPQRSTUVWX\n")); Serial.println(F("sent: ABCDEFGHIJKLMNOPQRSTUVWX\\n (25 bytes)")); break;
  }
  phase++;
}
