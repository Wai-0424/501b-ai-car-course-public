// ESP32(NodeMCU-32S) UART測試發送端 — 用來獨立驗證BMduino BM_COM1的RX是否能收到「任何」外部裝置的訊號
// 接線：ESP32 GPIO17(TX2) -> BMduino BM_COM1 pin3(RX1)，GND -> GND
// 選配：ESP32 GPIO16(RX2) <- BMduino BM_COM1 pin4(TX1)，可同時驗證BM->ESP32方向

HardwareSerial BMLink(2); // UART2

uint32_t counter = 0;

void setup() {
  Serial.begin(115200);
  BMLink.begin(115200, SERIAL_8N1, 16, 17); // RX2=16, TX2=17
  Serial.println("ESP32 UART test start, TX2=GPIO17 RX2=GPIO16");
}

void loop() {
  counter++;
  char buf[48];
  snprintf(buf, sizeof(buf), "<CMD,fwa %d,bwl 10,bwr -10>\n", 90 + (counter % 20));
  BMLink.print(buf);
  Serial.print("[TX] ");
  Serial.print(buf);

  uint32_t start = millis();
  while (millis() - start < 400) {
    while (BMLink.available()) {
      Serial.write(BMLink.read());
    }
  }
}
