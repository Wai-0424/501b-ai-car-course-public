// 車一 WS2812B 車燈測試(8 顆)：D5(PB3)、D7(PB5) 兩條腳位輪流當資料線，找出燈到底接在哪一條。
// 2026-10-27。驅動方式與時序照抄車二 car2_car_control.ino 的 ledShow(直接寫 GPIO 暫存器 + 忙等迴圈)。
// 安全：D8/D9 (L298N ENA/ENB) 全程拉低，馬達不會動。USB 115200 會印目前的步驟。
#include <Arduino.h>

const uint8_t LED_COUNT = 8;
const uint8_t BRIGHT = 64;  // 0~255，測試先用 1/4 亮度

const uint32_t LED_T0H_LOOPS = 6;
const uint32_t LED_T0L_LOOPS = 15;
const uint32_t LED_T1H_LOOPS = 14;
const uint32_t LED_T1L_LOOPS = 7;

static inline __attribute__((always_inline)) void ledSpin(uint32_t n) {
  __asm volatile(".syntax unified\n\t1: subs %0, %0, #1\n\t bne 1b" : "+l"(n));
}

static void ledSendByte(HT_GPIO_TypeDef* port, uint32_t mask, uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    bool one = (b & 0x80) != 0;
    b <<= 1;
    uint32_t hi = one ? LED_T1H_LOOPS : LED_T0H_LOOPS;
    uint32_t lo = one ? LED_T1L_LOOPS : LED_T0L_LOOPS;
    port->SRR = mask;
    ledSpin(hi);
    port->RR = mask;
    ledSpin(lo);
  }
}

// 把同一個顏色送到某一條資料線；pixels 顆全部同色。
void ledShow(uint8_t pin, uint8_t r, uint8_t g, uint8_t b) {
  r = (uint16_t)r * BRIGHT / 255;
  g = (uint16_t)g * BRIGHT / 255;
  b = (uint16_t)b * BRIGHT / 255;
  HT_GPIO_TypeDef* port = digitalPinToPort(pin);
  uint32_t mask = digitalPinToBitMask(pin);
  uint32_t primask = __get_PRIMASK();
  for (uint8_t n = 0; n < LED_COUNT; n++) {
    __disable_irq();
    ledSendByte(port, mask, g);
    ledSendByte(port, mask, r);
    ledSendByte(port, mask, b);
    if (!primask) __enable_irq();
  }
  port->RR = mask;
}

// 跑馬燈：一次只亮第 idx 顆(確認 8 顆都有、順序正確)。
void ledChase(uint8_t pin, uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
  r = (uint16_t)r * BRIGHT / 255;
  g = (uint16_t)g * BRIGHT / 255;
  b = (uint16_t)b * BRIGHT / 255;
  HT_GPIO_TypeDef* port = digitalPinToPort(pin);
  uint32_t mask = digitalPinToBitMask(pin);
  for (uint8_t n = 0; n < LED_COUNT; n++) {
    __disable_irq();
    bool on = (n == idx);
    ledSendByte(port, mask, on ? g : 0);
    ledSendByte(port, mask, on ? r : 0);
    ledSendByte(port, mask, on ? b : 0);
    __enable_irq();
  }
  port->RR = mask;
}

void setup() {
  Serial.begin(115200);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(7, LOW);
  delay(500);
  Serial.println("car1_led_test start: D8/D9 low (motors off), WS2812B x8 on D5 then D7");
}

void loop() {
  static const uint8_t pins[2] = {5, 7};
  for (uint8_t p = 0; p < 2; p++) {
    uint8_t pin = pins[p];
    Serial.print("--- data pin D"); Serial.println(pin);
    Serial.println("RED");   ledShow(pin, 255, 0, 0);   delay(1500);
    Serial.println("GREEN"); ledShow(pin, 0, 255, 0);   delay(1500);
    Serial.println("BLUE");  ledShow(pin, 0, 0, 255);   delay(1500);
    Serial.println("WHITE"); ledShow(pin, 255, 255, 255); delay(1500);
    Serial.println("CHASE (white, one LED at a time)");
    for (uint8_t round = 0; round < 2; round++)
      for (uint8_t i = 0; i < LED_COUNT; i++) { ledChase(pin, i, 255, 255, 255); delay(250); }
    Serial.println("OFF"); ledShow(pin, 0, 0, 0); delay(1500);
  }
}
