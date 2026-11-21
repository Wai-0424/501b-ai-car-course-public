/*
 * 車 1 L298N 第一階段診斷韌體。
 * 只使用 USB Serial（115200）接受明確短脈衝，不使用 Orin UART。
 * 開機和每次測試後都將 ENA/ENB 關閉；沒有自動馬達動作。
 *
 * 初始接線假設，皆須以實車診斷結果確認：
 * M1: PCA CH1/CH2 -> IN1/IN2, D8 -> ENA, encoder D2/D3
 * M2: PCA CH3/CH4 -> IN3/IN4, D9 -> ENB, encoder D6/D7
 *
 * USB commands:
 *   help | status | reset | stop
 *   pulse M1 F 20 250
 *   pulse M1 R 20 250
 *   pulse M2 F 20 250
 *   pulse M2 R 20 250
 * During a pulse, send ! to abort.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire1);

const uint8_t CH_M1_IN1 = 1;
const uint8_t CH_M1_IN2 = 2;
const uint8_t CH_M2_IN1 = 3;
const uint8_t CH_M2_IN2 = 4;

const uint8_t M1_PWM_PIN = 8;
const uint8_t M2_PWM_PIN = 9;
const uint8_t PIN_M1_A = 2;
const uint8_t PIN_M1_B = 3;
const uint8_t PIN_M2_A = 6;
const uint8_t PIN_M2_B = 7;

const uint32_t PWM_PERIOD_US = 200;  // 5 kHz
const uint8_t MIN_DIAGNOSTIC_DUTY = 20;
// The calibration sweep stays below the previously proven full-duty test but
// spans the L298N deadband and its usable low/mid-speed range.
const uint8_t MAX_DIAGNOSTIC_DUTY = 255;
const uint16_t MIN_PULSE_MS = 50;
const uint16_t MAX_PULSE_MS = 6000;
const uint16_t PROFILE_BIN_MS = 500;
const uint8_t PROFILE_MAX_BINS = MAX_PULSE_MS / PROFILE_BIN_MS;
const uint16_t DIRECTION_SETTLE_MS = 250;
const uint16_t PWM_SWEEP_STEP_MS = 20;
const uint16_t PCA_WALK_STEP_MS = 2000;

volatile int32_t posM1 = 0;
volatile int32_t posM2 = 0;
volatile uint8_t stateM1 = 0;
volatile uint8_t stateM2 = 0;
volatile uint32_t invalidM1 = 0;
volatile uint32_t invalidM2 = 0;
volatile uint32_t aRiseM1 = 0;
volatile uint32_t aRiseM2 = 0;
static const int8_t QDEC_TABLE[16] = {
  0, -1, 1, 0, 1, 0, 0, -1,
  -1, 0, 0, 1, 0, 1, -1, 0
};

uint8_t m1Duty = 0;
uint8_t m2Duty = 0;
bool pwmSweepEnabled = false;
uint8_t pwmSweepDuty = 0;
int8_t pwmSweepStep = 1;
uint32_t lastPwmSweepStepMs = 0;
bool pcaWalkEnabled = false;
uint8_t pcaWalkChannel = 1;
uint32_t lastPcaWalkStepMs = 0;
bool dualPwmFullEnabled = false;
char lineBuffer[64];
uint8_t lineLength = 0;

void updateQuadM1() {
  const uint8_t previous = stateM1;
  const uint8_t next = (digitalRead(PIN_M1_A) << 1) | digitalRead(PIN_M1_B);
  const int8_t step = QDEC_TABLE[((previous << 2) | next) & 0x0F];
  if (next != previous && step == 0) invalidM1++;
  if (!(previous & 2) && (next & 2)) aRiseM1++;
  posM1 += step;
  stateM1 = next;
}

void updateQuadM2() {
  const uint8_t previous = stateM2;
  const uint8_t next = (digitalRead(PIN_M2_A) << 1) | digitalRead(PIN_M2_B);
  const int8_t step = QDEC_TABLE[((previous << 2) | next) & 0x0F];
  if (next != previous && step == 0) invalidM2++;
  if (!(previous & 2) && (next & 2)) aRiseM2++;
  posM2 += step;
  stateM2 = next;
}

void isrM1A() { updateQuadM1(); }
void isrM1B() { updateQuadM1(); }
void isrM2A() { updateQuadM2(); }
void isrM2B() { updateQuadM2(); }

void setDirection(uint8_t in1, uint8_t in2, int8_t direction) {
  if (direction > 0) {
    pca.setPin(in1, 4095);
    pca.setPin(in2, 0);
  } else if (direction < 0) {
    pca.setPin(in1, 0);
    pca.setPin(in2, 4095);
  } else {
    // stopAll() sets EN low, so both L298N outputs are disabled.
    pca.setPin(in1, 0);
    pca.setPin(in2, 0);
  }
}

void updateMotorPwm() {
  const uint32_t phase = micros() % PWM_PERIOD_US;
  const uint32_t m1OnUs = static_cast<uint32_t>(m1Duty) * PWM_PERIOD_US / 255;
  const uint32_t m2OnUs = static_cast<uint32_t>(m2Duty) * PWM_PERIOD_US / 255;
  digitalWrite(M1_PWM_PIN, phase < m1OnUs ? HIGH : LOW);
  digitalWrite(M2_PWM_PIN, phase < m2OnUs ? HIGH : LOW);
}

void stopAll() {
  pwmSweepEnabled = false;
  pcaWalkEnabled = false;
  dualPwmFullEnabled = false;
  m1Duty = 0;
  m2Duty = 0;
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);
  setDirection(CH_M1_IN1, CH_M1_IN2, 0);
  setDirection(CH_M2_IN1, CH_M2_IN2, 0);
}

// D8/D9 signal-only scope test. Both L298N direction pairs stay LOW, so
// ENA/ENB may carry PWM while neither motor bridge is commanded to drive.
void startPwmSweep() {
  stopAll();
  pwmSweepDuty = 0;
  pwmSweepStep = 1;
  lastPwmSweepStepMs = millis();
  pwmSweepEnabled = true;
  Serial.println(F("PWM_SWEEP,start,D8_D9,carrier_hz=5000,duty=0..255..0,step_ms=20"));
}

void updatePwmSweep() {
  if (!pwmSweepEnabled) return;
  const uint32_t now = millis();
  if (now - lastPwmSweepStepMs < PWM_SWEEP_STEP_MS) return;

  lastPwmSweepStepMs = now;
  m1Duty = pwmSweepDuty;
  m2Duty = pwmSweepDuty;
  if (pwmSweepDuty == 255) {
    pwmSweepStep = -1;
  } else if (pwmSweepDuty == 0) {
    pwmSweepStep = 1;
  }
  pwmSweepDuty = static_cast<uint8_t>(static_cast<int16_t>(pwmSweepDuty) + pwmSweepStep);
}

int32_t atomicRead(volatile int32_t* value) {
  noInterrupts();
  const int32_t snapshot = *value;
  interrupts();
  return snapshot;
}

uint32_t atomicRead(volatile uint32_t* value) {
  noInterrupts();
  const uint32_t snapshot = *value;
  interrupts();
  return snapshot;
}

void printStatus() {
  Serial.print(F("STATUS,posM1="));
  Serial.print(atomicRead(&posM1));
  Serial.print(F(",posM2="));
  Serial.print(atomicRead(&posM2));
  Serial.print(F(",aRiseM1="));
  Serial.print(atomicRead(&aRiseM1));
  Serial.print(F(",aRiseM2="));
  Serial.print(atomicRead(&aRiseM2));
  Serial.print(F(",invalidM1="));
  Serial.print(atomicRead(&invalidM1));
  Serial.print(F(",invalidM2="));
  Serial.print(atomicRead(&invalidM2));
  Serial.print(F(",m1Duty="));
  Serial.print(m1Duty);
  Serial.print(F(",m2Duty="));
  Serial.print(m2Duty);
  Serial.print(F(",pwmSweep="));
  Serial.print(pwmSweepEnabled ? F("on") : F("off"));
  Serial.print(F(",dualPwmFull="));
  Serial.println(dualPwmFullEnabled ? F("on") : F("off"));
}

void printPcaProbe() {
  Wire1.beginTransmission(0x40);
  const uint8_t result = Wire1.endTransmission();
  Serial.print(F("PCA9685,address=0x40,i2c_result="));
  Serial.print(result);
  Serial.println(result == 0 ? F(",ack=yes") : F(",ack=no"));
}

// Scope/voltmeter test for the four L298N direction inputs. Enable PWM stays
// LOW through stopAll(), so setting every direction channel HIGH cannot drive
// either motor.
void setPcaChannels1To4(bool high) {
  stopAll();
  const uint16_t value = high ? 4095 : 0;
  pca.setPin(CH_M1_IN1, value);
  pca.setPin(CH_M1_IN2, value);
  pca.setPin(CH_M2_IN1, value);
  pca.setPin(CH_M2_IN2, value);
  Serial.println(high ? F("PCA_CH1_TO_CH4,high; ENA_ENB=low")
                     : F("PCA_CH1_TO_CH4,low; ENA_ENB=low"));
}

void applyPcaWalkChannel(uint8_t channel) {
  pca.setPin(CH_M1_IN1, 0);
  pca.setPin(CH_M1_IN2, 0);
  pca.setPin(CH_M2_IN2, 0);
  pca.setPin(CH_M2_IN1, 0);
  pca.setPin(channel, 4095);
  Serial.print(F("PCA_WALK,ch="));
  Serial.print(channel);
  Serial.println(F(",ENA_ENB=low"));
}

void startPcaWalk() {
  stopAll();
  pcaWalkChannel = 1;
  lastPcaWalkStepMs = millis();
  pcaWalkEnabled = true;
  applyPcaWalkChannel(pcaWalkChannel);
}

void updatePcaWalk() {
  if (!pcaWalkEnabled || millis() - lastPcaWalkStepMs < PCA_WALK_STEP_MS) return;
  lastPcaWalkStepMs = millis();
  pcaWalkChannel++;
  if (pcaWalkChannel > 4) pcaWalkChannel = 1;
  applyPcaWalkChannel(pcaWalkChannel);
}

// Bench test: M1 forward direction while both L298N Enable inputs receive
// 100% PWM. M2 direction inputs remain LOW.
void startDualPwmFullM1Forward() {
  stopAll();
  pca.setPin(CH_M1_IN1, 4095);
  pca.setPin(CH_M1_IN2, 0);
  m1Duty = 255;
  m2Duty = 255;
  dualPwmFullEnabled = true;
  Serial.println(F("DUAL_PWM_FULL,CH1=high,CH2=low,D8_D9=255; send stop to disable"));
}

// Isolates one physical Enable path while M1 direction is held forward. This
// distinguishes D9->ENA from a possible D8/D9 wire swap without driving M2.
void runM1EnableMapTest(bool useD8, uint16_t durationMs) {
  if (durationMs < MIN_PULSE_MS || durationMs > 1000) {
    Serial.println(F("ERROR,pwm_map_m1 duration_ms must be 50..1000"));
    return;
  }
  stopAll();
  waitWithPwm(DIRECTION_SETTLE_MS);
  const int32_t before = atomicRead(&posM1);
  setDirection(CH_M1_IN1, CH_M1_IN2, 1);
  if (useD8) {
    m1Duty = 255;
  } else {
    m2Duty = 255;
  }
  Serial.print(F("PWM_MAP_M1,start,enable="));
  Serial.print(useD8 ? F("D8") : F("D9"));
  Serial.print(F(",duration_ms="));
  Serial.println(durationMs);
  const bool completed = waitWithPwm(durationMs);
  stopAll();
  Serial.print(F("PWM_MAP_M1,"));
  Serial.print(completed ? F("done") : F("aborted"));
  Serial.print(F(",enable="));
  Serial.print(useD8 ? F("D8") : F("D9"));
  Serial.print(F(",encoder_delta="));
  Serial.println(atomicRead(&posM1) - before);
}

bool waitWithPwm(uint16_t durationMs) {
  const uint32_t started = millis();
  while (millis() - started < durationMs) {
    updateMotorPwm();
    if (Serial.available() > 0 && Serial.read() == '!') {
      stopAll();
      Serial.println(F("ABORT,received !"));
      return false;
    }
  }
  return true;
}

void runPulse(const char* motor, char directionChar, uint8_t duty, uint16_t durationMs,
              uint8_t maxDuty = MAX_DIAGNOSTIC_DUTY,
              uint16_t maxDurationMs = MAX_PULSE_MS) {
  const bool isM1 = strcmp(motor, "M1") == 0;
  const bool isM2 = strcmp(motor, "M2") == 0;
  const int8_t direction = directionChar == 'F' ? 1 : (directionChar == 'R' ? -1 : 0);

  if ((!isM1 && !isM2) || direction == 0 ||
      duty < MIN_DIAGNOSTIC_DUTY || duty > maxDuty ||
      durationMs < MIN_PULSE_MS || durationMs > maxDurationMs) {
    Serial.println(F("ERROR,pulse parameters out of allowed range"));
    return;
  }

  stopAll();
  waitWithPwm(DIRECTION_SETTLE_MS);
  const int32_t before = atomicRead(isM1 ? &posM1 : &posM2);

  if (isM1) {
    setDirection(CH_M1_IN1, CH_M1_IN2, direction);
    m1Duty = duty;
  } else {
    setDirection(CH_M2_IN1, CH_M2_IN2, direction);
    m2Duty = duty;
  }

  Serial.print(F("PULSE,start,motor="));
  Serial.print(motor);
  Serial.print(F(",dir="));
  Serial.print(directionChar);
  Serial.print(F(",duty="));
  Serial.print(duty);
  Serial.print(F(",duration_ms="));
  Serial.println(durationMs);

  const bool completed = waitWithPwm(durationMs);
  stopAll();
  const int32_t after = atomicRead(isM1 ? &posM1 : &posM2);

  Serial.print(F("PULSE,"));
  Serial.print(completed ? F("done") : F("aborted"));
  Serial.print(F(",motor="));
  Serial.print(motor);
  Serial.print(F(",encoder_delta="));
  Serial.println(after - before);
}

// Capture encoder data in RAM during one uninterrupted run.  No Serial output
// occurs while the motor is powered, so the reported 500-ms bins are not
// distorted by USB printing.  At duty 255 EN remains continuously HIGH.
void runProfile(const char* motor, char directionChar, unsigned int duty, uint16_t durationMs) {
  const bool isM1 = strcmp(motor, "M1") == 0;
  const bool isM2 = strcmp(motor, "M2") == 0;
  const int8_t direction = directionChar == 'F' ? 1 : (directionChar == 'R' ? -1 : 0);
  if ((!isM1 && !isM2) || direction == 0 || duty < 20 || duty > 255 ||
      durationMs < PROFILE_BIN_MS || durationMs > MAX_PULSE_MS ||
      durationMs % PROFILE_BIN_MS != 0) {
    Serial.println(F("ERROR,profile M1|M2 F|R duty(20..255) duration_ms(500..6000,step=500)"));
    return;
  }

  volatile int32_t* position = isM1 ? &posM1 : &posM2;
  volatile uint32_t* invalid = isM1 ? &invalidM1 : &invalidM2;
  volatile uint32_t* aRise = isM1 ? &aRiseM1 : &aRiseM2;
  uint32_t sampleUs[PROFILE_MAX_BINS + 1];
  int32_t samplePos[PROFILE_MAX_BINS + 1];
  uint32_t sampleInvalid[PROFILE_MAX_BINS + 1];
  uint32_t sampleARise[PROFILE_MAX_BINS + 1];
  const uint8_t binCount = durationMs / PROFILE_BIN_MS;

  stopAll();
  waitWithPwm(DIRECTION_SETTLE_MS);
  setDirection(isM1 ? CH_M1_IN1 : CH_M2_IN1,
               isM1 ? CH_M1_IN2 : CH_M2_IN2, direction);
  noInterrupts();
  samplePos[0] = *position;
  sampleInvalid[0] = *invalid;
  sampleARise[0] = *aRise;
  interrupts();
  sampleUs[0] = micros();
  if (isM1) m1Duty = (uint8_t)duty; else m2Duty = (uint8_t)duty;

  bool aborted = false;
  uint8_t completedBins = 0;
  for (uint8_t bin = 1; bin <= binCount; bin++) {
    const uint32_t dueUs = sampleUs[0] + (uint32_t)bin * PROFILE_BIN_MS * 1000UL;
    while ((int32_t)(micros() - dueUs) < 0) {
      updateMotorPwm();
      if (Serial.available() > 0 && Serial.read() == '!') { aborted = true; break; }
    }
    if (aborted) break;
    sampleUs[bin] = micros();
    noInterrupts();
    samplePos[bin] = *position;
    sampleInvalid[bin] = *invalid;
    sampleARise[bin] = *aRise;
    interrupts();
    completedBins = bin;
  }
  stopAll();

  Serial.print(F("PROFILE,start,motor=")); Serial.print(motor);
  Serial.print(F(",dir=")); Serial.print(directionChar);
  Serial.print(F(",duty=")); Serial.print(duty);
  Serial.print(F(",requested_ms=")); Serial.println(durationMs);
  for (uint8_t bin = 1; bin <= completedBins; bin++) {
    Serial.print(F("PROFILE,bin=")); Serial.print(bin);
    Serial.print(F(",dt_us=")); Serial.print(sampleUs[bin] - sampleUs[bin - 1]);
    Serial.print(F(",ticks=")); Serial.print(samplePos[bin] - samplePos[bin - 1]);
    Serial.print(F(",invalid=")); Serial.print(sampleInvalid[bin] - sampleInvalid[bin - 1]);
    Serial.print(F(",a_rise=")); Serial.println(sampleARise[bin] - sampleARise[bin - 1]);
  }
  Serial.print(F("PROFILE,")); Serial.print(aborted ? F("aborted") : F("done"));
  Serial.print(F(",bins=")); Serial.println(completedBins);
}

void printHelp() {
  Serial.println(F("Commands: help | status | pca_probe | pca_1to4 high|low | pca_walk start|stop | pwm_sweep start|stop | pulse M1|M2 F|R duty(20..255) duration_ms(50..6000) | profile M1|M2 F|R duty(20..255) duration_ms(500..6000)"));
  Serial.println(F("Emergency during a pulse: send !. No motor starts automatically."));
}

void processCommand(char* command) {
  while (*command == ' ') command++;
  if (strcmp(command, "help") == 0) {
    printHelp();
  } else if (strcmp(command, "status") == 0) {
    printStatus();
  } else if (strcmp(command, "pca_probe") == 0) {
    printPcaProbe();
  } else if (strcmp(command, "pca_1to4 high") == 0) {
    setPcaChannels1To4(true);
  } else if (strcmp(command, "pca_1to4 low") == 0) {
    setPcaChannels1To4(false);
  } else if (strcmp(command, "pca_walk start") == 0) {
    startPcaWalk();
  } else if (strcmp(command, "pca_walk stop") == 0) {
    stopAll();
    Serial.println(F("PCA_WALK,stopped; all outputs disabled"));
  } else if (strcmp(command, "dual_pwm_full m1f") == 0) {
    startDualPwmFullM1Forward();
  } else if (strcmp(command, "dual_pwm_full stop") == 0) {
    stopAll();
    Serial.println(F("DUAL_PWM_FULL,stopped; all outputs disabled"));
  } else if (strncmp(command, "pwm_map_m1 ", 11) == 0) {
    char enablePin[3] = {0};
    unsigned long durationMs = 0;
    if (sscanf(command, "pwm_map_m1 %2s %lu", enablePin, &durationMs) == 2 &&
        (strcmp(enablePin, "d8") == 0 || strcmp(enablePin, "d9") == 0)) {
      runM1EnableMapTest(strcmp(enablePin, "d8") == 0, static_cast<uint16_t>(durationMs));
    } else {
      Serial.println(F("ERROR,pwm_map_m1 requires d8|d9 duration_ms"));
    }
  } else if (strcmp(command, "reset") == 0) {
    noInterrupts();
    posM1 = 0;
    posM2 = 0;
    aRiseM1 = 0;
    aRiseM2 = 0;
    invalidM1 = 0;
    invalidM2 = 0;
    interrupts();
    Serial.println(F("OK,encoder counters reset"));
  } else if (strcmp(command, "stop") == 0) {
    stopAll();
    Serial.println(F("OK,all outputs disabled"));
  } else if (strcmp(command, "pwm_sweep start") == 0) {
    startPwmSweep();
  } else if (strcmp(command, "pwm_sweep stop") == 0) {
    stopAll();
    Serial.println(F("PWM_SWEEP,stopped; all outputs disabled"));
  } else {
    char motor[3] = {0};
    char direction = 0;
    unsigned int duty = 0;
    unsigned long durationMs = 0;
    if (sscanf(command, "profile %2s %c %u %lu", motor, &direction, &duty, &durationMs) == 4) {
      if (durationMs > MAX_PULSE_MS) Serial.println(F("ERROR,profile duration too long"));
      else runProfile(motor, direction, duty, (uint16_t)durationMs);
    } else if (sscanf(command, "pulse %2s %c %u %lu", motor, &direction, &duty, &durationMs) == 4) {
      runPulse(motor, direction, static_cast<uint8_t>(duty), static_cast<uint16_t>(durationMs));
    } else if (sscanf(command, "pulse_max %2s %c %lu", motor, &direction, &durationMs) == 3) {
      if (durationMs > 1000) {
        Serial.println(F("ERROR,pulse_max duration_ms must be 50..1000"));
      } else {
        runPulse(motor, direction, 255, static_cast<uint16_t>(durationMs), 255, 1000);
      }
    } else {
      Serial.println(F("ERROR,unknown command; type help"));
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(PIN_M1_A, INPUT);
  pinMode(PIN_M1_B, INPUT);
  pinMode(PIN_M2_A, INPUT);
  pinMode(PIN_M2_B, INPUT);
  stateM1 = (digitalRead(PIN_M1_A) << 1) | digitalRead(PIN_M1_B);
  stateM2 = (digitalRead(PIN_M2_A) << 1) | digitalRead(PIN_M2_B);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_A), isrM1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_B), isrM1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_A), isrM2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_B), isrM2B, CHANGE);

  Wire1.begin();
  pca.begin();
  pca.setPWMFreq(50);
  stopAll();

  Serial.println(F("CAR1_L298N_DIAGNOSTIC ready; all motor outputs disabled"));
  printHelp();
}

void loop() {
  updatePwmSweep();
  updatePcaWalk();
  updateMotorPwm();
  while (Serial.available() > 0) {
    const char byteRead = static_cast<char>(Serial.read());
    if (byteRead == '\r') continue;
    if (byteRead == '\n') {
      lineBuffer[lineLength] = '\0';
      if (lineLength > 0) processCommand(lineBuffer);
      lineLength = 0;
    } else if (lineLength + 1 < sizeof(lineBuffer)) {
      lineBuffer[lineLength++] = byteRead;
    } else {
      lineLength = 0;
      Serial.println(F("ERROR,command too long"));
    }
  }
}
