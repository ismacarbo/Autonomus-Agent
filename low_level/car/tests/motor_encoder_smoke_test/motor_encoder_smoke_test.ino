#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// Standalone motor + encoder smoke test.
// Flash this sketch by itself, not together with rspV1.cpp.

// =========================
// Motor pins
// =========================
#define PIN_Motor_PWMA  5
#define PIN_Motor_PWMB  6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY  3

// =========================
// Encoder AO mapping
// RF(A2) RR(A1) LF(A0) LR(A3)
// =========================
#define ENC_RF_AO A2
#define ENC_RR_AO A1
#define ENC_LF_AO A0
#define ENC_LR_AO A3

const long SERIAL_BAUD = 115200;
const unsigned long SAMPLE_PERIOD_MS = 5;
const unsigned long PRINT_PERIOD_MS = 250;
const unsigned long STARTUP_DELAY_MS = 1800;
const unsigned long ENCODER_ARM_DELAY_MS = 350;
const float IMU_GYRO_SCALE = 131.0f;
const uint16_t IMU_CALIBRATION_SAMPLES = 500;
const int TEST_PWM = 140;
const int ENCODER_MIN_SPAN_DEFAULT = 120;
const int ENCODER_MIN_SPAN_LR = 80;

MPU6050 mpu;

struct EncoderState {
  const char* name;
  uint8_t pin;
  int raw;
  int observedMin;
  int observedMax;
  int lowThreshold;
  int highThreshold;
  bool stateHigh;
  bool armed;
  bool valid;
  unsigned long ticks;
  unsigned long changes;
  int minSpanToArm;
};

EncoderState encRF;
EncoderState encRR;
EncoderState encLF;
EncoderState encLR;

EncoderState* encoders[] = {&encRF, &encRR, &encLF, &encLR};
const int NUM_ENCODERS = 4;

enum TestPhase {
  PHASE_STOP_1,
  PHASE_FORWARD,
  PHASE_STOP_2,
  PHASE_BACKWARD,
  PHASE_STOP_3,
  PHASE_TURN_LEFT,
  PHASE_STOP_4,
  PHASE_TURN_RIGHT,
  PHASE_STOP_5
};

TestPhase phase = PHASE_STOP_1;
unsigned long phaseStartMs = 0;
unsigned long lastSampleMs = 0;
unsigned long lastPrintMs = 0;
bool finalSummaryPrinted = false;
int currentPwmLeft = 0;
int currentPwmRight = 0;
bool imuReady = false;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
int16_t axRaw = 0;
int16_t ayRaw = 0;
int16_t azRaw = 0;
int16_t gxRaw = 0;
int16_t gyRaw = 0;
int16_t gzRaw = 0;
float gyroXdps = 0.0f;
float gyroYdps = 0.0f;
float gyroZdps = 0.0f;
float yawDeg = 0.0f;
unsigned long lastImuUpdateMs = 0;

void printStartupBanner();
void printEncoderSetup();
void printEncoderCompact(const EncoderState &e);
void printEncoderStatus();
void printIMUStatus();
void printHelp();

bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void calibrateGyroBias(uint16_t samples = IMU_CALIBRATION_SAMPLES) {
  long sx = 0L;
  long sy = 0L;
  long sz = 0L;

  if (!imuReady) return;

  Serial.println(F("[IMU] gyro calibration in progress, keep the robot still"));
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sx += gx;
    sy += gy;
    sz += gz;
    delay(3);
  }

  gyroBiasX = (float)sx / (float)samples;
  gyroBiasY = (float)sy / (float)samples;
  gyroBiasZ = (float)sz / (float)samples;

  Serial.print(F("[IMU] biasX="));
  Serial.print(gyroBiasX, 2);
  Serial.print(F(" biasY="));
  Serial.print(gyroBiasY, 2);
  Serial.print(F(" biasZ="));
  Serial.println(gyroBiasZ, 2);
  Serial.flush();
}

void resetYaw() {
  yawDeg = 0.0f;
  lastImuUpdateMs = millis();
  Serial.println(F("[IMU] yaw reset"));
  Serial.flush();
}

void updateIMU() {
  if (!imuReady) return;

  const unsigned long now = millis();
  const float dt = (float)(now - lastImuUpdateMs) / 1000.0f;
  if (dt <= 0.0f) {
    lastImuUpdateMs = now;
    return;
  }
  lastImuUpdateMs = now;

  mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

  gyroXdps = ((float)gxRaw - gyroBiasX) / IMU_GYRO_SCALE;
  gyroYdps = ((float)gyRaw - gyroBiasY) / IMU_GYRO_SCALE;
  gyroZdps = ((float)gzRaw - gyroBiasZ) / IMU_GYRO_SCALE;

  yawDeg += gyroZdps * dt;
}

void printIMUStatus() {
  if (!imuReady) {
    Serial.println(F("[IMU] ready=0 addr=0x68"));
    return;
  }

  Serial.print(F("[IMU] yawDeg="));
  Serial.print(yawDeg, 2);
  Serial.print(F(" dps="));
  Serial.print(gyroXdps, 2);
  Serial.print(F("/"));
  Serial.print(gyroYdps, 2);
  Serial.print(F("/"));
  Serial.print(gyroZdps, 2);
  Serial.print(F(" acc="));
  Serial.print(axRaw);
  Serial.print(F("/"));
  Serial.print(ayRaw);
  Serial.print(F("/"));
  Serial.print(azRaw);
  Serial.print(F(" gyroRaw="));
  Serial.print(gxRaw);
  Serial.print(F("/"));
  Serial.print(gyRaw);
  Serial.print(F("/"));
  Serial.println(gzRaw);
}

void printHelp() {
  Serial.println(F("[BOOT] serial commands: z=reset yaw, c=recalibrate gyro, p=print now, h=help"));
  Serial.flush();
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    switch (c) {
      case 'z':
        resetYaw();
        break;
      case 'c':
        calibrateGyroBias();
        resetYaw();
        break;
      case 'p':
        printEncoderStatus();
        break;
      case 'h':
        printHelp();
        break;
      case '\r':
      case '\n':
      case ' ':
        break;
      default:
        Serial.print(F("[BOOT] unknown command: "));
        Serial.println(c);
        Serial.flush();
        break;
    }
  }
}

int safeAnalogReadIfValid(uint8_t pin, bool &valid) {
  valid = true;
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  if (!(pin == A0 || pin == A1 || pin == A2 || pin == A3 || pin == A4 || pin == A5)) {
    valid = false;
    return -1;
  }
#endif
  return analogRead(pin);
}

void recalcThresholds(EncoderState &e) {
  const int span = e.observedMax - e.observedMin;
  const int center = (e.observedMax + e.observedMin) / 2;
  const int hysteresis = max(18, span / 10);
  e.lowThreshold = center - hysteresis;
  e.highThreshold = center + hysteresis;
}

void resetEncoderState(EncoderState &e) {
  bool valid = false;
  const int value = safeAnalogReadIfValid(e.pin, valid);
  e.valid = valid;
  e.raw = value;
  e.observedMin = valid ? value : 0;
  e.observedMax = valid ? value : 0;
  e.lowThreshold = value;
  e.highThreshold = value;
  e.stateHigh = false;
  e.armed = false;
  e.ticks = 0;
  e.changes = 0;
  recalcThresholds(e);
}

void initEncoder(EncoderState &e, const char* name, uint8_t pin, int minSpanToArm) {
  e.name = name;
  e.pin = pin;
  e.minSpanToArm = minSpanToArm;
  resetEncoderState(e);
}

void resetAllEncoders() {
  for (int i = 0; i < NUM_ENCODERS; ++i) {
    resetEncoderState(*encoders[i]);
  }
}

void setMotorRaw(int pwmLeft, int pwmRight) {
  pwmLeft = constrain(pwmLeft, -255, 255);
  pwmRight = constrain(pwmRight, -255, 255);

  currentPwmLeft = pwmLeft;
  currentPwmRight = pwmRight;

  // A = right side, B = left side
  digitalWrite(PIN_Motor_AIN_1, pwmRight >= 0 ? HIGH : LOW);
  digitalWrite(PIN_Motor_BIN_1, pwmLeft >= 0 ? HIGH : LOW);

  analogWrite(PIN_Motor_PWMA, abs(pwmRight));
  analogWrite(PIN_Motor_PWMB, abs(pwmLeft));
}

void stopMotors() {
  setMotorRaw(0, 0);
}

int encoderSpan(const EncoderState &e) {
  return e.observedMax - e.observedMin;
}

void maybeArmEncoder(EncoderState &e) {
  if (!e.valid || e.armed) return;
  if (encoderSpan(e) < e.minSpanToArm) return;

  recalcThresholds(e);
  const int center = (e.observedMax + e.observedMin) / 2;
  e.stateHigh = (e.raw >= center);
  e.armed = true;
}

void updateEncoder(EncoderState &e, bool allowArming) {
  bool valid = false;
  const int value = safeAnalogReadIfValid(e.pin, valid);
  e.valid = valid;
  e.raw = value;
  if (!valid) return;

  if (value < e.observedMin) e.observedMin = value;
  if (value > e.observedMax) e.observedMax = value;

  if (!e.armed) {
    recalcThresholds(e);
    if (allowArming) maybeArmEncoder(e);
    return;
  }

  const bool prevStateHigh = e.stateHigh;
  if (!e.stateHigh && value >= e.highThreshold) e.stateHigh = true;
  else if (e.stateHigh && value <= e.lowThreshold) e.stateHigh = false;

  if (prevStateHigh != e.stateHigh) {
    e.changes++;
    if (prevStateHigh && !e.stateHigh) e.ticks++;
  }
}

void updateEncoders() {
  const bool moving = (abs(currentPwmLeft) > 0) || (abs(currentPwmRight) > 0);
  const bool allowArming = moving && ((millis() - phaseStartMs) >= ENCODER_ARM_DELAY_MS);

  for (int i = 0; i < NUM_ENCODERS; ++i) {
    updateEncoder(*encoders[i], allowArming);
  }
}

const char* phaseName(TestPhase p) {
  switch (p) {
    case PHASE_STOP_1: return "STOP_1";
    case PHASE_FORWARD: return "FORWARD";
    case PHASE_STOP_2: return "STOP_2";
    case PHASE_BACKWARD: return "BACKWARD";
    case PHASE_STOP_3: return "STOP_3";
    case PHASE_TURN_LEFT: return "TURN_LEFT";
    case PHASE_STOP_4: return "STOP_4";
    case PHASE_TURN_RIGHT: return "TURN_RIGHT";
    case PHASE_STOP_5: return "STOP_5";
    default: return "UNKNOWN";
  }
}

unsigned long totalLeftTicks() {
  return encLF.ticks + encLR.ticks;
}

unsigned long totalRightTicks() {
  return encRF.ticks + encRR.ticks;
}

void printPhaseSummary() {
  Serial.print(F("[SUMMARY] phase="));
  Serial.print(phaseName(phase));
  Serial.print(F(" pwmL="));
  Serial.print(currentPwmLeft);
  Serial.print(F(" pwmR="));
  Serial.print(currentPwmRight);
  Serial.print(F(" leftTicks="));
  Serial.print(totalLeftTicks());
  Serial.print(F(" rightTicks="));
  Serial.print(totalRightTicks());
  Serial.print(F(" | RF="));
  Serial.print(encRF.ticks);
  Serial.print(F(" RR="));
  Serial.print(encRR.ticks);
  Serial.print(F(" LF="));
  Serial.print(encLF.ticks);
  Serial.print(F(" LR="));
  Serial.println(encLR.ticks);

  Serial.print(F("          span RF/RR/LF/LR="));
  Serial.print(encoderSpan(encRF));
  Serial.print(F("/"));
  Serial.print(encoderSpan(encRR));
  Serial.print(F("/"));
  Serial.print(encoderSpan(encLF));
  Serial.print(F("/"));
  Serial.print(encoderSpan(encLR));
  Serial.print(F(" arm="));
  Serial.print(encRF.armed ? 1 : 0);
  Serial.print(F("/"));
  Serial.print(encRR.armed ? 1 : 0);
  Serial.print(F("/"));
  Serial.print(encLF.armed ? 1 : 0);
  Serial.print(F("/"));
  Serial.println(encLR.armed ? 1 : 0);
  Serial.print(F("          imuReady="));
  Serial.print(imuReady ? 1 : 0);
  Serial.print(F(" yawDeg="));
  Serial.print(yawDeg, 2);
  Serial.print(F(" gyroZ_dps="));
  Serial.println(gyroZdps, 2);
  Serial.flush();
}

void printEncoderPair(const EncoderState &a, const EncoderState &b) {
  Serial.print(a.name);
  Serial.print(F(": raw="));
  Serial.print(a.raw);
  Serial.print(F(" span="));
  Serial.print(encoderSpan(a));
  Serial.print(F(" arm="));
  Serial.print(a.armed ? 1 : 0);
  Serial.print(F(" ok="));
  Serial.print(a.valid ? 1 : 0);
  Serial.print(F(" tick="));
  Serial.print(a.ticks);
  Serial.print(F(" chg="));
  Serial.print(a.changes);

  Serial.print(F(" || "));

  Serial.print(b.name);
  Serial.print(F(": raw="));
  Serial.print(b.raw);
  Serial.print(F(" span="));
  Serial.print(encoderSpan(b));
  Serial.print(F(" arm="));
  Serial.print(b.armed ? 1 : 0);
  Serial.print(F(" ok="));
  Serial.print(b.valid ? 1 : 0);
  Serial.print(F(" tick="));
  Serial.print(b.ticks);
  Serial.print(F(" chg="));
  Serial.println(b.changes);
}

void printEncoderCompact(const EncoderState &e) {
  Serial.print(e.name);
  Serial.print(F("[raw="));
  Serial.print(e.raw);
  Serial.print(F(" span="));
  Serial.print(encoderSpan(e));
  Serial.print(F(" thr="));
  Serial.print(e.lowThreshold);
  Serial.print(F(".."));
  Serial.print(e.highThreshold);
  Serial.print(F(" arm="));
  Serial.print(e.armed ? 1 : 0);
  Serial.print(F(" ok="));
  Serial.print(e.valid ? 1 : 0);
  Serial.print(F(" tick="));
  Serial.print(e.ticks);
  Serial.print(F("]"));
}

void printEncoderStatus() {
  Serial.print(F("[TEST] phase="));
  Serial.print(phaseName(phase));
  Serial.print(F(" t="));
  Serial.print(millis() - phaseStartMs);
  Serial.print(F("ms pwmL="));
  Serial.print(currentPwmLeft);
  Serial.print(F(" pwmR="));
  Serial.print(currentPwmRight);
  Serial.print(F(" leftTicks="));
  Serial.print(totalLeftTicks());
  Serial.print(F(" rightTicks="));
  Serial.println(totalRightTicks());

  Serial.print(F("       "));
  printEncoderCompact(encRF);
  Serial.print(F(" "));
  printEncoderCompact(encRR);
  Serial.print(F(" "));
  printEncoderCompact(encLF);
  Serial.print(F(" "));
  printEncoderCompact(encLR);
  Serial.println();

  printEncoderPair(encRF, encRR);
  printEncoderPair(encLF, encLR);
  printIMUStatus();
}

void printStartupBanner() {
  Serial.println();
  Serial.println();
  Serial.println(F("############################################################"));
  Serial.println(F("# MOTOR + ENCODER SMOKE TEST"));
  Serial.println(F("# Open Serial Monitor at 115200 baud"));
  Serial.println(F("# Expected sequence: STOP -> FWD -> STOP -> BACK -> STOP -> LEFT -> STOP -> RIGHT -> STOP"));
  Serial.println(F("############################################################"));
  Serial.print(F("[BOOT] baud="));
  Serial.print(SERIAL_BAUD);
  Serial.print(F(" sampleMs="));
  Serial.print(SAMPLE_PERIOD_MS);
  Serial.print(F(" printMs="));
  Serial.print(PRINT_PERIOD_MS);
  Serial.print(F(" testPwm="));
  Serial.println(TEST_PWM);
  Serial.print(F("[BOOT] motor pins PWMA/PWMB/AIN1/BIN1/STBY = "));
  Serial.print(PIN_Motor_PWMA);
  Serial.print(F("/"));
  Serial.print(PIN_Motor_PWMB);
  Serial.print(F("/"));
  Serial.print(PIN_Motor_AIN_1);
  Serial.print(F("/"));
  Serial.print(PIN_Motor_BIN_1);
  Serial.print(F("/"));
  Serial.println(PIN_Motor_STBY);
  Serial.print(F("[BOOT] encoder pins RF/RR/LF/LR = "));
  Serial.print(ENC_RF_AO);
  Serial.print(F("/"));
  Serial.print(ENC_RR_AO);
  Serial.print(F("/"));
  Serial.print(ENC_LF_AO);
  Serial.print(F("/"));
  Serial.println(ENC_LR_AO);
  Serial.println(F("[BOOT] IMU expected on I2C address 0x68"));
  Serial.flush();
}

void printEncoderSetup() {
  Serial.println(F("[BOOT] initial encoder snapshot"));
  Serial.print(F("       "));
  printEncoderCompact(encRF);
  Serial.print(F(" "));
  printEncoderCompact(encRR);
  Serial.print(F(" "));
  printEncoderCompact(encLF);
  Serial.print(F(" "));
  printEncoderCompact(encLR);
  Serial.println();
  Serial.print(F("[BOOT] minSpan RF/RR/LF/LR = "));
  Serial.print(encRF.minSpanToArm);
  Serial.print(F("/"));
  Serial.print(encRR.minSpanToArm);
  Serial.print(F("/"));
  Serial.print(encLF.minSpanToArm);
  Serial.print(F("/"));
  Serial.println(encLR.minSpanToArm);
  Serial.flush();
}

void enterPhase(TestPhase newPhase) {
  phase = newPhase;
  phaseStartMs = millis();
  resetAllEncoders();

  switch (phase) {
    case PHASE_STOP_1:
    case PHASE_STOP_2:
    case PHASE_STOP_3:
    case PHASE_STOP_4:
    case PHASE_STOP_5:
      stopMotors();
      break;
    case PHASE_FORWARD:
      setMotorRaw(TEST_PWM, TEST_PWM);
      break;
    case PHASE_BACKWARD:
      setMotorRaw(-TEST_PWM, -TEST_PWM);
      break;
    case PHASE_TURN_LEFT:
      setMotorRaw(-TEST_PWM, TEST_PWM);
      break;
    case PHASE_TURN_RIGHT:
      setMotorRaw(TEST_PWM, -TEST_PWM);
      break;
  }

  Serial.println();
  Serial.print(F("=== PHASE "));
  Serial.print(phaseName(phase));
  Serial.print(F(" cmdL="));
  Serial.print(currentPwmLeft);
  Serial.print(F(" cmdR="));
  Serial.print(currentPwmRight);
  Serial.println(F(" ==="));
  Serial.flush();
}

void handlePhaseSequencer() {
  const unsigned long elapsed = millis() - phaseStartMs;

  switch (phase) {
    case PHASE_STOP_1:
      if (elapsed >= 2000) {
        printPhaseSummary();
        enterPhase(PHASE_FORWARD);
      }
      break;
    case PHASE_FORWARD:
      if (elapsed >= 4000) {
        printPhaseSummary();
        enterPhase(PHASE_STOP_2);
      }
      break;
    case PHASE_STOP_2:
      if (elapsed >= 2000) {
        printPhaseSummary();
        enterPhase(PHASE_BACKWARD);
      }
      break;
    case PHASE_BACKWARD:
      if (elapsed >= 4000) {
        printPhaseSummary();
        enterPhase(PHASE_STOP_3);
      }
      break;
    case PHASE_STOP_3:
      if (elapsed >= 2000) {
        printPhaseSummary();
        enterPhase(PHASE_TURN_LEFT);
      }
      break;
    case PHASE_TURN_LEFT:
      if (elapsed >= 3500) {
        printPhaseSummary();
        enterPhase(PHASE_STOP_4);
      }
      break;
    case PHASE_STOP_4:
      if (elapsed >= 2000) {
        printPhaseSummary();
        enterPhase(PHASE_TURN_RIGHT);
      }
      break;
    case PHASE_TURN_RIGHT:
      if (elapsed >= 3500) {
        printPhaseSummary();
        enterPhase(PHASE_STOP_5);
      }
      break;
    case PHASE_STOP_5:
      stopMotors();
      if (!finalSummaryPrinted && elapsed >= 2000) {
        printPhaseSummary();
        Serial.println(F("[TEST] sequence complete. Send this serial log back for analysis."));
        finalSummaryPrinted = true;
      }
      break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(25000U, true);
#endif

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);

  pinMode(ENC_RF_AO, INPUT);
  pinMode(ENC_RR_AO, INPUT);
  pinMode(ENC_LF_AO, INPUT);
  pinMode(ENC_LR_AO, INPUT);

  initEncoder(encRF, "RF", ENC_RF_AO, ENCODER_MIN_SPAN_DEFAULT);
  initEncoder(encRR, "RR", ENC_RR_AO, ENCODER_MIN_SPAN_DEFAULT);
  initEncoder(encLF, "LF", ENC_LF_AO, ENCODER_MIN_SPAN_DEFAULT);
  initEncoder(encLR, "LR", ENC_LR_AO, ENCODER_MIN_SPAN_LR);

  digitalWrite(PIN_Motor_STBY, HIGH);
  stopMotors();

  delay(STARTUP_DELAY_MS);
  printStartupBanner();
  Serial.println(F("[BOOT] ok=1 means the analog pin is readable."));
  Serial.println(F("[BOOT] arm=1 means the encoder span is large enough to trust tick counting."));
  Serial.println(F("[BOOT] If a wheel moves but arm stays 0 or tick stays 0, that channel is the suspect."));
  if (!i2cDevicePresent(0x68)) {
    Serial.println(F("[BOOT] IMU not found on 0x68. Continuing without IMU."));
    imuReady = false;
  } else {
    Serial.println(F("[BOOT] IMU detected on 0x68"));
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println(F("[BOOT] IMU connection test failed. Continuing without IMU."));
      imuReady = false;
    } else {
      Serial.println(F("[BOOT] IMU connection OK"));
      imuReady = true;
      delay(1000);
      calibrateGyroBias();
      resetYaw();
    }
  }
  printEncoderSetup();
  printHelp();

  enterPhase(PHASE_STOP_1);
}

void loop() {
  const unsigned long now = millis();
  handleSerialCommands();

  handlePhaseSequencer();

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    updateIMU();
    updateEncoders();
  }

  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    printEncoderStatus();
  }
}
