#include <Arduino.h>

#if defined(__AVR__)
#include <avr/wdt.h>
void disableWatchdogEarly(void) __attribute__((naked)) __attribute__((section(".init3")));
void disableWatchdogEarly(void) {
  MCUSR = 0;
  wdt_disable();
}
#endif

// Standalone four-channel encoder diagnostic.
//
// This mapping includes the A0/A1 correction established by manual wheel
// rotation on 2026-08-23 and is identical to rspV1_arduino v1.4:
//   RF = right front = A2
//   RR = right rear  = A0
//   LF = left front  = A1
//   LR = left rear   = A3
//
// Motor shield mapping is also identical to RSP:
//   channel A / PWMA = physical right side
//   channel B / PWMB = physical left side
//
// Flash this sketch by itself. Reflash rspV1_arduino.ino after the diagnostic.

// =========================
// Motor pins (RSP mapping)
// =========================
#define PIN_MOTOR_PWMA  5
#define PIN_MOTOR_PWMB  6
#define PIN_MOTOR_AIN_1 7
#define PIN_MOTOR_BIN_1 8
#define PIN_MOTOR_STBY  3

// =========================
// Encoder pins (RSP mapping)
// =========================
#define ENC_RF_AO A2
#define ENC_RR_AO A0
#define ENC_LF_AO A1
#define ENC_LR_AO A3

const long SERIAL_BAUD = 115200;
const uint16_t SAMPLE_PERIOD_MS = 5U;
const uint16_t PRINT_PERIOD_MS = 100U;
const uint16_t ARM_DELAY_MS = 350U;
const uint16_t MOTOR_TEST_DURATION_MS = 1500U;
const uint16_t MANUAL_CAPTURE_DURATION_MS = 5000U;
const uint16_t AUTO_PAUSE_MS = 1000U;
const uint32_t MIN_TRANSITION_US = 250U;
const int MIN_SPAN_DEFAULT = 120;
const int MIN_SPAN_LR = 80;
const uint32_t MIN_EXPECTED_TICKS = 2U;
const uint32_t MAX_INACTIVE_TICKS = 1U;

int testPwm = 100;
int currentPwmLeft = 0;
int currentPwmRight = 0;
int phaseCommandLeft = 0;
int phaseCommandRight = 0;

struct EncoderState {
  const char* name;
  uint8_t pin;
  int raw;
  int observedMin;
  int observedMax;
  int lowThreshold;
  int highThreshold;
  int minSpanToArm;
  bool stateHigh;
  bool armed;
  bool valid;
  bool rejectedFastTransition;
  uint32_t toggles;
  uint32_t fallingTicks;
  uint32_t lastTransitionUs;
};

EncoderState encRF;
EncoderState encRR;
EncoderState encLF;
EncoderState encLR;
EncoderState* encoders[] = {&encRF, &encRR, &encLF, &encLR};
const uint8_t NUM_ENCODERS = 4U;

enum TestPhase : uint8_t {
  PHASE_IDLE = 0,
  PHASE_LEFT_ONLY,
  PHASE_RIGHT_ONLY,
  PHASE_MANUAL_CAPTURE,
  PHASE_AUTO_PAUSE
};

TestPhase phase = PHASE_IDLE;
uint32_t phaseStartMs = 0U;
uint32_t phaseDurationMs = 0U;
uint32_t lastSampleMs = 0U;
uint32_t lastPrintMs = 0U;
bool autoSequence = false;

// Explicit prototypes are required here. The Arduino IDE sketch preprocessor
// otherwise generates them before EncoderState/TestPhase and the build fails
// because those custom types have not been declared yet.
static inline int clampInt(int value, int lo, int hi);
const char* phaseName(TestPhase value);
void setMotorRaw(int pwmLeft, int pwmRight);
void stopMotors();
int encoderSpan(const EncoderState& encoder);
void recalcThresholds(EncoderState& encoder);
void resetEncoder(EncoderState& encoder);
void initEncoder(EncoderState& encoder,
                 const char* name,
                 uint8_t pin,
                 int minSpanToArm);
void resetAllEncoders();
void maybeArmEncoder(EncoderState& encoder);
void updateEncoder(EncoderState& encoder, bool allowArming);
void updateEncoders();
void printEncoderCsv(const EncoderState& encoder);
void printLiveData();
void printEncoderSummary(const EncoderState& encoder);
bool encoderActive(const EncoderState& encoder);
bool encoderInactive(const EncoderState& encoder);
void printAutomaticAssessment(TestPhase completedPhase);
void printSummary(TestPhase completedPhase);
void beginPhase(TestPhase nextPhase, uint32_t durationMs);
void abortTest(const __FlashStringHelper* reason);
void completeCurrentPhase();
void updatePhase();
void printHelp();
void handleSerialCommands();
void setup();
void loop();

static inline int clampInt(int value, int lo, int hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

const char* phaseName(TestPhase value) {
  switch (value) {
    case PHASE_IDLE: return "idle";
    case PHASE_LEFT_ONLY: return "left_only";
    case PHASE_RIGHT_ONLY: return "right_only";
    case PHASE_MANUAL_CAPTURE: return "manual_capture";
    case PHASE_AUTO_PAUSE: return "auto_pause";
    default: return "unknown";
  }
}

void setMotorRaw(int pwmLeft, int pwmRight) {
  pwmLeft = clampInt(pwmLeft, -255, 255);
  pwmRight = clampInt(pwmRight, -255, 255);
  currentPwmLeft = pwmLeft;
  currentPwmRight = pwmRight;

  // Exact RSP shield mapping: A=right, B=left.
  digitalWrite(PIN_MOTOR_AIN_1, pwmRight >= 0 ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_BIN_1, pwmLeft >= 0 ? HIGH : LOW);
  analogWrite(PIN_MOTOR_PWMA, abs(pwmRight));
  analogWrite(PIN_MOTOR_PWMB, abs(pwmLeft));
}

void stopMotors() {
  setMotorRaw(0, 0);
}

int encoderSpan(const EncoderState& encoder) {
  return encoder.observedMax - encoder.observedMin;
}

void recalcThresholds(EncoderState& encoder) {
  const int span = encoderSpan(encoder);
  const int center = (encoder.observedMax + encoder.observedMin) / 2;
  const int hysteresis = max(18, span / 10);
  encoder.lowThreshold = center - hysteresis;
  encoder.highThreshold = center + hysteresis;
}

void resetEncoder(EncoderState& encoder) {
  encoder.raw = analogRead(encoder.pin);
  encoder.observedMin = encoder.raw;
  encoder.observedMax = encoder.raw;
  encoder.lowThreshold = encoder.raw;
  encoder.highThreshold = encoder.raw;
  encoder.stateHigh = false;
  encoder.armed = false;
  encoder.valid = true;
  encoder.rejectedFastTransition = false;
  encoder.toggles = 0U;
  encoder.fallingTicks = 0U;
  encoder.lastTransitionUs = 0U;
  recalcThresholds(encoder);
}

void initEncoder(EncoderState& encoder,
                 const char* name,
                 uint8_t pin,
                 int minSpanToArm) {
  encoder.name = name;
  encoder.pin = pin;
  encoder.minSpanToArm = minSpanToArm;
  resetEncoder(encoder);
}

void resetAllEncoders() {
  for (uint8_t i = 0U; i < NUM_ENCODERS; ++i) {
    resetEncoder(*encoders[i]);
  }
}

void maybeArmEncoder(EncoderState& encoder) {
  if (!encoder.valid || encoder.armed) return;
  if (encoderSpan(encoder) < encoder.minSpanToArm) return;

  recalcThresholds(encoder);
  const int center = (encoder.observedMax + encoder.observedMin) / 2;
  encoder.stateHigh = encoder.raw >= center;
  encoder.armed = true;
  encoder.lastTransitionUs = micros();
}

void updateEncoder(EncoderState& encoder, bool allowArming) {
  const int value = analogRead(encoder.pin);
  encoder.raw = value;
  if (value < encoder.observedMin) encoder.observedMin = value;
  if (value > encoder.observedMax) encoder.observedMax = value;

  if (!encoder.armed) {
    recalcThresholds(encoder);
    if (allowArming) maybeArmEncoder(encoder);
    return;
  }

  const bool previousStateHigh = encoder.stateHigh;
  if (!encoder.stateHigh && value >= encoder.highThreshold) {
    encoder.stateHigh = true;
  } else if (encoder.stateHigh && value <= encoder.lowThreshold) {
    encoder.stateHigh = false;
  }

  if (previousStateHigh == encoder.stateHigh) return;

  const uint32_t nowUs = micros();
  if (encoder.lastTransitionUs != 0U &&
      (uint32_t)(nowUs - encoder.lastTransitionUs) < MIN_TRANSITION_US) {
    encoder.stateHigh = previousStateHigh;
    encoder.rejectedFastTransition = true;
    return;
  }

  encoder.lastTransitionUs = nowUs;
  encoder.toggles++;
  if (previousStateHigh && !encoder.stateHigh) encoder.fallingTicks++;
}

void updateEncoders() {
  const bool captureActive = phase == PHASE_LEFT_ONLY ||
                             phase == PHASE_RIGHT_ONLY ||
                             phase == PHASE_MANUAL_CAPTURE;
  const bool allowArming = captureActive &&
      (uint32_t)(millis() - phaseStartMs) >= ARM_DELAY_MS;
  for (uint8_t i = 0U; i < NUM_ENCODERS; ++i) {
    updateEncoder(*encoders[i], allowArming);
  }
}

void printEncoderCsv(const EncoderState& encoder) {
  Serial.print(',');
  Serial.print(encoder.raw);
  Serial.print(',');
  Serial.print(encoderSpan(encoder));
  Serial.print(',');
  Serial.print(encoder.armed ? 1 : 0);
  Serial.print(',');
  Serial.print(encoder.fallingTicks);
  Serial.print(',');
  Serial.print(encoder.toggles);
}

void printLiveData() {
  Serial.print(F("DATA,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(phaseName(phase));
  Serial.print(',');
  Serial.print(currentPwmLeft);
  Serial.print(',');
  Serial.print(currentPwmRight);
  printEncoderCsv(encRF);
  printEncoderCsv(encRR);
  printEncoderCsv(encLF);
  printEncoderCsv(encLR);
  Serial.println();
}

void printEncoderSummary(const EncoderState& encoder) {
  Serial.print(encoder.name);
  Serial.print(F("{raw="));
  Serial.print(encoder.raw);
  Serial.print(F(",min="));
  Serial.print(encoder.observedMin);
  Serial.print(F(",max="));
  Serial.print(encoder.observedMax);
  Serial.print(F(",span="));
  Serial.print(encoderSpan(encoder));
  Serial.print(F(",threshold="));
  Serial.print(encoder.lowThreshold);
  Serial.print(F(".."));
  Serial.print(encoder.highThreshold);
  Serial.print(F(",armed="));
  Serial.print(encoder.armed ? 1 : 0);
  Serial.print(F(",ticks="));
  Serial.print(encoder.fallingTicks);
  Serial.print(F(",toggles="));
  Serial.print(encoder.toggles);
  Serial.print(F(",fastReject="));
  Serial.print(encoder.rejectedFastTransition ? 1 : 0);
  Serial.print('}');
}

bool encoderActive(const EncoderState& encoder) {
  return encoder.armed && encoder.fallingTicks >= MIN_EXPECTED_TICKS;
}

bool encoderInactive(const EncoderState& encoder) {
  return encoder.fallingTicks <= MAX_INACTIVE_TICKS;
}

void printAutomaticAssessment(TestPhase completedPhase) {
  if (completedPhase != PHASE_LEFT_ONLY && completedPhase != PHASE_RIGHT_ONLY) return;

  bool passed = false;
  if (completedPhase == PHASE_LEFT_ONLY) {
    passed = encoderActive(encLF) && encoderActive(encLR) &&
             encoderInactive(encRF) && encoderInactive(encRR);
    Serial.print(F("ASSESS,left_only,"));
    Serial.print(passed ? F("PASS") : F("FAIL"));
    Serial.print(F(",expected_active=LF+LR,expected_inactive=RF+RR"));
  } else {
    passed = encoderActive(encRF) && encoderActive(encRR) &&
             encoderInactive(encLF) && encoderInactive(encLR);
    Serial.print(F("ASSESS,right_only,"));
    Serial.print(passed ? F("PASS") : F("FAIL"));
    Serial.print(F(",expected_active=RF+RR,expected_inactive=LF+LR"));
  }

  if (!passed) {
    Serial.print(F(",check raw spans: a stationary wheel must not arm or count"));
  }
  Serial.println();
}

void printSummary(TestPhase completedPhase) {
  Serial.print(F("SUMMARY,"));
  Serial.print(phaseName(completedPhase));
  Serial.print(F(",pwmL="));
  Serial.print(phaseCommandLeft);
  Serial.print(F(",pwmR="));
  Serial.println(phaseCommandRight);

  printEncoderSummary(encRF);
  Serial.print(' ');
  printEncoderSummary(encRR);
  Serial.println();
  printEncoderSummary(encLF);
  Serial.print(' ');
  printEncoderSummary(encLR);
  Serial.println();
  printAutomaticAssessment(completedPhase);
  Serial.flush();
}

void beginPhase(TestPhase nextPhase, uint32_t durationMs) {
  stopMotors();
  phase = nextPhase;
  phaseDurationMs = durationMs;
  phaseStartMs = millis();
  lastSampleMs = phaseStartMs;
  lastPrintMs = phaseStartMs;
  resetAllEncoders();

  if (phase == PHASE_LEFT_ONLY) {
    setMotorRaw(testPwm, 0);
  } else if (phase == PHASE_RIGHT_ONLY) {
    setMotorRaw(0, testPwm);
  }
  phaseCommandLeft = currentPwmLeft;
  phaseCommandRight = currentPwmRight;

  Serial.print(F("EVENT,start,"));
  Serial.print(phaseName(phase));
  Serial.print(F(",duration_ms="));
  Serial.print(durationMs);
  Serial.print(F(",pwmL="));
  Serial.print(currentPwmLeft);
  Serial.print(F(",pwmR="));
  Serial.println(currentPwmRight);
  Serial.flush();
}

void abortTest(const __FlashStringHelper* reason) {
  stopMotors();
  autoSequence = false;
  phase = PHASE_IDLE;
  Serial.print(F("EVENT,abort,"));
  Serial.println(reason);
  Serial.flush();
}

void completeCurrentPhase() {
  const TestPhase completedPhase = phase;
  stopMotors();
  printSummary(completedPhase);

  if (autoSequence && completedPhase == PHASE_LEFT_ONLY) {
    phase = PHASE_AUTO_PAUSE;
    phaseStartMs = millis();
    phaseDurationMs = AUTO_PAUSE_MS;
    Serial.println(F("EVENT,auto_pause_before_right_only"));
    return;
  }

  if (autoSequence && completedPhase == PHASE_RIGHT_ONLY) {
    Serial.println(F("EVENT,auto_sequence_complete"));
  }
  autoSequence = false;
  phase = PHASE_IDLE;
  Serial.println(F("EVENT,idle,motors_stopped"));
  Serial.flush();
}

void updatePhase() {
  if (phase == PHASE_IDLE) return;
  const uint32_t elapsed = (uint32_t)(millis() - phaseStartMs);

  if (phase == PHASE_AUTO_PAUSE) {
    if (elapsed >= phaseDurationMs) {
      beginPhase(PHASE_RIGHT_ONLY, MOTOR_TEST_DURATION_MS);
    }
    return;
  }

  if (elapsed >= phaseDurationMs) completeCurrentPhase();
}

void printHelp() {
  Serial.println();
  Serial.println(F("# FOUR-CHANNEL ENCODER MAPPING TEST"));
  Serial.println(F("# IMPORTANT: lift and secure the car before motor tests."));
  Serial.println(F("# Mapping: RF=A2 RR=A0 LF=A1 LR=A3; PWMA=right PWMB=left."));
  Serial.println(F("# Commands:"));
  Serial.println(F("#   a = automatic LEFT_ONLY -> stop -> RIGHT_ONLY"));
  Serial.println(F("#   l = LEFT_ONLY for 1.5 s"));
  Serial.println(F("#   r = RIGHT_ONLY for 1.5 s"));
  Serial.println(F("#   m = 5 s manual capture, motors off; rotate ONE wheel by hand"));
  Serial.println(F("#   p = print current raw values and counters"));
  Serial.println(F("#   + / - = change test PWM by 10 (60..200)"));
  Serial.println(F("#   s or SPACE = immediate motor stop"));
  Serial.println(F("#   z = reset all counters while idle"));
  Serial.println(F("#   h = print this help"));
  Serial.println(F("# DATA fields repeat for RF,RR,LF,LR: raw,span,armed,ticks,toggles"));
  Serial.print(F("# Current test PWM: "));
  Serial.println(testPwm);
  Serial.flush();
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();
    switch (command) {
      case 'a':
      case 'A':
        autoSequence = true;
        beginPhase(PHASE_LEFT_ONLY, MOTOR_TEST_DURATION_MS);
        break;
      case 'l':
      case 'L':
        autoSequence = false;
        beginPhase(PHASE_LEFT_ONLY, MOTOR_TEST_DURATION_MS);
        break;
      case 'r':
      case 'R':
        autoSequence = false;
        beginPhase(PHASE_RIGHT_ONLY, MOTOR_TEST_DURATION_MS);
        break;
      case 'm':
      case 'M':
        autoSequence = false;
        beginPhase(PHASE_MANUAL_CAPTURE, MANUAL_CAPTURE_DURATION_MS);
        break;
      case 'p':
      case 'P':
        printLiveData();
        break;
      case '+':
        if (phase == PHASE_IDLE) testPwm = min(200, testPwm + 10);
        Serial.print(F("EVENT,test_pwm,"));
        Serial.println(testPwm);
        break;
      case '-':
        if (phase == PHASE_IDLE) testPwm = max(60, testPwm - 10);
        Serial.print(F("EVENT,test_pwm,"));
        Serial.println(testPwm);
        break;
      case 's':
      case 'S':
      case ' ':
        abortTest(F("operator_stop"));
        break;
      case 'z':
      case 'Z':
        if (phase == PHASE_IDLE) {
          resetAllEncoders();
          Serial.println(F("EVENT,counters_reset"));
        }
        break;
      case 'h':
      case 'H':
        printHelp();
        break;
      case '\r':
      case '\n':
        break;
      default:
        Serial.print(F("EVENT,unknown_command,"));
        Serial.println(command);
        break;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_MOTOR_PWMA, OUTPUT);
  pinMode(PIN_MOTOR_PWMB, OUTPUT);
  pinMode(PIN_MOTOR_AIN_1, OUTPUT);
  pinMode(PIN_MOTOR_BIN_1, OUTPUT);
  pinMode(PIN_MOTOR_STBY, OUTPUT);
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  stopMotors();

  pinMode(ENC_RF_AO, INPUT);
  pinMode(ENC_RR_AO, INPUT);
  pinMode(ENC_LF_AO, INPUT);
  pinMode(ENC_LR_AO, INPUT);

  initEncoder(encRF, "RF(A2)", ENC_RF_AO, MIN_SPAN_DEFAULT);
  initEncoder(encRR, "RR(A0)", ENC_RR_AO, MIN_SPAN_DEFAULT);
  initEncoder(encLF, "LF(A1)", ENC_LF_AO, MIN_SPAN_DEFAULT);
  initEncoder(encLR, "LR(A3)", ENC_LR_AO, MIN_SPAN_LR);

  phase = PHASE_IDLE;
  phaseStartMs = millis();
  lastSampleMs = phaseStartMs;
  lastPrintMs = phaseStartMs;

  delay(250);
  printHelp();
  Serial.println(F("EVENT,ready,motors_stopped"));
}

void loop() {
  handleSerialCommands();

  const uint32_t now = millis();
  if ((uint32_t)(now - lastSampleMs) >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    updateEncoders();
  }
  if (phase != PHASE_IDLE && phase != PHASE_AUTO_PAUSE &&
      (uint32_t)(now - lastPrintMs) >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    printLiveData();
  }

  updatePhase();
}
