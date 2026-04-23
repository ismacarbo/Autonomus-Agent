#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <stdint.h>

#if defined(__AVR__)
#include <avr/wdt.h>
void disableWatchdogEarly(void) __attribute__((naked)) __attribute__((section(".init3")));
void disableWatchdogEarly(void) {
  MCUSR = 0;
  wdt_disable();
}
#endif

// Same shield mapping as rspV1_arduino.ino
#define PIN_Motor_PWMA  5
#define PIN_Motor_PWMB  6
#define PIN_Motor_AIN_1 7
#define PIN_Motor_BIN_1 8
#define PIN_Motor_STBY  3

const long SERIAL_BAUD = 115200;
const bool BOOT_DIAG_ASCII = true;

const float GYRO_DPS_TO_RAD = 0.017453292519943295f;
const int LEFT_SIGN = 1;
const int RIGHT_SIGN = 1;

// Conservative defaults so the robot rotates in place without being too aggressive.
const int16_t TEST_SPIN_PWM = 78;
const uint32_t BOOT_SETTLE_MS = 3000U;
const uint32_t STILL_BASELINE_MS = 2500U;
const uint32_t SPIN_DURATION_MS = 1800U;
const uint32_t BETWEEN_SPINS_MS = 2200U;
const uint32_t FINAL_STILL_MS = 3000U;
const uint32_t TELEMETRY_PERIOD_MS = 50U;
const uint16_t GYRO_BIAS_SAMPLES = 220U;

MPU6050 mpu;

bool imu_ready = false;
bool test_running = false;
float gyro_bias_z = 0.0f;
float yaw_wrapped_rad = 0.0f;
float yaw_unwrapped_rad = 0.0f;
float yaw_rate_dps = 0.0f;
float yaw_rate_rad_s = 0.0f;
int16_t acc_x_raw = 0;
int16_t acc_y_raw = 0;
int16_t acc_z_raw = 0;
int16_t gyro_z_raw = 0;
int16_t cmd_pwm_l = 0;
int16_t cmd_pwm_r = 0;
uint32_t last_imu_ms = 0U;
uint32_t last_print_ms = 0U;

enum TestPhase : uint8_t {
  PHASE_BOOT_WAIT = 0,
  PHASE_STILL_BEFORE,
  PHASE_SPIN_LNEG_RPOS,
  PHASE_STILL_MID,
  PHASE_SPIN_LPOS_RNEG,
  PHASE_STILL_AFTER,
  PHASE_DONE
};

TestPhase current_phase = PHASE_BOOT_WAIT;
uint32_t phase_start_ms = 0U;

struct PhaseStats {
  uint32_t started_ms;
  float yaw_start_rad;
  float sum_abs_rate_dps;
  float peak_abs_rate_dps;
  uint32_t samples;
};

PhaseStats phase_stats = {0U, 0.0f, 0.0f, 0.0f, 0U};

static inline int clampi(int value, int lo, int hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static inline float wrap_pi(float angle) {
  while (angle > 3.14159265f) angle -= 6.28318531f;
  while (angle < -3.14159265f) angle += 6.28318531f;
  return angle;
}

void boot_log(const __FlashStringHelper* msg) {
  if (!BOOT_DIAG_ASCII) return;
  Serial.print(F("[BOOT] "));
  Serial.println(msg);
  Serial.flush();
}

bool i2c_device_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0U;
}

void set_motor_hw(int16_t pwm_l, int16_t pwm_r) {
  int hw_l = clampi((int)LEFT_SIGN * (int)pwm_l, -255, 255);
  int hw_r = clampi((int)RIGHT_SIGN * (int)pwm_r, -255, 255);

  digitalWrite(PIN_Motor_AIN_1, hw_r >= 0 ? HIGH : LOW);
  digitalWrite(PIN_Motor_BIN_1, hw_l >= 0 ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMA, abs(hw_r));
  analogWrite(PIN_Motor_PWMB, abs(hw_l));

  cmd_pwm_l = pwm_l;
  cmd_pwm_r = pwm_r;
}

void hard_stop_motors() {
  set_motor_hw(0, 0);
}

void zero_yaw_state() {
  yaw_wrapped_rad = 0.0f;
  yaw_unwrapped_rad = 0.0f;
  yaw_rate_dps = 0.0f;
  yaw_rate_rad_s = 0.0f;
}

void calibrate_gyro_bias(uint16_t samples) {
  long sum = 0L;
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(4);
  }
  gyro_bias_z = (float)sum / (float)samples;
}

void update_imu() {
  if (!imu_ready) return;

  uint32_t now = millis();
  float dt = (float)(now - last_imu_ms) / 1000.0f;
  if (dt <= 0.0f) {
    last_imu_ms = now;
    return;
  }
  last_imu_ms = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  acc_x_raw = ax;
  acc_y_raw = ay;
  acc_z_raw = az;
  gyro_z_raw = gz;

  yaw_rate_dps = ((float)gz - gyro_bias_z) / 131.0f;
  yaw_rate_rad_s = yaw_rate_dps * GYRO_DPS_TO_RAD;
  yaw_unwrapped_rad += yaw_rate_rad_s * dt;
  yaw_wrapped_rad = wrap_pi(yaw_unwrapped_rad);

  phase_stats.sum_abs_rate_dps += fabsf(yaw_rate_dps);
  float abs_rate = fabsf(yaw_rate_dps);
  if (abs_rate > phase_stats.peak_abs_rate_dps) phase_stats.peak_abs_rate_dps = abs_rate;
  phase_stats.samples++;
}

const char* phase_name(uint8_t phase) {
  switch (phase) {
    case PHASE_BOOT_WAIT: return "boot_wait";
    case PHASE_STILL_BEFORE: return "still_before";
    case PHASE_SPIN_LNEG_RPOS: return "spin_lneg_rpos";
    case PHASE_STILL_MID: return "still_mid";
    case PHASE_SPIN_LPOS_RNEG: return "spin_lpos_rneg";
    case PHASE_STILL_AFTER: return "still_after";
    case PHASE_DONE: return "done";
    default: return "unknown";
  }
}

void reset_phase_stats() {
  phase_stats.started_ms = millis();
  phase_stats.yaw_start_rad = yaw_unwrapped_rad;
  phase_stats.sum_abs_rate_dps = 0.0f;
  phase_stats.peak_abs_rate_dps = 0.0f;
  phase_stats.samples = 0U;
}

void print_phase_summary(uint8_t phase, uint32_t ended_ms) {
  float yaw_delta = yaw_unwrapped_rad - phase_stats.yaw_start_rad;
  float mean_abs_rate = 0.0f;
  if (phase_stats.samples > 0U) {
    mean_abs_rate = phase_stats.sum_abs_rate_dps / (float)phase_stats.samples;
  }

  Serial.print(F("SUMMARY,"));
  Serial.print(phase_name(phase));
  Serial.print(F(","));
  Serial.print((unsigned long)(ended_ms - phase_stats.started_ms));
  Serial.print(F(","));
  Serial.print(yaw_delta, 6);
  Serial.print(F(","));
  Serial.print(mean_abs_rate, 4);
  Serial.print(F(","));
  Serial.println(phase_stats.peak_abs_rate_dps, 4);
}

void print_help() {
  Serial.println(F("# Commands:"));
  Serial.println(F("#   t -> rerun full auto test"));
  Serial.println(F("#   z -> recalibrate gyro bias and zero yaw (robot fermo)"));
  Serial.println(F("#   s -> hard stop motors"));
  Serial.println(F("#   h -> print this help"));
}

void print_csv_header() {
  Serial.println(F("# imu_spin_validation"));
  Serial.println(F("# Place the robot on the floor with clear space for in-place spin."));
  Serial.println(F("# Columns: kind,ms,phase,pwm_l,pwm_r,gyro_z_raw,gyro_bias_raw,yaw_rate_dps,yaw_rate_rad_s,yaw_wrapped_rad,yaw_unwrapped_rad,acc_x_raw,acc_y_raw,acc_z_raw"));
  Serial.println(F("# Summary columns: SUMMARY,phase,duration_ms,yaw_delta_rad,mean_abs_rate_dps,peak_abs_rate_dps"));
}

void enter_phase(uint8_t next_phase) {
  uint32_t now = millis();

  if (test_running || current_phase != PHASE_BOOT_WAIT) {
    print_phase_summary(current_phase, now);
  }

  current_phase = next_phase;
  phase_start_ms = now;
  reset_phase_stats();

  switch (current_phase) {
    case PHASE_BOOT_WAIT:
    case PHASE_STILL_BEFORE:
    case PHASE_STILL_MID:
    case PHASE_STILL_AFTER:
    case PHASE_DONE:
      hard_stop_motors();
      break;
    case PHASE_SPIN_LNEG_RPOS:
      set_motor_hw(-TEST_SPIN_PWM, TEST_SPIN_PWM);
      break;
    case PHASE_SPIN_LPOS_RNEG:
      set_motor_hw(TEST_SPIN_PWM, -TEST_SPIN_PWM);
      break;
    default:
      hard_stop_motors();
      break;
  }

  Serial.print(F("EVENT,"));
  Serial.print(now);
  Serial.print(F(","));
  Serial.print(phase_name(current_phase));
  Serial.print(F(","));
  Serial.print(cmd_pwm_l);
  Serial.print(F(","));
  Serial.println(cmd_pwm_r);
}

void start_auto_test() {
  zero_yaw_state();
  test_running = true;
  enter_phase(PHASE_STILL_BEFORE);
}

void recalibrate_and_zero() {
  hard_stop_motors();
  test_running = false;
  Serial.println(F("EVENT,recalibrate_start"));
  delay(400);
  calibrate_gyro_bias(GYRO_BIAS_SAMPLES);
  zero_yaw_state();
  last_imu_ms = millis();
  enter_phase(PHASE_DONE);
  Serial.println(F("EVENT,recalibrate_done"));
}

void handle_serial_commands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 't' || c == 'T') {
      Serial.println(F("EVENT,manual_test_start"));
      start_auto_test();
    } else if (c == 'z' || c == 'Z') {
      recalibrate_and_zero();
    } else if (c == 's' || c == 'S') {
      hard_stop_motors();
      test_running = false;
      enter_phase(PHASE_DONE);
      Serial.println(F("EVENT,manual_stop"));
    } else if (c == 'h' || c == 'H') {
      print_help();
    }
  }
}

void maybe_advance_test() {
  uint32_t now = millis();
  uint32_t elapsed = now - phase_start_ms;

  switch (current_phase) {
    case PHASE_BOOT_WAIT:
      if (elapsed >= BOOT_SETTLE_MS) {
        start_auto_test();
      }
      break;

    case PHASE_STILL_BEFORE:
      if (elapsed >= STILL_BASELINE_MS) enter_phase(PHASE_SPIN_LNEG_RPOS);
      break;

    case PHASE_SPIN_LNEG_RPOS:
      if (elapsed >= SPIN_DURATION_MS) enter_phase(PHASE_STILL_MID);
      break;

    case PHASE_STILL_MID:
      if (elapsed >= BETWEEN_SPINS_MS) enter_phase(PHASE_SPIN_LPOS_RNEG);
      break;

    case PHASE_SPIN_LPOS_RNEG:
      if (elapsed >= SPIN_DURATION_MS) enter_phase(PHASE_STILL_AFTER);
      break;

    case PHASE_STILL_AFTER:
      if (elapsed >= FINAL_STILL_MS) {
        test_running = false;
        enter_phase(PHASE_DONE);
      }
      break;

    case PHASE_DONE:
    default:
      break;
  }
}

void print_telemetry() {
  uint32_t now = millis();
  if ((uint32_t)(now - last_print_ms) < TELEMETRY_PERIOD_MS) return;
  last_print_ms = now;

  Serial.print(F("DATA,"));
  Serial.print(now);
  Serial.print(F(","));
  Serial.print(phase_name(current_phase));
  Serial.print(F(","));
  Serial.print(cmd_pwm_l);
  Serial.print(F(","));
  Serial.print(cmd_pwm_r);
  Serial.print(F(","));
  Serial.print(gyro_z_raw);
  Serial.print(F(","));
  Serial.print(gyro_bias_z, 3);
  Serial.print(F(","));
  Serial.print(yaw_rate_dps, 4);
  Serial.print(F(","));
  Serial.print(yaw_rate_rad_s, 6);
  Serial.print(F(","));
  Serial.print(yaw_wrapped_rad, 6);
  Serial.print(F(","));
  Serial.print(yaw_unwrapped_rad, 6);
  Serial.print(F(","));
  Serial.print(acc_x_raw);
  Serial.print(F(","));
  Serial.print(acc_y_raw);
  Serial.print(F(","));
  Serial.println(acc_z_raw);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  boot_log(F("serial"));

  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  digitalWrite(PIN_Motor_STBY, HIGH);

  hard_stop_motors();

  Wire.begin();
  boot_log(F("wire"));
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(25000U, true);
  boot_log(F("wire-timeout"));
#endif

  if (i2c_device_present(0x68U)) {
    boot_log(F("imu-detected"));
    mpu.initialize();
    boot_log(F("imu-init"));
    delay(900);
    Serial.println(F("EVENT,keep_robot_still_for_calibration"));
    calibrate_gyro_bias(GYRO_BIAS_SAMPLES);
    imu_ready = true;
    zero_yaw_state();
    boot_log(F("gyro-cal-done"));
  } else {
    imu_ready = false;
    boot_log(F("imu-missing"));
  }

  print_csv_header();
  print_help();

  uint32_t now = millis();
  last_imu_ms = now;
  last_print_ms = now;
  phase_start_ms = now;
  reset_phase_stats();
  current_phase = PHASE_BOOT_WAIT;

  Serial.print(F("EVENT,boot_wait_ms,"));
  Serial.println(BOOT_SETTLE_MS);
}

void loop() {
  handle_serial_commands();
  update_imu();

  if (imu_ready) {
    maybe_advance_test();
    print_telemetry();
  } else {
    hard_stop_motors();
  }
}
