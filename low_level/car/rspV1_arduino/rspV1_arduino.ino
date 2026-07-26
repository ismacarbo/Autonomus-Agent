#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

#if defined(__AVR__)
#include <avr/wdt.h>
void disableWatchdogEarly(void) __attribute__((naked)) __attribute__((section(".init3")));
void disableWatchdogEarly(void) {
  MCUSR = 0;
  wdt_disable();
}
#endif

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
const uint8_t FW_MAJOR = 1;
const uint8_t FW_MINOR = 3;
const bool BOOT_DIAG_ASCII = true;

const uint16_t BNO080_REPORT_INTERVAL_MS = 20U;
const uint16_t BNO080_STARTUP_WAIT_MS = 1600U;
const uint16_t BNO080_ZERO_WAIT_MS = 900U;
const uint8_t BNO080_MIN_VALID_FRAMES = 3U;
const float BNO080_YAW_SIGN = 1.0f;
const float BNO080_GYRO_Z_SIGN = 1.0f;

const int LEFT_SIGN = 1;
const int RIGHT_SIGN = 1;

const uint16_t HOST_LINK_TIMEOUT_MS = 1500U;
const uint16_t DEFAULT_CMD_TIMEOUT_MS = 450U;
const uint16_t DEFAULT_IMU_TELEMETRY_MS = 50U;
const uint16_t DEFAULT_SAFETY_TELEMETRY_MS = 120U;
const uint16_t DEFAULT_ENCODER_TELEMETRY_MS = 100U;
const uint16_t DEFAULT_MOTOR_TELEMETRY_MS = 60U;
const uint16_t DEFAULT_HEARTBEAT_MS = 500U;
const uint16_t RX_IDLE_TIMEOUT_MS = 80U;
const uint8_t DEFAULT_SLEW_STEP = 16U;
const uint16_t VELOCITY_CONTROL_PERIOD_MS = 20U;
const uint16_t DEFAULT_VELOCITY_KP_Q8 = 90U;   // 0.352 PWM/(mm/s)
const uint16_t DEFAULT_VELOCITY_KI_Q8 = 128U;  // 0.500 PWM/(mm/s*s)
const uint16_t DEFAULT_VELOCITY_FF_Q8 = 160U;  // 0.625 PWM/(mm/s)
const uint16_t DEFAULT_ENCODER_TICKS_PER_REV = 360U;
const uint32_t DEFAULT_WHEEL_RADIUS_UM = 32700UL;

const uint16_t ENCODER_ARM_DELAY_MS = 350U;
const uint16_t ENCODER_MIN_SPAN_DEFAULT = 120U;
const uint16_t ENCODER_MIN_SPAN_LR = 80U;
const uint32_t ENCODER_MIN_TRANSITION_US = 250U;

const uint8_t RSP_SOF1 = 0xAA;
const uint8_t RSP_SOF2 = 0x55;
const uint8_t RSP_VERSION = 0x01;
const uint8_t RSP_FLAG_ACK_REQ = 0x01;
const uint8_t RSP_FLAG_ACK_FRAME = 0x02;
const uint8_t RSP_FLAG_ERR_FRAME = 0x04;

const uint8_t RSP_MSG_PING = 0x01;
const uint8_t RSP_MSG_ACK = 0x02;
const uint8_t RSP_MSG_ERROR = 0x03;
const uint8_t RSP_MSG_MOTOR_CMD = 0x10;
const uint8_t RSP_MSG_STOP_CMD = 0x11;
const uint8_t RSP_MSG_MODE_CMD = 0x12;
const uint8_t RSP_MSG_GYRO_ZERO_CMD = 0x13;
const uint8_t RSP_MSG_CONFIG_SET = 0x14;
const uint8_t RSP_MSG_HEARTBEAT_CMD = 0x15;
const uint8_t RSP_MSG_IMU_TELEMETRY = 0x20;
const uint8_t RSP_MSG_SAFETY_TELEMETRY = 0x21;
const uint8_t RSP_MSG_ENCODER_TELEMETRY = 0x22;
const uint8_t RSP_MSG_MOTOR_STATE = 0x23;
const uint8_t RSP_MSG_HEARTBEAT_STATE = 0x24;

const uint8_t ACK_STATUS_COMPLETED = 0x00;
const uint8_t ACK_STATUS_PENDING = 0x01;
const uint8_t ACK_STATUS_ALREADY = 0x02;
const uint8_t ACK_STATUS_REJECTED = 0x03;

const uint8_t ERR_UNKNOWN_MSG_TYPE = 0x01;
const uint8_t ERR_INVALID_LENGTH = 0x02;
const uint8_t ERR_CRC_MISMATCH = 0x03;
const uint8_t ERR_INVALID_VALUE = 0x04;
const uint8_t ERR_IMU_NOT_READY = 0x05;
const uint8_t ERR_CALIBRATION_BUSY = 0x06;
const uint8_t ERR_MOTORS_DISABLED = 0x07;
const uint8_t ERR_ENCODERS_UNAVAILABLE = 0x08;
const uint8_t ERR_SENSOR_TIMEOUT = 0x09;
const uint8_t ERR_UNSUPPORTED_MODE = 0x0A;
const uint8_t ERR_INTERNAL_FAULT = 0x0B;
const uint8_t ERR_BUSY = 0x0C;
const uint8_t ERR_NOT_IMPLEMENTED = 0x0D;

const uint8_t CONTROL_MODE_DIRECT_PWM = 0x00;
const uint8_t CONTROL_MODE_SAFE_DIRECT_PWM = 0x01;
const uint8_t CONTROL_MODE_WHEEL_VELOCITY = 0x02;

const uint8_t STOP_REASON_USER_REQUEST = 0x00;
const uint8_t STOP_REASON_HOST_TIMEOUT = 0x01;
const uint8_t STOP_REASON_OBSTACLE = 0x02;
const uint8_t STOP_REASON_SAFETY_OVERRIDE = 0x03;
const uint8_t STOP_REASON_FAULT_RECOVERY = 0x04;
const uint8_t STOP_REASON_SHUTDOWN = 0x05;

const uint8_t MODE_IDLE = 0x00;
const uint8_t MODE_MANUAL = 0x01;
const uint8_t MODE_AUTONOMOUS = 0x02;
const uint8_t MODE_CALIBRATION = 0x03;
const uint8_t MODE_EMERGENCY_STOP = 0x04;

const uint8_t VALUE_TYPE_UINT8 = 0x01;
const uint8_t VALUE_TYPE_INT16 = 0x02;
const uint8_t VALUE_TYPE_UINT16 = 0x03;
const uint8_t VALUE_TYPE_INT32 = 0x04;
const uint8_t VALUE_TYPE_UINT32 = 0x05;

const uint8_t PARAM_CMD_TIMEOUT_MS = 0x01;
const uint8_t PARAM_IMU_TELEMETRY_MS = 0x02;
const uint8_t PARAM_SAFETY_TELEMETRY_MS = 0x03;
const uint8_t PARAM_MOTOR_TELEMETRY_MS = 0x04;
const uint8_t PARAM_HEARTBEAT_MS = 0x05;
// 0x06 e 0x07 non usati più (vecchi IR/front)
const uint8_t PARAM_SLEW_STEP = 0x08;
const uint8_t PARAM_SAFETY_BYPASS = 0x09;
const uint8_t PARAM_ENCODER_TELEMETRY_MS = 0x0A;
const uint8_t PARAM_VELOCITY_KP_Q8 = 0x0B;
const uint8_t PARAM_VELOCITY_KI_Q8 = 0x0C;
const uint8_t PARAM_ENCODER_TICKS_PER_REV = 0x0D;
const uint8_t PARAM_WHEEL_RADIUS_UM = 0x0E;
const uint8_t PARAM_VELOCITY_FF_Q8 = 0x0F;

const uint16_t SAFETY_FLAG_ULTRA_VALID = 1U << 0;       // non usato
const uint16_t SAFETY_FLAG_IR_LEFT_ALERT = 1U << 1;     // non usato
const uint16_t SAFETY_FLAG_IR_RIGHT_ALERT = 1U << 2;    // non usato
const uint16_t SAFETY_FLAG_FRONT_ALERT = 1U << 3;       // non usato
const uint16_t SAFETY_FLAG_CMD_TIMEOUT = 1U << 4;
const uint16_t SAFETY_FLAG_EMERGENCY_STOP = 1U << 5;

const uint16_t MOTOR_FLAG_ENABLED = 1U << 0;
const uint16_t MOTOR_FLAG_STBY_HIGH = 1U << 1;
const uint16_t MOTOR_FLAG_CMD_TIMEOUT = 1U << 2;
const uint16_t MOTOR_FLAG_SLEW_LIMITING = 1U << 3;
const uint16_t MOTOR_FLAG_STOP_REQUESTED = 1U << 4;
const uint16_t MOTOR_FLAG_VELOCITY_CLOSED_LOOP = 1U << 5;

const uint16_t STATUS_FLAG_IMU_READY = 1U << 0;
const uint16_t STATUS_FLAG_ULTRA_READY = 1U << 1;   // lasciato per compatibilità, rimane 0
const uint16_t STATUS_FLAG_IR_READY = 1U << 2;      // lasciato per compatibilità, rimane 0
const uint16_t STATUS_FLAG_ENCODERS_READY = 1U << 3;
const uint16_t STATUS_FLAG_MOTORS_READY = 1U << 4;
const uint16_t STATUS_FLAG_CALIBRATING = 1U << 5;
const uint16_t STATUS_FLAG_FAULT_LATCHED = 1U << 6;
const uint16_t STATUS_FLAG_HOST_LINK_OK = 1U << 7;

const uint16_t ENC_FLAG_LEFT_VALID    = 1U << 0;
const uint16_t ENC_FLAG_RIGHT_VALID   = 1U << 1;
const uint16_t ENC_FLAG_LEFT_DIR_NEG  = 1U << 2;
const uint16_t ENC_FLAG_RIGHT_DIR_NEG = 1U << 3;
const uint16_t ENC_FLAG_OVERFLOW_WARN = 1U << 4;

const uint16_t RSP_MAX_PAYLOAD = 48U;
const uint8_t RSP_HEADER_LEN = 6U;

BNO080 imu;
bool imu_present = false;
float imu_yaw_zero_rad = 0.0f;
float imu_raw_yaw_rad = 0.0f;
float yaw_rad = 0.0f;
float yaw_rate_rad_s = 0.0f;
uint8_t imu_valid_frames = 0U;
int16_t acc_x_raw = 0;
int16_t acc_y_raw = 0;
int16_t acc_z_raw = 0;
int16_t gyro_z_raw = 0;

int16_t target_pwm_l = 0;
int16_t target_pwm_r = 0;
int16_t current_pwm_l = 0;
int16_t current_pwm_r = 0;
bool velocity_control_active = false;
int16_t target_velocity_l_mm_s = 0;
int16_t target_velocity_r_mm_s = 0;
float measured_velocity_l_mm_s = 0.0f;
float measured_velocity_r_mm_s = 0.0f;
float velocity_integral_l = 0.0f;
float velocity_integral_r = 0.0f;
uint16_t velocity_kp_q8 = DEFAULT_VELOCITY_KP_Q8;
uint16_t velocity_ki_q8 = DEFAULT_VELOCITY_KI_Q8;
uint16_t velocity_ff_q8 = DEFAULT_VELOCITY_FF_Q8;
uint16_t velocity_encoder_ticks_per_rev = DEFAULT_ENCODER_TICKS_PER_REV;
uint32_t velocity_wheel_radius_um = DEFAULT_WHEEL_RADIUS_UM;
uint32_t last_velocity_control_ms = 0U;
int32_t last_velocity_ticks_l = 0;
int32_t last_velocity_ticks_r = 0;
bool velocity_snapshot_valid = false;

uint8_t controller_mode = MODE_IDLE;
uint8_t tx_seq = 0U;
uint8_t last_error_code = 0U;
uint8_t last_stop_reason = STOP_REASON_USER_REQUEST;
bool imu_ready = false;
bool calibrating = false;
bool stop_requested = true;
bool safety_bypass = false;

uint16_t cmd_timeout_ms = DEFAULT_CMD_TIMEOUT_MS;
uint16_t imu_telemetry_ms = DEFAULT_IMU_TELEMETRY_MS;
uint16_t safety_telemetry_ms = DEFAULT_SAFETY_TELEMETRY_MS;
uint16_t encoder_telemetry_ms = DEFAULT_ENCODER_TELEMETRY_MS;
uint16_t motor_telemetry_ms = DEFAULT_MOTOR_TELEMETRY_MS;
uint16_t heartbeat_telemetry_ms = DEFAULT_HEARTBEAT_MS;
uint8_t slew_step = DEFAULT_SLEW_STEP;

uint32_t last_cmd_ms = 0U;
uint32_t last_imu_ms = 0U;
uint32_t last_safety_ms = 0U;
uint32_t last_encoder_tx_ms = 0U;
uint32_t last_motor_state_ms = 0U;
uint32_t last_heartbeat_ms = 0U;
uint32_t last_host_frame_ms = 0U;
uint32_t last_host_heartbeat_ms = 0U;
uint32_t host_time_ms = 0U;
uint16_t host_status_flags = 0U;
bool host_link_seen = false;

uint8_t rx_state = 0U;
uint8_t rx_header[RSP_HEADER_LEN];
uint8_t rx_payload[RSP_MAX_PAYLOAD];
uint8_t rx_header_idx = 0U;
uint16_t rx_payload_idx = 0U;
uint16_t rx_payload_len = 0U;
uint16_t rx_crc_calc = 0U;
uint16_t rx_crc_recv = 0U;
uint8_t rx_msg_type = 0U;
uint8_t rx_flags = 0U;
uint8_t rx_seq = 0U;
uint32_t last_rx_byte_ms = 0U;

struct EncoderAoState {
  uint8_t aoPin;
  int value;
  int observedMin;
  int observedMax;
  int lowThreshold;
  int highThreshold;
  bool stateHigh;
  bool armed;
  bool everArmed;
  bool valid;
  bool overflowWarn;
  uint32_t toggles;
  uint32_t fallingTicks;
  uint32_t lastTransitionUs;
  int minSpanForArm;
};



void recalcThresholds(EncoderAoState &encoder);
void reset_encoder_state(EncoderAoState &encoder);
void rearm_encoder_state(EncoderAoState &encoder);
void reset_all_encoders();
void rearm_all_encoders_for_motion();
void maybeArmEncoder(EncoderAoState &encoder);
void updateEncoderAo(EncoderAoState &encoder, bool allowArming);
void update_encoders();
int32_t left_ticks_total();
int32_t right_ticks_total();
uint16_t build_encoder_flags();
void clear_encoder_warnings();

EncoderAoState enc_rf = {ENC_RF_AO, 0, 1023, 0, 0, 0, false, false, false, false, false, 0UL, 0UL, 0UL, ENCODER_MIN_SPAN_DEFAULT};
EncoderAoState enc_rr = {ENC_RR_AO, 0, 1023, 0, 0, 0, false, false, false, false, false, 0UL, 0UL, 0UL, ENCODER_MIN_SPAN_DEFAULT};
EncoderAoState enc_lf = {ENC_LF_AO, 0, 1023, 0, 0, 0, false, false, false, false, false, 0UL, 0UL, 0UL, ENCODER_MIN_SPAN_DEFAULT};
EncoderAoState enc_lr = {ENC_LR_AO, 0, 1023, 0, 0, 0, false, false, false, false, false, 0UL, 0UL, 0UL, ENCODER_MIN_SPAN_LR};

uint32_t encoder_motion_start_ms = 0U;
uint32_t last_encoder_sample_ms = 0U;
int16_t prev_current_pwm_l = 0;
int16_t prev_current_pwm_r = 0;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline uint16_t clampu16(uint32_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (uint16_t)v;
}

static inline float wrap_pi(float a) {
  while (a > 3.14159265f) a -= 6.28318531f;
  while (a < -3.14159265f) a += 6.28318531f;
  return a;
}

static inline int16_t clamp_i16_long(long value) {
  if (value < -32768L) return (int16_t)-32768;
  if (value > 32767L) return (int16_t)32767;
  return (int16_t)value;
}

static inline bool finite_float(float value) {
  return !isnan(value) && !isinf(value);
}

static inline int16_t step_towards(int16_t current, int16_t target, uint8_t step) {
  if (target > current) return (int16_t)(current + min((int16_t)step, (int16_t)(target - current)));
  if (target < current) return (int16_t)(current - min((int16_t)step, (int16_t)(current - target)));
  return current;
}

static inline void put_u16_le(uint8_t* buf, uint8_t& idx, uint16_t value) {
  buf[idx++] = (uint8_t)(value & 0xFFU);
  buf[idx++] = (uint8_t)((value >> 8) & 0xFFU);
}

static inline void put_i16_le(uint8_t* buf, uint8_t& idx, int16_t value) {
  put_u16_le(buf, idx, (uint16_t)value);
}

static inline void put_u32_le(uint8_t* buf, uint8_t& idx, uint32_t value) {
  buf[idx++] = (uint8_t)(value & 0xFFUL);
  buf[idx++] = (uint8_t)((value >> 8) & 0xFFUL);
  buf[idx++] = (uint8_t)((value >> 16) & 0xFFUL);
  buf[idx++] = (uint8_t)((value >> 24) & 0xFFUL);
}

static inline void put_i32_le(uint8_t* buf, uint8_t& idx, int32_t value) {
  put_u32_le(buf, idx, (uint32_t)value);
}

static inline uint16_t read_u16_le(const uint8_t* data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static inline int16_t read_i16_le(const uint8_t* data) {
  return (int16_t)read_u16_le(data);
}

static inline uint32_t read_u32_le(const uint8_t* data) {
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static inline int32_t read_i32_le(const uint8_t* data) {
  return (int32_t)read_u32_le(data);
}

void boot_log(const __FlashStringHelper* msg) {
  if (!BOOT_DIAG_ASCII) return;
  Serial.print(F("[BOOT] "));
  Serial.println(msg);
  Serial.flush();
}

bool i2c_device_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  uint8_t rc = Wire.endTransmission();
  return rc == 0U;
}

uint16_t crc16_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; ++i) {
    if (crc & 0x8000U) crc = (uint16_t)((crc << 1) ^ 0x1021U);
    else crc <<= 1;
  }
  return crc;
}

uint16_t crc16_compute(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFFU;
  for (uint16_t i = 0; i < len; ++i) crc = crc16_update(crc, data[i]);
  return crc;
}

uint16_t crc16_extend(uint16_t crc, const uint8_t* data, uint16_t len) {
  for (uint16_t i = 0; i < len; ++i) crc = crc16_update(crc, data[i]);
  return crc;
}

bool host_link_ok() {
  if (!host_link_seen) return false;
  return (uint32_t)(millis() - last_host_frame_ms) <= HOST_LINK_TIMEOUT_MS;
}

bool command_timeout_active() {
  return (uint32_t)(millis() - last_cmd_ms) > cmd_timeout_ms;
}

bool motors_enabled_mode() {
  return controller_mode == MODE_MANUAL || controller_mode == MODE_AUTONOMOUS;
}

bool fault_latched() {
  return controller_mode == MODE_EMERGENCY_STOP;
}

bool encoders_ready() {
  return enc_rf.valid && enc_rr.valid && enc_lf.valid && enc_lr.valid &&
         enc_rf.everArmed && enc_rr.everArmed && enc_lf.everArmed && enc_lr.everArmed;
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

void recalcThresholds(EncoderAoState &encoder) {
  const int span = encoder.observedMax - encoder.observedMin;
  const int center = (encoder.observedMax + encoder.observedMin) / 2;
  const int hysteresis = max(18, span / 10);
  encoder.lowThreshold = center - hysteresis;
  encoder.highThreshold = center + hysteresis;
}

void reset_encoder_state(EncoderAoState &encoder) {
  bool valid = false;
  int value = safeAnalogReadIfValid(encoder.aoPin, valid);
  encoder.valid = valid;
  encoder.value = value;
  encoder.observedMin = valid ? value : 0;
  encoder.observedMax = valid ? value : 0;
  encoder.toggles = 0;
  encoder.fallingTicks = 0;
  encoder.stateHigh = false;
  encoder.armed = false;
  encoder.everArmed = false;
  encoder.overflowWarn = false;
  encoder.lastTransitionUs = 0U;
  recalcThresholds(encoder);
}

void rearm_encoder_state(EncoderAoState &encoder) {
  bool valid = false;
  int value = safeAnalogReadIfValid(encoder.aoPin, valid);
  encoder.valid = valid;
  encoder.value = value;
  encoder.observedMin = valid ? value : 0;
  encoder.observedMax = valid ? value : 0;
  encoder.stateHigh = false;
  encoder.armed = false;
  encoder.overflowWarn = false;
  encoder.lastTransitionUs = 0U;
  recalcThresholds(encoder);
}

void reset_all_encoders() {
  reset_encoder_state(enc_rf);
  reset_encoder_state(enc_rr);
  reset_encoder_state(enc_lf);
  reset_encoder_state(enc_lr);
  encoder_motion_start_ms = millis();
  last_encoder_sample_ms = millis();
}

void rearm_all_encoders_for_motion() {
  rearm_encoder_state(enc_rf);
  rearm_encoder_state(enc_rr);
  rearm_encoder_state(enc_lf);
  rearm_encoder_state(enc_lr);
  encoder_motion_start_ms = millis();
  last_encoder_sample_ms = millis();
}

void maybeArmEncoder(EncoderAoState &encoder) {
  if (!encoder.valid || encoder.armed) return;
  const int span = encoder.observedMax - encoder.observedMin;
  if (span < encoder.minSpanForArm) return;
  recalcThresholds(encoder);
  const int center = (encoder.observedMax + encoder.observedMin) / 2;
  encoder.stateHigh = (encoder.value >= center);
  encoder.armed = true;
  encoder.everArmed = true;
  encoder.lastTransitionUs = micros();
}

void updateEncoderAo(EncoderAoState &encoder, bool allowArming) {
  bool valid = false;
  const int value = safeAnalogReadIfValid(encoder.aoPin, valid);
  encoder.valid = valid;
  encoder.value = value;
  if (!valid) return;

  if (value < encoder.observedMin) encoder.observedMin = value;
  if (value > encoder.observedMax) encoder.observedMax = value;

  if (!encoder.armed) {
    recalcThresholds(encoder);
    if (allowArming) maybeArmEncoder(encoder);
    return;
  }

  const bool prevStateHigh = encoder.stateHigh;
  if (!encoder.stateHigh && value >= encoder.highThreshold) encoder.stateHigh = true;
  else if (encoder.stateHigh && value <= encoder.lowThreshold) encoder.stateHigh = false;

  if (prevStateHigh != encoder.stateHigh) {
    const uint32_t nowUs = micros();
    if (encoder.lastTransitionUs != 0U &&
        (uint32_t)(nowUs - encoder.lastTransitionUs) < ENCODER_MIN_TRANSITION_US) {
      encoder.stateHigh = prevStateHigh;
      encoder.overflowWarn = true;
      return;
    }
    encoder.lastTransitionUs = nowUs;
    encoder.toggles++;
    if (prevStateHigh && !encoder.stateHigh) encoder.fallingTicks++;
  }
}

void update_encoders() {
  bool moving_now = (abs(current_pwm_l) > 0) || (abs(current_pwm_r) > 0);
  bool moving_prev = (abs(prev_current_pwm_l) > 0) || (abs(prev_current_pwm_r) > 0);

  if (moving_now && !moving_prev) {
    rearm_all_encoders_for_motion();
  }

  bool allowArming = moving_now && ((uint32_t)(millis() - encoder_motion_start_ms) >= ENCODER_ARM_DELAY_MS);

  updateEncoderAo(enc_rf, allowArming);
  updateEncoderAo(enc_rr, allowArming);
  updateEncoderAo(enc_lf, allowArming);
  updateEncoderAo(enc_lr, allowArming);

  prev_current_pwm_l = current_pwm_l;
  prev_current_pwm_r = current_pwm_r;
}

int32_t left_ticks_total() {
  return (int32_t)(enc_lf.fallingTicks + enc_lr.fallingTicks);
}

int32_t right_ticks_total() {
  return (int32_t)(enc_rf.fallingTicks + enc_rr.fallingTicks);
}

uint16_t build_encoder_flags() {
  uint16_t flags = 0U;
  bool leftValid = enc_lf.valid && enc_lr.valid;
  bool rightValid = enc_rf.valid && enc_rr.valid;
  if (leftValid) flags |= ENC_FLAG_LEFT_VALID;
  if (rightValid) flags |= ENC_FLAG_RIGHT_VALID;
  if (current_pwm_l < 0) flags |= ENC_FLAG_LEFT_DIR_NEG;
  if (current_pwm_r < 0) flags |= ENC_FLAG_RIGHT_DIR_NEG;
  if (enc_rf.overflowWarn || enc_rr.overflowWarn || enc_lf.overflowWarn || enc_lr.overflowWarn) {
    flags |= ENC_FLAG_OVERFLOW_WARN;
  }
  return flags;
}

void clear_encoder_warnings() {
  enc_rf.overflowWarn = false;
  enc_rr.overflowWarn = false;
  enc_lf.overflowWarn = false;
  enc_lr.overflowWarn = false;
}

void set_motor_hw(int16_t pwm_l, int16_t pwm_r) {
  // Shield mapping confirmed:
  // A = right side, B = left side
  int hw_l = clampi((int)LEFT_SIGN * (int)pwm_l, -255, 255);
  int hw_r = clampi((int)RIGHT_SIGN * (int)pwm_r, -255, 255);

  digitalWrite(PIN_Motor_AIN_1, hw_r >= 0 ? HIGH : LOW);
  digitalWrite(PIN_Motor_BIN_1, hw_l >= 0 ? HIGH : LOW);

  analogWrite(PIN_Motor_PWMA, abs(hw_r));
  analogWrite(PIN_Motor_PWMB, abs(hw_l));
}

void hard_stop_motors() {
  target_pwm_l = 0;
  target_pwm_r = 0;
  current_pwm_l = 0;
  current_pwm_r = 0;
  velocity_control_active = false;
  target_velocity_l_mm_s = 0;
  target_velocity_r_mm_s = 0;
  measured_velocity_l_mm_s = 0.0f;
  measured_velocity_r_mm_s = 0.0f;
  velocity_integral_l = 0.0f;
  velocity_integral_r = 0.0f;
  velocity_snapshot_valid = false;
  set_motor_hw(0, 0);
}

void set_targets(int16_t pwm_l, int16_t pwm_r) {
  velocity_control_active = false;
  target_pwm_l = (int16_t)clampi((int)pwm_l, -255, 255);
  target_pwm_r = (int16_t)clampi((int)pwm_r, -255, 255);
  last_cmd_ms = millis();
}

void set_velocity_targets(int16_t left_mm_s, int16_t right_mm_s) {
  target_velocity_l_mm_s = (int16_t)clampi((int)left_mm_s, -2000, 2000);
  target_velocity_r_mm_s = (int16_t)clampi((int)right_mm_s, -2000, 2000);
  velocity_control_active = true;
  last_cmd_ms = millis();
}

int16_t velocity_pid_output(int16_t target_mm_s,
                            float measured_mm_s,
                            float dt_s,
                            float* integral) {
  if (abs(target_mm_s) < 2) {
    *integral = 0.0f;
    return 0;
  }
  const float error = (float)target_mm_s - measured_mm_s;
  *integral += error * dt_s;
  const float ki = (float)velocity_ki_q8 / 256.0f;
  const float integral_limit = ki > 0.001f ? 120.0f / ki : 0.0f;
  *integral = constrain(*integral, -integral_limit, integral_limit);
  const float ff = ((float)velocity_ff_q8 / 256.0f) * (float)target_mm_s;
  const float feedback = ((float)velocity_kp_q8 / 256.0f) * error + ki * *integral;
  return (int16_t)clampi((int)lroundf(ff + feedback), -255, 255);
}

void update_velocity_control() {
  if (!velocity_control_active) return;
  const uint32_t now = millis();
  if ((uint32_t)(now - last_velocity_control_ms) < VELOCITY_CONTROL_PERIOD_MS) return;
  const uint32_t dt_ms = (uint32_t)(now - last_velocity_control_ms);
  last_velocity_control_ms = now;
  const int32_t ticks_l = left_ticks_total();
  const int32_t ticks_r = right_ticks_total();
  if (!velocity_snapshot_valid) {
    last_velocity_ticks_l = ticks_l;
    last_velocity_ticks_r = ticks_r;
    velocity_snapshot_valid = true;
    return;
  }
  const int32_t delta_l_abs = ticks_l - last_velocity_ticks_l;
  const int32_t delta_r_abs = ticks_r - last_velocity_ticks_r;
  last_velocity_ticks_l = ticks_l;
  last_velocity_ticks_r = ticks_r;
  const float distance_per_tick_mm =
      6.28318531f * ((float)velocity_wheel_radius_um / 1000.0f) /
      (float)max((uint16_t)1U, velocity_encoder_ticks_per_rev);
  const float inv_dt = 1000.0f / (float)max((uint32_t)1U, dt_ms);
  measured_velocity_l_mm_s =
      (target_velocity_l_mm_s < 0 ? -1.0f : 1.0f) * delta_l_abs * distance_per_tick_mm * inv_dt;
  measured_velocity_r_mm_s =
      (target_velocity_r_mm_s < 0 ? -1.0f : 1.0f) * delta_r_abs * distance_per_tick_mm * inv_dt;
  const float dt_s = (float)dt_ms * 0.001f;
  target_pwm_l = velocity_pid_output(
      target_velocity_l_mm_s, measured_velocity_l_mm_s, dt_s, &velocity_integral_l);
  target_pwm_r = velocity_pid_output(
      target_velocity_r_mm_s, measured_velocity_r_mm_s, dt_s, &velocity_integral_r);
}

void update_imu() {
  if (!imu_present) return;
  uint32_t now = millis();
  if (!imu.dataAvailable()) return;

  const float raw_yaw = imu.getYaw();
  const float gyro_z = imu.getGyroZ();
  const float acc_x = imu.getAccelX();
  const float acc_y = imu.getAccelY();
  const float acc_z = imu.getAccelZ();

  if (finite_float(acc_x)) acc_x_raw = clamp_i16_long(lroundf(acc_x * 1000.0f));
  if (finite_float(acc_y)) acc_y_raw = clamp_i16_long(lroundf(acc_y * 1000.0f));
  if (finite_float(acc_z)) acc_z_raw = clamp_i16_long(lroundf(acc_z * 1000.0f));

  if (finite_float(gyro_z)) {
    yaw_rate_rad_s = BNO080_GYRO_Z_SIGN * gyro_z;
    gyro_z_raw = clamp_i16_long(lroundf(yaw_rate_rad_s * 1000.0f));
  }

  if (finite_float(raw_yaw)) {
    imu_raw_yaw_rad = raw_yaw;
    yaw_rad = wrap_pi(BNO080_YAW_SIGN * (raw_yaw - imu_yaw_zero_rad));
    if (imu_valid_frames < BNO080_MIN_VALID_FRAMES) {
      ++imu_valid_frames;
    }
    if (imu_valid_frames >= BNO080_MIN_VALID_FRAMES) {
      imu_ready = true;
    }
  }

  last_imu_ms = now;
}

bool zero_bno080_yaw_reference(uint16_t wait_ms) {
  if (!imu_present) return false;

  const uint32_t start = millis();
  do {
    update_imu();
    if (imu_ready && finite_float(imu_raw_yaw_rad)) {
      imu_yaw_zero_rad = imu_raw_yaw_rad;
      yaw_rad = 0.0f;
      yaw_rate_rad_s = 0.0f;
      return true;
    }
    delay(5);
  } while ((uint32_t)(millis() - start) < wait_ms);

  return false;
}

bool send_frame(uint8_t msg_type, uint8_t flags, const uint8_t* payload, uint16_t len) {
  uint16_t total_len = (uint16_t)(2U + RSP_HEADER_LEN + len + 2U);
  if (Serial.availableForWrite() < (int)total_len) return false;

  uint8_t header[RSP_HEADER_LEN];
  header[0] = RSP_VERSION;
  header[1] = msg_type;
  header[2] = flags;
  header[3] = tx_seq++;
  header[4] = (uint8_t)(len & 0xFFU);
  header[5] = (uint8_t)((len >> 8) & 0xFFU);

  uint16_t crc = crc16_compute(header, RSP_HEADER_LEN);
  if (len > 0U && payload != NULL) crc = crc16_extend(crc, payload, len);

  Serial.write(RSP_SOF1);
  Serial.write(RSP_SOF2);
  Serial.write(header, RSP_HEADER_LEN);
  if (len > 0U && payload != NULL) Serial.write(payload, len);
  Serial.write((uint8_t)(crc & 0xFFU));
  Serial.write((uint8_t)((crc >> 8) & 0xFFU));
  return true;
}

void send_ack(uint8_t acked_seq, uint8_t acked_type, uint8_t status, uint8_t detail) {
  uint8_t payload[4];
  payload[0] = acked_seq;
  payload[1] = acked_type;
  payload[2] = status;
  payload[3] = detail;
  send_frame(RSP_MSG_ACK, RSP_FLAG_ACK_FRAME, payload, sizeof(payload));
}

void send_error(uint8_t error_code, uint8_t related_type, uint8_t related_seq, uint8_t detail) {
  uint8_t payload[4];
  payload[0] = error_code;
  payload[1] = related_type;
  payload[2] = related_seq;
  payload[3] = detail;
  last_error_code = error_code;
  send_frame(RSP_MSG_ERROR, RSP_FLAG_ERR_FRAME, payload, sizeof(payload));
}

void reject_frame(uint8_t msg_type, uint8_t seq, bool ack_req, uint8_t error_code, uint8_t detail) {
  send_error(error_code, msg_type, seq, detail);
  if (ack_req) send_ack(seq, msg_type, ACK_STATUS_REJECTED, error_code);
}

bool validate_length(uint8_t msg_type, uint16_t len) {
  switch (msg_type) {
    case RSP_MSG_PING: return len == 0U;
    case RSP_MSG_ACK: return len == 4U;
    case RSP_MSG_ERROR: return len == 4U;
    case RSP_MSG_MOTOR_CMD: return len == 6U;
    case RSP_MSG_STOP_CMD: return len == 1U;
    case RSP_MSG_MODE_CMD: return len == 2U;
    case RSP_MSG_GYRO_ZERO_CMD: return len == 0U;
    case RSP_MSG_CONFIG_SET: return len >= 2U;
    case RSP_MSG_HEARTBEAT_CMD: return len == 6U;
    default: return false;
  }
}

bool read_config_value(uint8_t value_type, const uint8_t* data, uint16_t len, int32_t* out_value) {
  if (out_value == NULL) return false;
  switch (value_type) {
    case VALUE_TYPE_UINT8:
      if (len != 1U) return false;
      *out_value = (int32_t)data[0];
      return true;
    case VALUE_TYPE_INT16:
      if (len != 2U) return false;
      *out_value = (int32_t)read_i16_le(data);
      return true;
    case VALUE_TYPE_UINT16:
      if (len != 2U) return false;
      *out_value = (int32_t)read_u16_le(data);
      return true;
    case VALUE_TYPE_INT32:
      if (len != 4U) return false;
      *out_value = read_i32_le(data);
      return true;
    case VALUE_TYPE_UINT32:
      if (len != 4U) return false;
      *out_value = (int32_t)read_u32_le(data);
      return true;
    default:
      return false;
  }
}

bool apply_config(uint8_t param_id, uint8_t value_type, int32_t value) {
  switch (param_id) {
    case PARAM_CMD_TIMEOUT_MS:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      cmd_timeout_ms = clampu16((uint32_t)value, 50U, 5000U);
      return true;
    case PARAM_IMU_TELEMETRY_MS:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      imu_telemetry_ms = clampu16((uint32_t)value, 10U, 1000U);
      return true;
    case PARAM_SAFETY_TELEMETRY_MS:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      safety_telemetry_ms = clampu16((uint32_t)value, 20U, 2000U);
      return true;
    case PARAM_MOTOR_TELEMETRY_MS:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      motor_telemetry_ms = clampu16((uint32_t)value, 10U, 1000U);
      return true;
    case PARAM_HEARTBEAT_MS:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      heartbeat_telemetry_ms = clampu16((uint32_t)value, 100U, 5000U);
      return true;
    case PARAM_SLEW_STEP:
      if (value_type != VALUE_TYPE_UINT8) return false;
      slew_step = (uint8_t)clampi((int)value, 1, 64);
      return true;
    case PARAM_SAFETY_BYPASS:
      if (value_type != VALUE_TYPE_UINT8) return false;
      safety_bypass = (value != 0);
      return true;
    case PARAM_ENCODER_TELEMETRY_MS:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      encoder_telemetry_ms = clampu16((uint32_t)value, 20U, 2000U);
      return true;
    case PARAM_VELOCITY_KP_Q8:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      velocity_kp_q8 = clampu16((uint32_t)value, 0U, 2048U);
      return true;
    case PARAM_VELOCITY_KI_Q8:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      velocity_ki_q8 = clampu16((uint32_t)value, 0U, 4096U);
      return true;
    case PARAM_ENCODER_TICKS_PER_REV:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      velocity_encoder_ticks_per_rev = clampu16((uint32_t)value, 1U, 30000U);
      return true;
    case PARAM_WHEEL_RADIUS_UM:
      if (value_type != VALUE_TYPE_UINT32 || value < 5000L || value > 250000L) return false;
      velocity_wheel_radius_um = (uint32_t)value;
      return true;
    case PARAM_VELOCITY_FF_Q8:
      if (!(value_type == VALUE_TYPE_UINT16 || value_type == VALUE_TYPE_UINT32)) return false;
      velocity_ff_q8 = clampu16((uint32_t)value, 0U, 2048U);
      return true;
    default:
      return false;
  }
}

void handle_ping(uint8_t seq, bool ack_req) {
  if (ack_req) send_ack(seq, RSP_MSG_PING, ACK_STATUS_COMPLETED, 0U);
}

void handle_mode_cmd(uint8_t seq, bool ack_req, const uint8_t* payload) {
  uint8_t mode = payload[0];
  uint8_t reserved = payload[1];

  if (reserved != 0U) {
    reject_frame(RSP_MSG_MODE_CMD, seq, ack_req, ERR_INVALID_VALUE, reserved);
    return;
  }
  if (mode > MODE_EMERGENCY_STOP) {
    reject_frame(RSP_MSG_MODE_CMD, seq, ack_req, ERR_UNSUPPORTED_MODE, mode);
    return;
  }
  if (controller_mode == mode) {
    if (ack_req) send_ack(seq, RSP_MSG_MODE_CMD, ACK_STATUS_ALREADY, mode);
    return;
  }

  controller_mode = mode;
  if (!motors_enabled_mode()) {
    stop_requested = true;
    hard_stop_motors();
  }

  if (ack_req) send_ack(seq, RSP_MSG_MODE_CMD, ACK_STATUS_COMPLETED, mode);
}

void handle_stop_cmd(uint8_t seq, bool ack_req, const uint8_t* payload) {
  last_stop_reason = payload[0];
  stop_requested = true;
  set_targets(0, 0);
  if (ack_req) send_ack(seq, RSP_MSG_STOP_CMD, ACK_STATUS_COMPLETED, last_stop_reason);
}

void handle_motor_cmd(uint8_t seq, bool ack_req, const uint8_t* payload) {
  int16_t pwm_l = read_i16_le(payload + 0);
  int16_t pwm_r = read_i16_le(payload + 2);
  uint8_t control_mode = payload[4];
  uint8_t reserved = payload[5];

  if (reserved != 0U) {
    reject_frame(RSP_MSG_MOTOR_CMD, seq, ack_req, ERR_INVALID_VALUE, reserved);
    return;
  }
  if (control_mode != CONTROL_MODE_DIRECT_PWM &&
      control_mode != CONTROL_MODE_SAFE_DIRECT_PWM &&
      control_mode != CONTROL_MODE_WHEEL_VELOCITY) {
    reject_frame(RSP_MSG_MOTOR_CMD, seq, ack_req, ERR_INVALID_VALUE, control_mode);
    return;
  }
  if (calibrating || controller_mode == MODE_CALIBRATION) {
    reject_frame(RSP_MSG_MOTOR_CMD, seq, ack_req, ERR_CALIBRATION_BUSY, 0U);
    return;
  }
  if (!motors_enabled_mode() || controller_mode == MODE_EMERGENCY_STOP) {
    reject_frame(RSP_MSG_MOTOR_CMD, seq, ack_req, ERR_MOTORS_DISABLED, controller_mode);
    return;
  }

  // SAFE_DIRECT_PWM qui non blocca più per sensori inesistenti.
  stop_requested = false;
  if (control_mode == CONTROL_MODE_WHEEL_VELOCITY) {
    set_velocity_targets(pwm_l, pwm_r);
  } else {
    set_targets(pwm_l, pwm_r);
  }

  if (ack_req) send_ack(seq, RSP_MSG_MOTOR_CMD, ACK_STATUS_COMPLETED, 0U);
}

void handle_gyro_zero_cmd(uint8_t seq, bool ack_req) {
  if (!imu_present) {
    reject_frame(RSP_MSG_GYRO_ZERO_CMD, seq, ack_req, ERR_IMU_NOT_READY, 0U);
    return;
  }
  if (calibrating) {
    reject_frame(RSP_MSG_GYRO_ZERO_CMD, seq, ack_req, ERR_BUSY, 0U);
    return;
  }

  uint8_t prev_mode = controller_mode;
  calibrating = true;
  controller_mode = MODE_CALIBRATION;
  stop_requested = true;
  hard_stop_motors();

  bool zero_ok = zero_bno080_yaw_reference(BNO080_ZERO_WAIT_MS);

  calibrating = false;
  controller_mode = prev_mode;

  if (!zero_ok) {
    last_error_code = ERR_SENSOR_TIMEOUT;
    reject_frame(RSP_MSG_GYRO_ZERO_CMD, seq, ack_req, ERR_SENSOR_TIMEOUT, 0U);
    return;
  }

  if (!motors_enabled_mode()) stop_requested = true;
  if (ack_req) send_ack(seq, RSP_MSG_GYRO_ZERO_CMD, ACK_STATUS_COMPLETED, 0U);
}

void handle_config_set(uint8_t seq, bool ack_req, const uint8_t* payload, uint16_t len) {
  uint8_t param_id = payload[0];
  uint8_t value_type = payload[1];
  int32_t value = 0;

  if (!read_config_value(value_type, payload + 2, len - 2U, &value)) {
    reject_frame(RSP_MSG_CONFIG_SET, seq, ack_req, ERR_INVALID_LENGTH, value_type);
    return;
  }
  if (!apply_config(param_id, value_type, value)) {
    reject_frame(RSP_MSG_CONFIG_SET, seq, ack_req, ERR_INVALID_VALUE, param_id);
    return;
  }
  if (ack_req) send_ack(seq, RSP_MSG_CONFIG_SET, ACK_STATUS_COMPLETED, param_id);
}

void handle_heartbeat_cmd(const uint8_t* payload) {
  host_time_ms = read_u32_le(payload + 0);
  host_status_flags = read_u16_le(payload + 4);
  last_host_heartbeat_ms = millis();
}

void handle_frame(uint8_t msg_type, uint8_t flags, uint8_t seq, const uint8_t* payload, uint16_t len) {
  bool ack_req = (flags & RSP_FLAG_ACK_REQ) != 0U;
  host_link_seen = true;
  last_host_frame_ms = millis();

  if (!validate_length(msg_type, len)) {
    reject_frame(msg_type, seq, ack_req, ERR_INVALID_LENGTH, (uint8_t)(len & 0xFFU));
    return;
  }

  switch (msg_type) {
    case RSP_MSG_PING:          handle_ping(seq, ack_req); return;
    case RSP_MSG_MOTOR_CMD:     handle_motor_cmd(seq, ack_req, payload); return;
    case RSP_MSG_STOP_CMD:      handle_stop_cmd(seq, ack_req, payload); return;
    case RSP_MSG_MODE_CMD:      handle_mode_cmd(seq, ack_req, payload); return;
    case RSP_MSG_GYRO_ZERO_CMD: handle_gyro_zero_cmd(seq, ack_req); return;
    case RSP_MSG_CONFIG_SET:    handle_config_set(seq, ack_req, payload, len); return;
    case RSP_MSG_HEARTBEAT_CMD: handle_heartbeat_cmd(payload); return;
    case RSP_MSG_ACK:
    case RSP_MSG_ERROR:
      return;
    default:
      reject_frame(msg_type, seq, ack_req, ERR_UNKNOWN_MSG_TYPE, 0U);
      return;
  }
}

void reset_rx() {
  rx_state = 0U;
  rx_header_idx = 0U;
  rx_payload_idx = 0U;
  rx_payload_len = 0U;
  rx_crc_calc = 0U;
  rx_crc_recv = 0U;
  rx_msg_type = 0U;
  rx_flags = 0U;
  rx_seq = 0U;
  last_rx_byte_ms = 0U;
}

void poll_serial() {
  if (rx_state != 0U && last_rx_byte_ms != 0U) {
    if ((uint32_t)(millis() - last_rx_byte_ms) > RX_IDLE_TIMEOUT_MS) {
      reset_rx();
    }
  }

  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    last_rx_byte_ms = millis();

    switch (rx_state) {
      case 0U:
        if (b == RSP_SOF1) rx_state = 1U;
        break;

      case 1U:
        if (b == RSP_SOF2) {
          rx_state = 2U;
          rx_header_idx = 0U;
        } else if (b != RSP_SOF1) {
          rx_state = 0U;
        }
        break;

      case 2U:
        rx_header[rx_header_idx++] = b;
        if (rx_header_idx >= RSP_HEADER_LEN) {
          if (rx_header[0] != RSP_VERSION) {
            reset_rx();
            break;
          }
          rx_msg_type = rx_header[1];
          rx_flags = rx_header[2];
          rx_seq = rx_header[3];
          rx_payload_len = read_u16_le(rx_header + 4);
          if (rx_payload_len > RSP_MAX_PAYLOAD) {
            reset_rx();
            break;
          }
          rx_crc_calc = crc16_compute(rx_header, RSP_HEADER_LEN);
          rx_payload_idx = 0U;
          rx_state = (rx_payload_len == 0U) ? 4U : 3U;
        }
        break;

      case 3U:
        rx_payload[rx_payload_idx++] = b;
        rx_crc_calc = crc16_update(rx_crc_calc, b);
        if (rx_payload_idx >= rx_payload_len) rx_state = 4U;
        break;

      case 4U:
        rx_crc_recv = b;
        rx_state = 5U;
        break;

      case 5U:
        rx_crc_recv |= (uint16_t)b << 8;
        if (rx_crc_recv == rx_crc_calc) {
          handle_frame(rx_msg_type, rx_flags, rx_seq, rx_payload, rx_payload_len);
        }
        reset_rx();
        break;

      default:
        reset_rx();
        break;
    }
  }
}

uint16_t build_safety_flags() {
  uint16_t flags = 0U;
  if (command_timeout_active()) flags |= SAFETY_FLAG_CMD_TIMEOUT;
  if (controller_mode == MODE_EMERGENCY_STOP) flags |= SAFETY_FLAG_EMERGENCY_STOP;
  return flags;
}

uint16_t build_motor_flags() {
  uint16_t flags = 0U;
  bool timeout = command_timeout_active();
  bool slew_active = (current_pwm_l != target_pwm_l) || (current_pwm_r != target_pwm_r);
  bool enabled = motors_enabled_mode() && controller_mode != MODE_EMERGENCY_STOP;
  bool stop_active = stop_requested || timeout || !enabled;

  if (enabled) flags |= MOTOR_FLAG_ENABLED;
  if (digitalRead(PIN_Motor_STBY)) flags |= MOTOR_FLAG_STBY_HIGH;
  if (timeout) flags |= MOTOR_FLAG_CMD_TIMEOUT;
  if (slew_active) flags |= MOTOR_FLAG_SLEW_LIMITING;
  if (stop_active) flags |= MOTOR_FLAG_STOP_REQUESTED;
  if (velocity_control_active) flags |= MOTOR_FLAG_VELOCITY_CLOSED_LOOP;
  return flags;
}

uint16_t build_status_flags() {
  uint16_t flags = 0U;
  if (imu_ready) flags |= STATUS_FLAG_IMU_READY;
  if (encoders_ready()) flags |= STATUS_FLAG_ENCODERS_READY;
  flags |= STATUS_FLAG_MOTORS_READY;
  if (calibrating) flags |= STATUS_FLAG_CALIBRATING;
  if (fault_latched()) flags |= STATUS_FLAG_FAULT_LATCHED;
  if (host_link_ok()) flags |= STATUS_FLAG_HOST_LINK_OK;
  return flags;
}

void send_imu_telemetry() {
  if (!imu_ready) return;
  if (!host_link_ok()) return;

  static uint32_t last_tx = 0U;
  uint32_t now = millis();
  if ((uint32_t)(now - last_tx) < imu_telemetry_ms) return;
  last_tx = now;

  uint8_t payload[20];
  uint8_t idx = 0U;
  int32_t yaw_mrad = (int32_t)lroundf(yaw_rad * 1000.0f);
  int32_t yaw_rate_mrad_s = (int32_t)lroundf(yaw_rate_rad_s * 1000.0f);

  put_u32_le(payload, idx, now);
  put_i32_le(payload, idx, yaw_mrad);
  put_i32_le(payload, idx, yaw_rate_mrad_s);
  put_i16_le(payload, idx, acc_x_raw);
  put_i16_le(payload, idx, acc_y_raw);
  put_i16_le(payload, idx, acc_z_raw);
  put_i16_le(payload, idx, gyro_z_raw);
  send_frame(RSP_MSG_IMU_TELEMETRY, 0U, payload, idx);
}

void send_safety_telemetry() {
  if (!host_link_ok()) return;
  uint32_t now = millis();
  if ((uint32_t)(now - last_safety_ms) < safety_telemetry_ms) return;
  last_safety_ms = now;

  uint8_t payload[12];
  uint8_t idx = 0U;
  put_u32_le(payload, idx, now);
  put_u16_le(payload, idx, 0U);  // ultra cm non usato
  put_u16_le(payload, idx, 0U);  // ir left non usato
  put_u16_le(payload, idx, 0U);  // ir right non usato
  put_u16_le(payload, idx, build_safety_flags());
  send_frame(RSP_MSG_SAFETY_TELEMETRY, 0U, payload, idx);
}

void send_encoder_telemetry() {
  if (!host_link_ok()) return;
  uint32_t now = millis();
  if ((uint32_t)(now - last_encoder_tx_ms) < encoder_telemetry_ms) return;

  uint16_t dt_ms = (uint16_t)min((uint32_t)65535U, (uint32_t)(now - last_encoder_tx_ms));
  last_encoder_tx_ms = now;

  uint8_t payload[16];
  uint8_t idx = 0U;
  put_u32_le(payload, idx, now);
  put_i32_le(payload, idx, left_ticks_total());
  put_i32_le(payload, idx, right_ticks_total());
  put_u16_le(payload, idx, dt_ms);
  put_u16_le(payload, idx, build_encoder_flags());
  if (send_frame(RSP_MSG_ENCODER_TELEMETRY, 0U, payload, idx)) {
    clear_encoder_warnings();
  }
}

void send_motor_state() {
  if (!host_link_ok()) return;
  uint32_t now = millis();
  if ((uint32_t)(now - last_motor_state_ms) < motor_telemetry_ms) return;
  last_motor_state_ms = now;

  uint8_t payload[14];
  uint8_t idx = 0U;
  put_u32_le(payload, idx, now);
  put_i16_le(payload, idx, target_pwm_l);
  put_i16_le(payload, idx, target_pwm_r);
  put_i16_le(payload, idx, current_pwm_l);
  put_i16_le(payload, idx, current_pwm_r);
  put_u16_le(payload, idx, build_motor_flags());
  send_frame(RSP_MSG_MOTOR_STATE, 0U, payload, idx);
}

void send_heartbeat_state() {
  if (!host_link_ok()) return;
  uint32_t now = millis();
  if ((uint32_t)(now - last_heartbeat_ms) < heartbeat_telemetry_ms) return;
  last_heartbeat_ms = now;

  uint8_t payload[12];
  uint8_t idx = 0U;
  put_u32_le(payload, idx, now);
  put_u16_le(payload, idx, (uint16_t)(now / 1000UL));
  put_u16_le(payload, idx, build_status_flags());
  payload[idx++] = FW_MAJOR;
  payload[idx++] = FW_MINOR;
  put_u16_le(payload, idx, last_error_code);
  send_frame(RSP_MSG_HEARTBEAT_STATE, 0U, payload, idx);
}

void update_motors() {
  bool timeout = command_timeout_active();
  bool allow_motion = motors_enabled_mode() && !calibrating && controller_mode != MODE_EMERGENCY_STOP;

  if (timeout) {
    target_pwm_l = 0;
    target_pwm_r = 0;
    stop_requested = true;
    last_stop_reason = STOP_REASON_HOST_TIMEOUT;
  }

  if (!allow_motion) {
    target_pwm_l = 0;
    target_pwm_r = 0;
  }

  if (allow_motion && !timeout) update_velocity_control();

  current_pwm_l = step_towards(current_pwm_l, target_pwm_l, slew_step);
  current_pwm_r = step_towards(current_pwm_r, target_pwm_r, slew_step);
  set_motor_hw(current_pwm_l, current_pwm_r);
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

  pinMode(ENC_RF_AO, INPUT);
  pinMode(ENC_RR_AO, INPUT);
  pinMode(ENC_LF_AO, INPUT);
  pinMode(ENC_LR_AO, INPUT);

  Wire.begin();
  boot_log(F("wire"));
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(25000U, true);
  boot_log(F("wire-timeout"));
#endif

  if (imu.begin()) {
    imu_present = true;
    imu_ready = false;
    boot_log(F("bno080-detected"));
    imu.enableRotationVector(BNO080_REPORT_INTERVAL_MS);
    imu.enableGyro(BNO080_REPORT_INTERVAL_MS);
    imu.enableAccelerometer(BNO080_REPORT_INTERVAL_MS);
    boot_log(F("bno080-init"));

    last_imu_ms = millis();
    uint32_t warmup_start = millis();
    while (!imu_ready && (uint32_t)(millis() - warmup_start) < BNO080_STARTUP_WAIT_MS) {
      update_imu();
      delay(5);
    }

    if (zero_bno080_yaw_reference(250U)) {
      boot_log(F("bno080-zero-done"));
    } else {
      boot_log(F("bno080-warmup-pending"));
    }
  } else {
    imu_present = false;
    imu_ready = false;
    last_error_code = ERR_IMU_NOT_READY;
    yaw_rad = 0.0f;
    yaw_rate_rad_s = 0.0f;
    boot_log(F("imu-missing"));
  }

  reset_all_encoders();

  uint32_t now = millis();
  last_cmd_ms = now;
  last_imu_ms = now;
  last_safety_ms = now;
  last_encoder_tx_ms = now;
  last_motor_state_ms = now;
  last_heartbeat_ms = now;
  last_host_frame_ms = now;
  last_host_heartbeat_ms = now;
  host_link_seen = false;

  hard_stop_motors();
  reset_rx();
  boot_log(F("setup-done"));
}

void loop() {
  poll_serial();
  update_imu();
  update_encoders();
  update_motors();
  send_imu_telemetry();
  send_safety_telemetry();
  send_encoder_telemetry();
  send_motor_state();
  send_heartbeat_state();
}
