#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===================== BNO055 =====================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool imuOk = false;

// ===================== MOTORI =====================
#define M1_IN1 7
#define M1_IN2 9

#define M2_IN1 8
#define M2_IN2 10

// ===================== ENCODER =====================
#define ENC1_A 2
#define ENC1_B 4

#define ENC2_A 3
#define ENC2_B 5

volatile long enc1_ticks = 0;
volatile long enc2_ticks = 0;

// ===================== BATTERIA =====================
#define BAT_PIN A0

// ===================== TIMING =====================
unsigned long lastPrint = 0;
unsigned long lastImuRead = 0;

// ===================== IMU DATA =====================
float yaw = 0;
float roll = 0;
float pitch = 0;

uint8_t sys = 0;
uint8_t gyro = 0;
uint8_t accel = 0;
uint8_t mag = 0;

// ===================== ISR =====================
void enc1ISR() {
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B))
    enc1_ticks++;
  else
    enc1_ticks--;
}

void enc2ISR() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B))
    enc2_ticks++;
  else
    enc2_ticks--;
}

// ===================== MOTORI =====================
void stopMotors() {
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN2, 0);
}

void forward() {
  digitalWrite(M1_IN1, HIGH);
  analogWrite(M1_IN2, 130);

  digitalWrite(M2_IN1, HIGH);
  analogWrite(M2_IN2, 130);
}

void backward() {
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_IN2, 130);

  digitalWrite(M2_IN1, LOW);
  analogWrite(M2_IN2, 130);
}

void turnRight() {
  digitalWrite(M1_IN1, HIGH);
  analogWrite(M1_IN2, 130);

  digitalWrite(M2_IN1, LOW);
  analogWrite(M2_IN2, 130);
}

void turnLeft() {
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_IN2, 130);

  digitalWrite(M2_IN1, HIGH);
  analogWrite(M2_IN2, 130);
}

// ===================== BATTERIA =====================
float readBattery() {
  long sum = 0;

  for (int i = 0; i < 10; i++) {
    sum += analogRead(BAT_PIN);
    delay(2);
  }

  float raw = sum / 10.0;
  float v_out = raw * (5.0 / 1023.0);

  return v_out * 5.0;
}

// ===================== IMU =====================
void updateIMU() {
  if (!imuOk) return;

  if (millis() - lastImuRead < 200)
    return;

  lastImuRead = millis();

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  yaw = orientationData.orientation.x;
  roll = orientationData.orientation.y;
  pitch = orientationData.orientation.z;

  bno.getCalibration(&sys, &gyro, &accel, &mag);
}

// ===================== PRINT =====================
void printSensors(const char* phase) {
  if (millis() - lastPrint < 500)
    return;

  lastPrint = millis();

  noInterrupts();
  long e1 = enc1_ticks;
  long e2 = enc2_ticks;
  interrupts();

  Serial.println();

  Serial.print("=== ");
  Serial.print(phase);
  Serial.println(" ===");

  Serial.print("ENC1: ");
  Serial.print(e1);

  Serial.print(" | ENC2: ");
  Serial.println(e2);

  Serial.print("Battery: ");
  Serial.print(readBattery(), 2);
  Serial.println(" V");

  if (imuOk) {
    Serial.print("Yaw: ");
    Serial.print(yaw, 2);

    Serial.print(" | Roll: ");
    Serial.print(roll, 2);

    Serial.print(" | Pitch: ");
    Serial.println(pitch, 2);

    Serial.print("Calib SYS:");
    Serial.print(sys);

    Serial.print(" G:");
    Serial.print(gyro);

    Serial.print(" A:");
    Serial.print(accel);

    Serial.print(" M:");
    Serial.println(mag);
  } else {
    Serial.println("IMU not detected");
  }
}

// ===================== MOVEMENT =====================
void runMotion(const char* name, void (*func)(), int duration) {
  Serial.print(">>> ");
  Serial.println(name);

  func();

  unsigned long start = millis();

  while (millis() - start < duration) {
    updateIMU();
    printSensors(name);
  }

  stopMotors();

  delay(1000);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  delay(2000);

  Serial.println("===== SYSTEM TEST NO DHT =====");

  // MOTORI
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  stopMotors();

  // ENCODER
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);

  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, CHANGE);

  // IMU
  Wire.begin();

  if (bno.begin()) {
    imuOk = true;

    bno.setExtCrystalUse(true);

    Serial.println("[OK] BNO055 detected");
  } else {
    imuOk = false;

    Serial.println("[ERROR] BNO055 not detected");
  }

  Serial.println("[OK] Setup complete");
}

// ===================== LOOP =====================
void loop() {
  updateIMU();

  printSensors("IDLE");

  runMotion("FORWARD", forward, 1500);

  runMotion("BACKWARD", backward, 1500);

  runMotion("RIGHT", turnRight, 1200);

  runMotion("LEFT", turnLeft, 1200);

  Serial.println("Cycle done");

  delay(2000);
}