#include <Wire.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===================== DHT11 =====================
#define DHTPIN 6
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

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

// ===================== ISR ENCODER =====================
void enc1ISR() {
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) enc1_ticks++;
  else enc1_ticks--;
}

void enc2ISR() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B)) enc2_ticks++;
  else enc2_ticks--;
}

// ===================== MOTORI =====================
void stopMotors() {
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN2, 0);
}

void forward() {
  digitalWrite(M1_IN1, HIGH);
  analogWrite(M1_IN2, 150);

  digitalWrite(M2_IN1, HIGH);
  analogWrite(M2_IN2, 150);
}

void backward() {
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_IN2, 150);

  digitalWrite(M2_IN1, LOW);
  analogWrite(M2_IN2, 150);
}

void turnRight() {
  digitalWrite(M1_IN1, HIGH);
  analogWrite(M1_IN2, 150);

  digitalWrite(M2_IN1, LOW);
  analogWrite(M2_IN2, 150);
}

void turnLeft() {
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_IN2, 150);

  digitalWrite(M2_IN1, HIGH);
  analogWrite(M2_IN2, 150);
}

// ===================== BATTERIA =====================
float readBattery() {
  int raw = analogRead(BAT_PIN);
  float v_out = raw * (5.0 / 1023.0);
  float v_batt = v_out * 5.0;
  return v_batt;
}

// ===================== PRINT =====================
void printSensors(const char* phase) {
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

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (isnan(t) || isnan(h)) {
    Serial.println("DHT11 ERROR");
  } else {
    Serial.print("Temp: ");
    Serial.print(t);
    Serial.print(" C | Hum: ");
    Serial.print(h);
    Serial.println(" %");
  }

  if (imuOk) {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("Yaw: ");
    Serial.print(orientationData.orientation.x, 2);
    Serial.print(" deg | Roll: ");
    Serial.print(orientationData.orientation.y, 2);
    Serial.print(" deg | Pitch: ");
    Serial.print(orientationData.orientation.z, 2);
    Serial.println(" deg");

    Serial.print("Calib SYS:");
    Serial.print(sys);
    Serial.print(" G:");
    Serial.print(gyro);
    Serial.print(" A:");
    Serial.print(accel);
    Serial.print(" M:");
    Serial.println(mag);
  } else {
    Serial.println("IMU: not detected");
  }
}

// ===================== MOVIMENTO =====================
void runMotion(const char* name, void (*func)(), int duration) {
  Serial.print(">>> ");
  Serial.println(name);

  func();

  unsigned long start = millis();
  while (millis() - start < duration) {
    printSensors(name);
    delay(300);
  }

  stopMotors();
  delay(800);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("===== FULL SYSTEM TEST ARDUINO UNO =====");

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  stopMotors();

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, CHANGE);

  dht.begin();

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
  printSensors("IDLE");

  runMotion("FORWARD", forward, 1500);
  runMotion("BACKWARD", backward, 1500);
  runMotion("RIGHT", turnRight, 1200);
  runMotion("LEFT", turnLeft, 1200);

  Serial.println("Cycle done");
  delay(2000);
}