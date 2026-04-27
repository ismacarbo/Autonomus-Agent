// ===================== MOTORI =====================
#define M1_IN1 10
#define M1_IN2 11

#define M2_IN1 12
#define M2_IN2 13

// ===================== ENCODER =====================
#define ENC1_A 35
#define ENC1_B 42

#define ENC2_A 31
#define ENC2_B 36

long enc1_ticks = 0;
long enc2_ticks = 0;

int lastEnc1A = 0;
int lastEnc1B = 0;
int lastEnc2A = 0;
int lastEnc2B = 0;

unsigned long lastPrint = 0;

// ===================== MOTORI =====================
void stopMotors() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}

void forward() {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);

  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
}

void backward() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);

  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);
}

void turnRight() {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);

  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);
}

void turnLeft() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);

  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
}

// ===================== ENCODER POLLING =====================
void updateEncoders() {
  int a1 = digitalRead(ENC1_A);
  int b1 = digitalRead(ENC1_B);

  int a2 = digitalRead(ENC2_A);
  int b2 = digitalRead(ENC2_B);

  // Conta solo sul fronte di A
  if (a1 != lastEnc1A) {
    if (a1 == b1) enc1_ticks++;
    else enc1_ticks--;
  }

  if (a2 != lastEnc2A) {
    if (a2 == b2) enc2_ticks++;
    else enc2_ticks--;
  }

  lastEnc1A = a1;
  lastEnc1B = b1;

  lastEnc2A = a2;
  lastEnc2B = b2;
}

void printStatus(const char *phase) {
  int a1 = digitalRead(ENC1_A);
  int b1 = digitalRead(ENC1_B);
  int a2 = digitalRead(ENC2_A);
  int b2 = digitalRead(ENC2_B);

  Serial.print(phase);
  Serial.print(" | ENC1 ticks=");
  Serial.print(enc1_ticks);
  Serial.print(" A=");
  Serial.print(a1);
  Serial.print(" B=");
  Serial.print(b1);

  Serial.print(" | ENC2 ticks=");
  Serial.print(enc2_ticks);
  Serial.print(" A=");
  Serial.print(a2);
  Serial.print(" B=");
  Serial.println(b2);
}

void runMotion(const char *name, void (*motionFunc)(), int durationMs) {
  Serial.println();
  Serial.print(">>> ");
  Serial.println(name);

  motionFunc();

  unsigned long start = millis();
  while (millis() - start < durationMs) {
    updateEncoders();

    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      printStatus(name);
    }

    delay(1);
  }

  stopMotors();
  delay(500);
  printStatus("AFTER STOP");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("===== MOTOR + ENCODER POLLING TEST =====");

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  stopMotors();

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  lastEnc1A = digitalRead(ENC1_A);
  lastEnc1B = digitalRead(ENC1_B);
  lastEnc2A = digitalRead(ENC2_A);
  lastEnc2B = digitalRead(ENC2_B);

  printStatus("START");
  Serial.println("[OK] Setup done");
}

void loop() {
  runMotion("FORWARD", forward, 1000);
  delay(800);

  runMotion("BACKWARD", backward, 1000);
  delay(800);

  runMotion("TURN RIGHT", turnRight, 800);
  delay(800);

  runMotion("TURN LEFT", turnLeft, 800);
  delay(1500);
}