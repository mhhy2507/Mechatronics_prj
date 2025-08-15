#include <Arduino.h>

/* =========================
   LEFT MOTOR (your first set)
   ========================= */
const int L_IN1 = 22;     // AIN1
const int L_IN2 = 18;     // AIN2
const int L_PWM = 23;     // PWMA (PWM output)

const int L_ENC_A = 34;   // Encoder A
const int L_ENC_B = 35;   // Encoder B
const int L_PPR   = 660;  // pulses per revolution

/* ==========================
   RIGHT MOTOR (your second set)
   ========================== */
const int R_IN1 = 21;     // AIN1
const int R_IN2 = 19;     // AIN2
const int R_PWM = 5;      // PWMA (PWM output)

const int R_ENC_A = 17;   // Encoder A
const int R_ENC_B = 16;   // Encoder B
const int R_PPR   = 660;  // pulses per revolution

/* ===== Logging ===== */
const unsigned long LOG_INTERVAL_MS = 10;
unsigned long lastLogTime = 0;

/* ===== Encoder state (LEFT) ===== */
volatile long  L_counts = 0;
volatile int   L_lastA  = LOW;

/* ===== Encoder state (RIGHT) ===== */
volatile long  R_counts = 0;
volatile int   R_lastA  = LOW;

/* ===== Commanded PWMs ===== */
int L_pwm_cmd = 0;   // -255..+255
int R_pwm_cmd = 0;   // -255..+255

/* ====== ISRs ====== */
void IRAM_ATTR L_handleEncoder() {
  int a = digitalRead(L_ENC_A);
  int b = digitalRead(L_ENC_B);
  if (a != L_lastA) {
    L_counts += (a == b) ? +1 : -1;
    L_lastA = a;
  }
}

void IRAM_ATTR R_handleEncoder() {
  int a = digitalRead(R_ENC_A);
  int b = digitalRead(R_ENC_B);
  if (a != R_lastA) {
    R_counts += (a == b) ? +1 : -1;
    R_lastA = a;
  }
}

/* ===== Motor helpers ===== */
void leftStop() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  analogWrite(L_PWM, 0);
  L_pwm_cmd = 0;
}

void rightStop() {
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
  analogWrite(R_PWM, 0);
  R_pwm_cmd = 0;
}

// pwm in [-255..+255]
void leftSetPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  L_pwm_cmd = pwm;

  if (pwm > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_PWM, -pwm);
  } else {
    leftStop();
  }
}

// pwm in [-255..+255]
void rightSetPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  R_pwm_cmd = pwm;

  if (pwm > 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    analogWrite(R_PWM, pwm);
  } else if (pwm < 0) {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    analogWrite(R_PWM, -pwm);
  } else {
    rightStop();
  }
}

/* ===== Setup ===== */
void setup() {
  Serial.begin(115200);

  // Left driver pins
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  leftStop();

  // Right driver pins
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  rightStop();

  // Left encoder
  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  L_lastA = digitalRead(L_ENC_A);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), L_handleEncoder, CHANGE);

  // Right encoder
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);
  R_lastA = digitalRead(R_ENC_A);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), R_handleEncoder, CHANGE);

  // CSV header
  Serial.println("Time,LPWM,RPWM,LRPM,RRPM");
}

/* ===== Loop ===== */
void loop() {
  // ===== Parse serial commands =====
  // Format:
  //   L<val>\n   -> set left PWM   (e.g., L120, L-200)
  //   R<val>\n   -> set right PWM  (e.g., R80, R-150)
  //   B<val>\n   -> set both PWM   (e.g., B50, B-100)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() >= 2) {
      char m = line.charAt(0);
      int val = line.substring(1).toInt();
      if (m == 'L' || m == 'l') {
        leftSetPWM(val);
      } else if (m == 'R' || m == 'r') {
        rightSetPWM(val);
      } else if (m == 'B' || m == 'b') {
        leftSetPWM(val);
        rightSetPWM(val);
      }
    }
  }

  // ===== Log at interval =====
  unsigned long now = millis();
  unsigned long dt = now - lastLogTime;
  if (dt >= LOG_INTERVAL_MS) {
    static long L_lastCounts = 0;
    static long R_lastCounts = 0;

    long L_delta = L_counts - L_lastCounts;
    long R_delta = R_counts - R_lastCounts;
    L_lastCounts = L_counts;
    R_lastCounts = R_counts;

    // RPM = (counts / PPR) * (60000 / dt_ms)
    float L_rpm = (L_delta / (float)L_PPR) * (60000.0f / dt);
    float R_rpm = (R_delta / (float)R_PPR) * (60000.0f / dt);

    lastLogTime = now;
    Serial.printf("%lu,%d,%d,%.2f,%.2f\n", now, L_pwm_cmd, R_pwm_cmd, L_rpm, R_rpm);
  }
}
