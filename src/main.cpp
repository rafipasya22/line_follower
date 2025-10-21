#include <Arduino.h>

const int S1 = 2; // left-most
const int S2 = 3;
const int S3 = 4; // center
const int S4 = 5;
const int S5 = 6; // right-most

// L298N pins (change as needed)
const int L_IN1 = 8;  // left motor IN1
const int L_IN2 = 9;  // left motor IN2
const int R_IN3 = 10; // right motor IN3
const int R_IN4 = 11; // right motor IN4

// timing
const unsigned long ACTION_DELAY = 20; // ms between loops

void setup() {
  // sensors
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  // motors
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);

  stopMotors();
}

void loop() {
  bool b1 = digitalRead(S1) == HIGH; // true if sensor sees line
  bool b2 = digitalRead(S2) == HIGH;
  bool b3 = digitalRead(S3) == HIGH;
  bool b4 = digitalRead(S4) == HIGH;
  bool b5 = digitalRead(S5) == HIGH;

  // Priority: center -> slight -> hard
  if (b3 && !b2 && !b4) {
    // center only: forward
    forward();
  } else if (b2 && b3) {
    // slight left
    slightLeft();
  } else if (b4 && b3) {
    // slight right
    slightRight();
  } else if (b1 || (b2 && !b3)) {
    // strong left (outer or left without center)
    hardLeft();
  } else if (b5 || (b4 && !b3)) {
    // strong right
    hardRight();
  } else if (!b1 && !b2 && !b3 && !b4 && !b5) {
    // lost line: stop
    stopMotors();
  } else {
    // fallback: go forward
    forward();
  }

  delay(ACTION_DELAY);
}

/* Motor action helpers - fixed on/off */
void forward() {
  // LEFT forward
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  // RIGHT forward
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
}

void slightLeft() {
  // slow-ish: left stop, right forward
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
}

void slightRight() {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);
}

void hardLeft() {
  // spin-left: left backward, right forward
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
}

void hardRight() {
  // spin-right: left forward, right backward
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, HIGH);
}

void stopMotors() {
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);
}