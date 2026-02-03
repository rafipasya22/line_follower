#include <Arduino.h>

#define S1 6
#define S2 5
#define S3 4
#define S4 3
#define S5 2

#define ENA 9
#define IN1 7
#define IN2 10

#define ENB 11
#define IN3 8
#define IN4 12
float Kp = 14.0;
float Ki = 0.0;
float Kd = 6.0;

float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pid = 0;

float integralLimit = 50;

int baseSpeed = 70;
int maxSpeed  = 120;

int lastDirection = 0;
int recoveryBackSpeed = -60;   
int recoveryTurnSpeed = 90;    
int recoveryDelay = 120;       

void motorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void motorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}
void setup() {
  Serial.begin(9600);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  int s[5];
  s[0] = digitalRead(S1);
  s[1] = digitalRead(S2);
  s[2] = digitalRead(S3);
  s[3] = digitalRead(S4);
  s[4] = digitalRead(S5);

  int weights[5] = {12, 6, 0, -6, -12};

  int activeCount = 0;
  int weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    if (s[i] == 0) { 
      weightedSum += weights[i];
      activeCount++;
    }
  }

  if (activeCount == 0) {

    motorLeft(recoveryBackSpeed);
    motorRight(recoveryBackSpeed);
    delay(recoveryDelay);

    if (lastDirection < 0) {
      motorLeft(-recoveryTurnSpeed);
      motorRight(recoveryTurnSpeed);
    } else {
      motorLeft(recoveryTurnSpeed);
      motorRight(-recoveryTurnSpeed);
    }

    return;
  }

  if (activeCount == 1 && (s[0] == 0 || s[4] == 0)) {
    if (s[0] == 0) {
      lastDirection = -1;
      motorLeft(-70);
      motorRight(100);
    } else {
      lastDirection = 1;
      motorLeft(100);
      motorRight(-70);
    }
    return;
  }

  error = (float)weightedSum / activeCount;

  if (error < 0) lastDirection = -1;
  else if (error > 0) lastDirection = 1;

  integral += error;
  integral = constrain(integral, -integralLimit, integralLimit);

  derivative = error - lastError;
  pid = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int dynamicBase = baseSpeed - abs(pid) * 0.8;
  dynamicBase = constrain(dynamicBase, 40, baseSpeed);

  int leftSpeed  = dynamicBase - pid;
  int rightSpeed = dynamicBase + pid;

  leftSpeed  = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  motorLeft(leftSpeed);
  motorRight(rightSpeed);

  Serial.print("Err: "); Serial.print(error);
  Serial.print(" PID: "); Serial.print(pid);
  Serial.print(" L: "); Serial.print(leftSpeed);
  Serial.print(" R: "); Serial.println(rightSpeed);

  delay(5);
}
