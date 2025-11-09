#include <Arduino.h>

//pin sensor line follower
#define S1 6
#define S2 5
#define S3 4
#define S4 3
#define S5 2

//pin driver motor
#define ENA 9   // PWM motor kanan
#define IN1 7   // motor kanan maju
#define IN2 10  // motor kanan mundur

#define ENB 11  // PWM motor kiri
#define IN3 8   // motor kiri maju
#define IN4 12  // motor kiri mundur

// ---------------------------
// param pid
// ---------------------------
float Kp = 10.0;
float Ki = 0.0;
float Kd = 3.0;

float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pid = 0;

//kec motor
int baseSpeed = 80;
int maxSpeed = 100;
int minSpeed = 40;


void motorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {  // maju
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  } else if (speed < 0) {  // mundur
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  } else {  // berhenti
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void motorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {  // maju
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {  // mundur
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {  // berhenti
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

  int weights[5] = {8, 3, 0, -3, -8};
  int activeCount = 0;
  int weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    if (s[i] == 0) { // hitam terdeteksi
      weightedSum += weights[i];
      activeCount++;
    }
  }

  if(activeCount == 0){
    pid = 0;
    error = 0;
  }else{
  error = (float)weightedSum / activeCount;

  integral += error;
  derivative = error - lastError;
  pid = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;
  }



  int leftSpeed = baseSpeed - pid;
  int rightSpeed = baseSpeed + pid;

  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  motorLeft(leftSpeed);
  motorRight(rightSpeed);



  //monitor sensor
  Serial.print("Sensors: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(s[i]);
    Serial.print(" ");
  }
  Serial.print(" | Err: "); Serial.print(error);
  Serial.print(" | PID: "); Serial.print(pid);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.println(rightSpeed);

  Serial.print(digitalRead(S1)); Serial.print(" ");
  Serial.print(digitalRead(S2)); Serial.print(" ");
  Serial.print(digitalRead(S3)); Serial.print(" ");
  Serial.print(digitalRead(S4)); Serial.print(" ");
  Serial.println(digitalRead(S5));
  delay(200);

  delay(50);
}
