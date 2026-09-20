#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 14
#define IN3 32
#define IN4 33

#define OUT1 34
#define OUT2 35
#define OUT3 36
#define OUT4 4
#define OUT5 23

#define TRIG_PIN 18
#define ECHO_PIN 19

#define OBSTACLE_DISTANCE 20.0

int forwardSpeed = 140;
int lineTurnSpeed = 180;
int bypassDriveSpeed = 200;
int searchTurnSpeed = 180;
int obstacleTurnSpeed = 255;

unsigned long bypassDriveTime = 1000;

int lastDirection = 0;
bool lineLost = false;
unsigned long lostStart = 0;

float gyroZBias = 0.0;
float angleZ = 0.0;
unsigned long previousTime;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  pinMode(OUT1, INPUT);
  pinMode(OUT2, INPUT);
  pinMode(OUT3, INPUT);
  pinMode(OUT4, INPUT);
  pinMode(OUT5, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND!");

    while (1) {
      delay(10);
    }
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);

  Serial.println("KEEP ROBOT STILL...");

  float total = 0;

  for (int i = 0; i < 500; i++) {
    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t temp;

    mpu.getEvent(&a, &g, &temp);

    total += g.gyro.z;

    delay(5);
  }

  gyroZBias = total / 500.0;

  Serial.print("Gyro Z Bias: ");
  Serial.println(gyroZBias, 6);

  delay(2000);

  Serial.println("-");
  Serial.println("ROBOT READY");
  Serial.println("-");
}

void loop() {
  float distance = getDistance();

  if (distance > 0 && distance < OBSTACLE_DISTANCE) {
    Serial.println();
    Serial.println("-");
    Serial.println("OBSTACLE DETECTED");
    Serial.println("-");

    stopMotors();

    delay(500);

    Serial.println("STEP 1: TURN +90");
    turnCounterClockwise90();

    Serial.println("STEP 2: DRIVE");

    driveForward(bypassDriveSpeed);

    delay(bypassDriveTime);

    stopMotors();

    delay(300);

    Serial.println("STEP 3: TURN -90");
    turnClockwise90();

    delay(300);

    Serial.println("STEP 4: SEARCH FOR LINE");

    bool found = searchForLine();

    if (found) {
      Serial.println("-");
      Serial.println("LINE FOUND!");
      Serial.println("RESUMING LINE FOLLOWER");
      Serial.println("-");

      delay(300);
    }
    else {
      Serial.println("-");
      Serial.println("LINE NOT FOUND!");
      Serial.println("SAFETY STOP");
      Serial.println("-");

      stopMotors();

      while (1) {
        delay(1000);
      }
    }
  }

  followLine();

  delay(5);
}

bool searchForLine() {
  Serial.println("Checking current position...");

  if (lineDetected()) {
    stopMotors();
    return true;
  }

  Serial.println("Searching clockwise...");

  unsigned long phaseStart = millis();

  while (millis() - phaseStart < 2500) {
    if (lineDetected()) {
      stopMotors();
      Serial.println("LINE FOUND DURING CLOCKWISE SEARCH");
      return true;
    }

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, searchTurnSpeed);
    analogWrite(ENB, searchTurnSpeed);

    delay(5);
  }

  stopMotors();

  delay(300);

  Serial.println("Searching counter-clockwise...");

  phaseStart = millis();

  while (millis() - phaseStart < 2500) {
    if (lineDetected()) {
      stopMotors();
      Serial.println("LINE FOUND DURING COUNTER-CLOCKWISE SEARCH");
      return true;
    }

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, searchTurnSpeed);
    analogWrite(ENB, searchTurnSpeed);

    delay(5);
  }

  stopMotors();

  delay(200);

  if (lineDetected()) {
    Serial.println("LINE FOUND AFTER SEARCH");
    return true;
  }

  return false;
}

bool lineDetected() {
  int s1 = digitalRead(OUT1);
  int s2 = digitalRead(OUT2);
  int s3 = digitalRead(OUT3);
  int s4 = digitalRead(OUT4);
  int s5 = digitalRead(OUT5);

  Serial.print("IR: ");
  Serial.print(s1);
  Serial.print(" ");
  Serial.print(s2);
  Serial.print(" ");
  Serial.print(s3);
  Serial.print(" ");
  Serial.print(s4);
  Serial.print(" ");
  Serial.println(s5);

  return (
    s1 == LOW ||
    s2 == LOW ||
    s3 == LOW ||
    s4 == LOW ||
    s5 == LOW
  );
}

void followLine() {
  int s1 = digitalRead(OUT1);
  int s2 = digitalRead(OUT2);
  int s3 = digitalRead(OUT3);
  int s4 = digitalRead(OUT4);
  int s5 = digitalRead(OUT5);

  int b1 = !s1;
  int b2 = !s2;
  int b3 = !s3;
  int b4 = !s4;
  int b5 = !s5;

  int total = b1 + b2 + b3 + b4 + b5;

  if (total == 0) {
    if (!lineLost) {
      lineLost = true;
      lostStart = millis();
    }

    if (millis() - lostStart < 400) {
      if (lastDirection < 0) {
        setMotors(0, lineTurnSpeed);
      }
      else if (lastDirection > 0) {
        setMotors(lineTurnSpeed, 0);
      }
      else {
        stopMotors();
      }
    }
    else {
      stopMotors();
    }

    return;
  }

  lineLost = false;

  if (b1) {
    lastDirection = -1;
    setMotors(0, lineTurnSpeed);
  }
  else if (b2) {
    lastDirection = -1;
    setMotors(80, lineTurnSpeed);
  }
  else if (b3) {
    lastDirection = 0;
    setMotors(forwardSpeed, forwardSpeed);
  }
  else if (b4) {
    lastDirection = 1;
    setMotors(lineTurnSpeed, 80);
  }
  else if (b5) {
    lastDirection = 1;
    setMotors(lineTurnSpeed, 0);
  }
  else {
    stopMotors();
  }
}

void turnCounterClockwise90() {
  angleZ = 0;
  previousTime = micros();

  while (angleZ < 90.0) {
    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t temp;

    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = micros();

    float dt =
      (currentTime - previousTime) / 1000000.0;

    previousTime = currentTime;

    float correctedZ =
      g.gyro.z - gyroZBias;

    float angularVelocity =
      correctedZ * 57.2958;

    angleZ += angularVelocity * dt;

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, obstacleTurnSpeed);
    analogWrite(ENB, obstacleTurnSpeed);

    delay(5);
  }

  stopMotors();

  Serial.print("CCW angle: ");
  Serial.println(angleZ, 2);

  delay(500);
}

void turnClockwise90() {
  angleZ = 0;
  previousTime = micros();

  while (angleZ > -90.0) {
    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t temp;

    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = micros();

    float dt =
      (currentTime - previousTime) / 1000000.0;

    previousTime = currentTime;

    float correctedZ =
      g.gyro.z - gyroZBias;

    float angularVelocity =
      correctedZ * 57.2958;

    angleZ += angularVelocity * dt;

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, obstacleTurnSpeed);
    analogWrite(ENB, obstacleTurnSpeed);

    delay(5);
  }

  stopMotors();

  Serial.print("CW angle: ");
  Serial.println(angleZ, 2);

  delay(500);
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);

  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration =
    pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  return duration * 0.0343 / 2.0;
}

void driveForward(int speedValue) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);
}

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, rightSpeed);
  analogWrite(ENB, leftSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}