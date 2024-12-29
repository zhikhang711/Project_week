#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <stdio.h>
#include <math.h>

#define TRIG_PIN A2
#define ECHO_PIN A3
#define MOTOR_LEFT_EN 3
#define MOTOR_LEFT_IN1 2
#define MOTOR_LEFT_IN2 10
#define MOTOR_RIGHT_EN 11
#define MOTOR_RIGHT_IN1 13
#define MOTOR_RIGHT_IN2 12

Adafruit_MPU6050 mpu;

float originalAngle;
const float correctionThreshold = 7.0; // Angle threshold to correct path

void setup() {
  originalAngle = getRollAngle();

  Serial.begin(9600);
  Wire.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(MOTOR_LEFT_EN, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_EN, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  originalAngle = getRollAngle();
}

void loop() {
  Serial.print("original:");
  Serial.print(originalAngle);
  Serial.print("\n");

  float distance = measureDistance();

  if (distance < 20.0) {
    avoidObstacle();
    

  } else {
    moveForward();

  }
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return (duration * 0.034) / 2.0;
}

float getRollAngle() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float roll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180 / PI;
  Serial.print(roll);
  Serial.print("\n");
  return roll;
}

void moveForward() {
  analogWrite(MOTOR_LEFT_EN, 100);
  analogWrite(MOTOR_RIGHT_EN, 90);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  Serial.print("straight\n");
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_EN, 0);
  analogWrite(MOTOR_RIGHT_EN, 0);
  Serial.print("stop\n");
}

void avoidObstacle() {
  stopMotors();
  delay(1000);

  turnRight();//move right
  delay(600);
  moveForward();
  delay(1200);

  turnLeft();//move back straight
  delay(500);
  moveForward();
  delay(2000);

  turnLeft();//move left
  delay(500);
  moveForward();
  delay(1500);
  
  stopMotors();
  delay(500);
  
  turnRight();
  delay(450);
}

void turnRight() {
  analogWrite(MOTOR_LEFT_EN, 180);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);

  analogWrite(MOTOR_RIGHT_EN, 160);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  Serial.print("right\n");
}

void turnLeft() {
  analogWrite(MOTOR_LEFT_EN, 180);
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);

  analogWrite(MOTOR_RIGHT_EN, 160);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  Serial.print("left\n");
}

void correctDirection() {
  float currentAngle = getRollAngle();
  float angleDifference = currentAngle - originalAngle;
  Serial.print(currentAngle);

  while (abs(angleDifference) > correctionThreshold) {
    float currentAngle = getRollAngle();
    if (angleDifference > 0) {
      Serial.print("Correcting left\n");
      turnLeft();
    } else  {
      Serial.print("Correcting right\n");
      turnRight();
    }
    float angleDifference = currentAngle - originalAngle;
  }
}


