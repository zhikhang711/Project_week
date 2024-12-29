#include <LiquidCrystal.h>
#include <stdio.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// Motor A connections
int enA = 3;
int in1 = 2;
int in2 = 10;
// Motor B connections
int enB = 11;
int in3 = 13;
int in4 = 12;
//ir sensor connections
int ir1 = A0;
int ir2 = A1;
int thresholdl = 150;
int thresholdr = 150;
//rotary encoder distance variables
float leftpulse = 0; 
float rightpulse = 0;
bool leftDetected = false;
bool rightDetected = false;
//rotary encoder pin
int re1 = A3;
int re2 = A2;


//make sure ramp only happens once
int i=0;

float distance;

bool countdownfinished = false;

int measureDistance(int rotaryl, int rotaryr) {//function to measure distance using rotary encoder

  if (rotaryl>300 && !leftDetected){
    leftpulse++;
    leftDetected = true;
  }
  else if (rotaryl <= 300){
    leftDetected = false;
  }
  if (rotaryr>300 && !rightDetected){
    rightpulse++;
    rightDetected = true;
  }
  else if (rotaryr <= 300) {
    rightDetected = false;
  }
  distance = (((leftpulse+rightpulse)/2.00)/20.00)*44.00;
  return distance;
}




int countdownTime = 80; // Countdown time in seconds 
unsigned long previousMillis = 0; 
const long interval = 1000; // Interval at which to decrease the countdown (1 second)

const int backlight = 10;

Adafruit_MPU6050 mpu;


void setup() {
  Serial.begin(9600);
  pinMode(backlight, OUTPUT);
  digitalWrite(backlight, HIGH); // Turn on backlight
  lcd.begin(16, 2);
  lcd.clear();
  //lcd.print("Countdown: 60");
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
  //set ir sensor pins
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);

  //set rotary sensor pins
  pinMode(re1,INPUT);
  pinMode(re2,INPUT);

  //initialise mpu 6050
  if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
      delay(10);
      }
  }

  // set accelerometer range to +-4G
	mpu.setAccelerometerRange(MPU6050_RANGE_4_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate roll and pitch
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  // Integrate gyroscope data to calculate yaw
  static float yaw = 0;
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  yaw += g.gyro.z * deltaTime;


  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Yaw: ");
  Serial.println(yaw);

  unsigned long currentMillis = millis(); 
  // Update countdown timer without blocking 
  if (currentMillis - previousMillis >= interval) { 
    previousMillis = currentMillis; 
    if (countdownTime > 0) { 
      countdownTime--; 
      lcd.setCursor(0, 0); // Set cursor to the beginning of the second row 
      lcd.print("Time:"); 
      lcd.print(countdownTime); 
      lcd.print("deg:");
      lcd.print(pitch);
      lcd.setCursor(0, 1);
      lcd.print("distance:");
      lcd.print(distance,2);
    }
    else if (!countdownfinished) { // After the countdown, stop the motors 
      analogWrite(enA, 0); 
      analogWrite(enB, 0); 
      digitalWrite(in1, LOW); 
      digitalWrite(in2, LOW); 
      digitalWrite(in3, LOW); 
      digitalWrite(in4, LOW); 
      lcd.clear(); 
      lcd.print("done "); 
      return; // Exit loop } 
    }
  }

  int rotaryl = analogRead(re1);// read encoder
  int rotaryr = analogRead(re2);
  int left = analogRead(ir1);// read ir
  int right = analogRead(ir2);
  distance = measureDistance(rotaryl,rotaryr) ;
  
    //car movement
  if (i==0 && left>thresholdl && right>thresholdr && pitch < -24){
    analogWrite(enA, 225);
    analogWrite(enB, 150);
    // go up ramp
    digitalWrite(in1, LOW);// left wheels
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);//right wheels
    digitalWrite(in4, LOW);
    delay(1800);
    //stop on top
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(4000);
    //turn 360
    analogWrite(enA, 230);
    analogWrite(enB, 230);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(2265);
    //stop 
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(2000);
    //go down ramp
    analogWrite(enA, 130);
    analogWrite(enB, 100);
    digitalWrite(in1, LOW);// left wheels
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);//right wheels
    digitalWrite(in4, LOW);
    delay(1300);
    //at flat after ramp
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(1000);
    //go 40cm
    Serial.print("start 40cm");

    
    rightpulse = 0;
    leftpulse = 0;
    Serial.print("reset dist");

    delay(1000);

    while (leftpulse < 15){
      lcd.setCursor(0, 1);
      lcd.print("distance: ");
      lcd.print(distance, 2);
      Serial.print("success");
      rotaryl = analogRead(re1);
      rotaryr = analogRead(re2);
      distance = measureDistance(rotaryl,rotaryr);
      int left = analogRead(ir1);// read ir
      int right = analogRead(ir2);
      if (left>thresholdl && right>thresholdr){//both white line
      // Set motors speed
      // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 100);
      analogWrite(enB, 70);
      
      // Turn on motor A and  B go straight
      digitalWrite(in1, LOW);// left wheels      
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);//right wheels
      digitalWrite(in4, LOW);
      
      Serial.print("the value is");
      Serial.print(left);
      Serial.print(" and ");
      Serial.print(right);
      Serial.print("\n");
      Serial.print("straight\n");
    }
    else if (left<thresholdl && right>thresholdr){// left black right white
    // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 180);
      analogWrite(enB, 150);
      // Turn on motor A and  B to go left
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    
      Serial.print("the value is");
      Serial.print(left);
      Serial.print(" and ");
      Serial.print(right);
      Serial.print("\n");
      Serial.print("left\n");
      }
    else if (left>thresholdl && right<thresholdr){//right turn
      // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 255);
      analogWrite(enB, 230);
    
      // Turn on motor A and  B to go left
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    
      Serial.print("the value is");
      Serial.print(left);
      Serial.print(" and ");
      Serial.print(right);
      Serial.print("\n");
      Serial.print("right\n");
      delay(180);
      }
    else if (left<=thresholdl && right<=thresholdr){//both black line
      // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 0);
      analogWrite(enB, 0);

      // Turn off motor A and B
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      
      Serial.print("the value is");
      Serial.print(left);
      Serial.print(" and ");
      Serial.print(right);
      Serial.print("\n");
      Serial.print("stop\n");
    }
    }
    
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    Serial.print("stopfor3sec");
      
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    Serial.print("stop at 40cm");
    delay(3000);

    i++;
    Serial.print("up\n");
    }
    
    else if (left>thresholdl && right>thresholdr){//both white line
      // Set motors speed
      // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 120);
      analogWrite(enB, 83);
      
      // Turn on motor A and  B go straight
      digitalWrite(in1, LOW);// left wheels
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);//right wheels
      digitalWrite(in4, LOW);
      
      Serial.print("the value is");
      Serial.print(left);
      Serial.print(" and ");
      Serial.print(right);
      Serial.print("\n");
      Serial.print("straight\n");
      
    }

    else if (left<thresholdl && right>thresholdr){// left black right white
    // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 180);
      analogWrite(enB, 150);
     

      // Turn on motor A and  B to go left
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
     
      Serial.print("the value is");
      Serial.print(left);
      Serial.print(" and ");
      Serial.print(right);
      Serial.print("\n");
      Serial.print("left\n");
      }


    else if (left>thresholdl && right<thresholdr){//right turn
      // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 255);
      analogWrite(enB, 230);
     
      // Turn on motor A and  B to go left
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
     
      Serial.print("the value is");
      Serial.print(left);
      Serial.print(" and ");
      Serial.print(right);
      Serial.print("\n");
      Serial.print("right\n");
      delay(180);
      
      }

    else if (left<=thresholdl && right<=thresholdr){//both black line
      // For PWM maximum possible values are 0 to 255
      analogWrite(enA, 0);
      analogWrite(enB, 0);

      // Turn off motor A and B
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      
      Serial.print("the value is");
      Serial.print(left);
      Serial.print(" and ");
      Serial.print(right);
      Serial.print("\n");
      Serial.print("stop\n");
      Serial.print(leftpulse);
      Serial.print("\n");
      Serial.print(rotaryl);
      Serial.print("\n");
      

      
      }
}


