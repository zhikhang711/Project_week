//Arduino Bluetooth Controlled Car//


#include <LiquidCrystal.h>
#include <stdio.h>
#include <math.h>

// Initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// Motor A connections
int enA = 3;
int in1 = 2;
int in2 = 1;
// Motor B connections
int enB = 11;
int in3 = 13;
int in4 = 12;



char command; 

void setup() {
  Serial.begin(9600);
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

void loop(){
  if(Serial.available() > 0){ 
    command = Serial.read(); 
    stop(); //initialize with motors stoped
    //Change pin mode only if new command is different from previous.   
    Serial.println(command);
    switch(command){
    case 'F':  
      forward();
      break;
    case 'B':  
       back();
      break;
    case 'L':  
      left();
      break;
    case 'R':
      right();
      break;
    case 'S':
      stop();
      break;
      
    }
  } 
}

void forward()
{
 analogWrite(enA, 190);
      analogWrite(enB, 70);

      // Turn on motor A and  B go straight
      digitalWrite(in1, LOW);// left wheels
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);//right wheels
      digitalWrite(in4, LOW);
}

void back()
{
 analogWrite(enA, 190);
      analogWrite(enB, 70);

      // Turn on motor A and  B go in reverse
      digitalWrite(in1, HIGH);// left wheels
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);//right wheels
      digitalWrite(in4, HIGH);
}

void left()
{
 analogWrite(enA, 180);
      analogWrite(enB, 150);
     

      // Turn on motor A and  B to go left
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
}

void right()
{
      analogWrite(enA, 250);
      analogWrite(enB, 190);
     
  
      // Turn on motor A and  B to go right
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
} 

void stop()
{
      analogWrite(enA, 0);
      analogWrite(enB, 0);

      // Turn off motor A and B
      
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
}