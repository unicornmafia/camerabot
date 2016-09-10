/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);
// You can also make another motor on port M2
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);

int MOTOR_MAX_SPEED = 150;



Servo pitchServo;
Servo yawServo;
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int yawSpeed=90;

void setup() {
  pitchServo.attach(9);  // attaches the servo on pin 9 to the servo object
  yawServo.attach(10);  // attaches the servo on pin 9 to the servo object

  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  rightMotor->setSpeed(0);
  rightMotor->run(RELEASE);
  leftMotor->setSpeed(0);
  leftMotor->run(RELEASE);

}

void loop() {
  moveYawServo();
  movePitchServo();
  moveMotors();
}

void moveYawServo(){
  yawSpeed=90;
  yawServo.write(yawSpeed);
  delay(1000);

  for (yawSpeed=90;yawSpeed<=92;yawSpeed++){
    yawServo.write(yawSpeed);
    delay(500);
  }

  for (yawSpeed=90;yawSpeed>=88;yawSpeed--){
    yawServo.write(yawSpeed);
    delay(500);
  }
  yawServo.write(90);
}

void movePitchServo(){
  for (pos = 0; pos <= 180; pos += 10) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    pitchServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 10) { // goes from 180 degrees to 0 degrees
    pitchServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  } 
  pitchServo.write(90); 
}


void moveMotors() {
  uint8_t i;
  
  Serial.print("tick");

  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  for (i=0; i<MOTOR_MAX_SPEED; i++) {
    leftMotor->setSpeed(i);
    rightMotor->setSpeed(i);  
    delay(10);
  }
  for (i=MOTOR_MAX_SPEED; i!=0; i--) {
    leftMotor->setSpeed(i);  
    rightMotor->setSpeed(i);  
    delay(10);
  }
  
  Serial.print("tock");

  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  for (i=0; i<MOTOR_MAX_SPEED; i++) {
    leftMotor->setSpeed(i);  
    rightMotor->setSpeed(i);  
    delay(10);
  }
  for (i=MOTOR_MAX_SPEED; i!=0; i--) {
    leftMotor->setSpeed(i);  
    rightMotor->setSpeed(i);  

    delay(10);
  }

  Serial.print("tech");
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

