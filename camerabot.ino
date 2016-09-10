/* 
 *  camerabot brain
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
int MOTOR_TURN_MAX_SPEED = 150;
int maxInput = 2000;

int minSpeed = 30;

int tCenter = 20;
int yCenter = 20;

int syCenter = -20;
int spCenter = -20;

int servoYawThreshold = 10;
int servoPitchThreshold = 10;;

Servo pitchServo;
Servo yawServo;

int throttlePin = 5;
int yawPin = 3;
int servoYawPin = 6;
int servoPitchPin = 11;
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
  
  pinMode(throttlePin, INPUT);
  pinMode(yawPin, INPUT);
  pinMode(servoYawPin, INPUT);
  pinMode(servoPitchPin, INPUT);
}

void loop() {
  //moveYawServo();
  //movePitchServo();
  //moveMotors();
  int rawThrottle = pulseIn(throttlePin, HIGH)-1500;
  int rawYaw = -1*(pulseIn(yawPin, HIGH)-1500);
  int rawServoYaw = pulseIn(servoYawPin, HIGH)-1000;
  int rawServoPitch = (pulseIn(servoPitchPin, HIGH)-1000);
  Serial.print(rawThrottle);
  Serial.print("\t");
  Serial.print(rawYaw);
  Serial.print("\t");
  Serial.print(rawServoYaw);
  Serial.print("\t");
  Serial.print(rawServoPitch);
  Serial.print("\t");
  
  

  float throttleVal = (float)(rawThrottle-tCenter)/500.0; // Read the pulse width of  
  Serial.print(throttleVal);
  Serial.print("\t");

  float yawVal = (float)(rawYaw-yCenter)/500.0; // each channel
  Serial.print(yawVal);
  Serial.print("\t");

  float servoYawVal = (float)(rawServoYaw-syCenter)/1000.0; // Read the pulse width of  
  Serial.print(servoYawVal);
  Serial.print("\t");

  float servoPitchVal = (float)(rawServoPitch-spCenter)/1000.0; // each channel
  Serial.print(servoPitchVal);
  Serial.print("\t");
  

  float lVal = throttleVal - yawVal;
  float rVal = throttleVal + yawVal;

  //Serial.print(lVal);
  //Serial.print("\t");
  //Serial.print(rVal);
  //Serial.print("\t");
  moveMotors(lVal, rVal);
  moveServoYaw(servoYawVal);
  moveServoPitch(servoPitchVal);
  Serial.println(" ");
  delay(15);
}

void moveServoYaw(float servoYawVal){
  //Serial.print(lVal);
  //Serial.print("\t");
  int servoPos = (int)(180.0*abs(servoYawVal)) ;  
  Serial.print(servoPos);
  Serial.print("\t");

  if (abs(servoPos-90) > servoYawThreshold){
    yawServo.write(servoPos); 
    
  }
}

void moveServoPitch(float servoPitchVal){
  int servoPos = (int)(180.0*abs(servoPitchVal)) ;  
  Serial.print(servoPos); 
  Serial.print("\t");

  if (abs(servoPos-90) > servoPitchThreshold){
    pitchServo.write(servoPos); 
  }
}

void moveMotors(float lSpeed, float rSpeed){
  lSpeed = lSpeed * MOTOR_MAX_SPEED;
  rSpeed = rSpeed * MOTOR_MAX_SPEED;

  //Serial.print(abs(lSpeed));
  //Serial.print("\t");
  //Serial.print(abs(rSpeed));
  //Serial.println("\t");

  if (abs(lSpeed)<minSpeed){
    leftMotor->run(RELEASE);
    leftMotor->setSpeed(0);
  } else if (lSpeed>0.0) {
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(abs(lSpeed));
  }else{
    leftMotor->run(BACKWARD);
    leftMotor->setSpeed(abs(lSpeed));
  }

  if (abs(rSpeed)<minSpeed){
    rightMotor->run(RELEASE);
    rightMotor->setSpeed(0);
  } else if (rSpeed>0.0) {
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(abs(rSpeed));
  }else{
    rightMotor->run(BACKWARD);
    rightMotor->setSpeed(abs(rSpeed));

  }

}

