

//----------------------------LIBRARY--------------------
#include <Adafruit_MotorShield.h> //Library for motorshield
#include <Wire.h> //Library for I2C connection
#include <NewPing.h> 
#include "I2Cdev.h"
#include <MPU6050_light.h>
#include <PID_v1.h>


//--------------------------GYRO----------------------------

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//Default I2C (IIC) address is 0x68
//AD0 low = 0x68
//AD0 high = 0x69

MPU6050 mpu(Wire);
int target = 0; //tracking angles for turning


//---------------------------ULTRASONIC SENSORS---------------------------
//Ultrasonic Sensor pin (you can add other peripheral pins here)
#define TRIG1_PIN 2
#define ECHO1_PIN 3
#define TRIG2_PIN 4
#define ECHO2_PIN 5
#define TRIG3_PIN 6
#define ECHO3_PIN 7

NewPing rightSensor(TRIG1_PIN, ECHO1_PIN, 200);
NewPing centerSensor(TRIG2_PIN, ECHO2_PIN, 200);
NewPing leftSensor(TRIG3_PIN, ECHO3_PIN, 200);

//Sets wall threshold and default motor speed values
int walldist = 15;

//Boolean wall values, distance measurements, and echo pulse widths
bool left = false;
bool center = false;
bool right = false;

float cmLeft, cmCenter, cmRight;

//---------------------MOTOR DRIVER---------------------

//Initialization of motorshield and recieving pointers for the motors in use
Adafruit_MotorShield Motors = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = Motors.getMotor(3);
Adafruit_DCMotor *motorR = Motors.getMotor(2);

#define FWD 1
#define BACK 2
#define LEFT 3
#define RIGHT 4
#define STOP 5

int rightSpeed = 50;
int leftSpeed = 54;


//-----------------------5 SEC COUNTER--------------------

#define switchPin 8
#define ledPin 5

bool switchState = false;
int counter = 0;

//-------------------------------PID------------------------------

double setpoint = 6; //Desired wall distance
double input,output;
double Kp = 4, Ki = 0, Kd = 1; //PID Parameters

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


//-----------------------------SETUP------------------------------------
void setup(){
  //Serial communication for debugging
  Serial.begin(9600);
  Serial.print("NaviBot Test");

  //----------------GYRO INITIALIZATION----------------

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(5000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");  

  //-----------------------MOTOR INITIALIZATION----------------
  
  //Verifies that motors are connected or breaks
  if (!Motors.begin()) {
    Serial.println("Could not find motorshield, check wiring! While(1).");
    while(1);
  }
  //Successful connection
  Serial.println("NaviBot motors connected");

  //Sets speed for motors and releases them (brings the terminal voltages low)
  motorL->setSpeed(leftSpeed);
  motorR->setSpeed(rightSpeed);
  motorL->run(RELEASE);
  motorR->run(RELEASE);
  //At this point, everything is initialized.

  //----------------------------PID INITIALIZATION----------------
  myPID.SetMode(AUTOMATIC);

  //------------------------------2 SECOND DELAY------------------
  //At this point, we could also write the code necessary for the bot to enter the maze
  delay(2000);


  //---------------------5 SECOND COUNTOWN--------------------------



  //--------------------ENTER THE MAZE------------------------------
  scan(); 
  /*
   * if (center == false) {
    int i = 0;
    while (i < 4) {
      MotorCommand(LEFT, 90);
      i = i + 1;
    }
  *}
  */
  while (center == false) {
    MotorCommand(FWD, 0);
    scan();
  }
  MotorCommand(RIGHT, 90);
}


void loop() {

  scan(); //Sends out a scan

  if (left == false) {
    delay(400);
    MotorCommand(LEFT, 90);
    while (left == false) {
      MotorCommand(FWD, 0);
      scan();
    }
  } else if (center == true) {
    MotorCommand(RIGHT, 90);
  } else {
    MotorCommand(FWD, 0);
    //PID Code for Parallel Travel
    if (left == true) {
      input = cmLeft;
      myPID.Compute();
      motorL->setSpeed(leftSpeed + output);
      motorR->setSpeed(rightSpeed+6);
    }
  }

  //So each time this loops, we will be sending a command to the motor system. It
  //will either turn left, turn right, or go forward. Left wall follow.
  //Serial monitor for reading sensor measurements. 
  //Serial.print(cmLeft);
  //Serial.print(' ');
  //Serial.print(cmCenter);
  //Serial.print(' ');
  //Serial.print(cmCenter);
  //Serial.print(' ');
  //Serial.println();

  delay(100);
}

void scan() {
  //Matthew,can you help me maybe make this into a class
  //So each of the sensors have to be given a command and then the pulse response has to be read

  float pingLeft, pingCenter, pingRight;
  
  pingLeft = leftSensor.ping();
  pingCenter = centerSensor.ping();
  pingRight = rightSensor.ping();
  
  cmLeft = leftSensor.convert_cm(pingLeft);
  if(cmLeft == 0) { cmLeft = 200;}
  cmCenter = centerSensor.convert_cm(pingCenter);
  if(cmCenter == 0) { cmCenter = 200;}
  cmRight = rightSensor.convert_cm(pingRight);
  if(cmRight == 0) { cmRight = 200;}
  //Gets boolean value for navigation function
  if (cmLeft < 10) {
    left = true;
  } else {
    left = false;
  }

  if (cmCenter < 8) {
    center = true;
  } else {
    center = false;
  }

  if (cmRight < 10) {
    right = true;
  } else {
    right = false;
  }
  delay(50);
}

// Edna, here is the motor command functions
// I have so far. Now, when you get the gyro in,
// we're gonna insert it into the switch statements for left and right turns
// It can even call to another function that tracks the angle if you want.
void MotorCommand(int n, int turnAngle) {
  // 1 - FORWARD
  // 2 - BACKWARD
  // 3 - RIGHT
  // 4 - LEFT
  // 5 - STOP
  switch (n) {
    case 1: 
    motorL->run(FORWARD);
    motorR->run(FORWARD);
    motorL->setSpeed(leftSpeed);
    motorR->setSpeed(rightSpeed);
    break;

    case 2:
    motorL->run(BACKWARD);
    motorR->run(BACKWARD);
    motorL->setSpeed(leftSpeed);
    motorR->setSpeed(rightSpeed);
    break;

    case 3:
    motorL->run(BACKWARD);
    motorR->run(FORWARD);
    motorL->setSpeed(35);
    motorR->setSpeed(35);
    turn(-1*turnAngle);
    motorL->fullOff();
    motorR->fullOff();
    break;

    case 4:
    motorL->run(FORWARD);
    motorR->run(BACKWARD);
    motorL->setSpeed(35);
    motorR->setSpeed(35);
    turn(turnAngle);
    motorL->fullOff();
    motorR->fullOff();
    break;

    case 5:
    motorL->run(RELEASE);
    motorR->run(RELEASE);
    motorL->fullOff();
    motorR->fullOff();
    break;

    default:
    break;
  }
}

int leftWallFollow(bool left, bool center, bool right) {
  // Always turn left if there is no wall to the left.
  if (left == false) {
    return LEFT;
  }
  // If we enter this else if this means there we couldn't turn left and a wall is in front
  // of us. If there is another wall on the 'right'
  // it will be detected as in front of us, and we will thus turn right again.
  // This could potentially be expanded to first detect if there is a wall to the right, then do
  // a 180 degree turn rather than two right 90 degree turns.
  else if (center == true) {
    return RIGHT;
  }
  // Always going forward under the assumption we have oriented in a direction
  // with no wall in front of us.
  return FWD;
}

int rightWallFollow(bool left, bool center, bool right) {
  if (right == false) {
    return RIGHT;
  }
  else if (center == true) {
    return LEFT;
  }
  return FWD;
}

void getAngles(){

  mpu.update();
  if((millis()-timer)>500){ // print data every 500ms
	Serial.println(mpu.getAngleZ());
	timer = millis();  
  }
}

void L_angle(int target){

  Serial.println(mpu.getAngleZ());
  Serial.println("moving LEFT");
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  leftMotor->setSpeed(80);
  rightMotor->setSpeed(80);

  if (mpu.getAngleZ() >= target){
    Serial.print("stop");
    leftMotor->run(RELEASE); //stops the Motors
    rightMotor->run(RELEASE);
    leftMotor->fullOff();
    rightMotor->fullOff();
  }
   mpu.update(); 
}

void R_angle(int target){
    Serial.println(mpu.getAngleZ());
    Serial.println("moving left");
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(70);
    rightMotor->setSpeed(70);

  
  if (mpu.getAngleZ() <= target){
      Serial.print("stop");
      leftMotor->run(RELEASE); //stops the Motors
      rightMotor->run(RELEASE);
      leftMotor->fullOff();
      rightMotor->fullOff();
    }
  mpu.update(); 
}

void drivestraight(int target){
  
  
  if (mpu.getAngleZ() >= target){
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    leftMotor->setSpeed(70);
    rightMotor->setSpeed(70);
  }
}


