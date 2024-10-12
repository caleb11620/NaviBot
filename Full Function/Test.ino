//Libraries
#include <MPU6050_light.h>
#include "Wire.h"
#include <VL53L0X.h>
#include <SPI.h>
#include <SD.h>

//IR Sensor Initialization
VL53L0X LeftSensor, CenterSensor, RightSensor;
#define LXSHUT 2
#define CXSHUT 3
#define RXSHUT 4
float left,center,right;
byte leftWall, centerWall, rightWall;
#define followDistance 13

//Gyro Initialization
MPU6050 mpu(Wire);
float angle;

//SD initialization 
float x, y;
#define CS A0
File memoryLog;
File Map;
float leftStart, centerStart, rightStart;


#define LEFT 1
#define CENTER 2
#define RIGHT 3
#define PI 3.14159265

//DIP Initialization
#define SW1 8
#define SW2 7
#define SW3 6

//Motor Initialization
#define PWML 10
#define AIN1 6
#define AIN2 7

#define PWMR 9
#define BIN1 A2
#define BIN2 A1

#define STBY A3

#define FWD 1
#define LEFT_TURN 2
#define RIGHT_TURN 3
#define STOP 4

float leftSpeed = 60;
float rightSpeed = 61;


void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(13, OUTPUT);

  initializeMotor();
  initializeGyro();
  initializeIR();
  initializeSD();
  waitForStart();
  enterMaze();
}

void loop() {
  //rotationalMap();
  RightWallFollow();
  //MemoryLog();
}

void initializeMotor() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
}

void initializeGyro() {
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  while(status!=0) {}
  Serial.println("Calculating offsets, do not move the MPU6050");
  mpu.calcOffsets();
  Serial.println("Done!\n");
}

void initializeIR() {
  pinMode(LXSHUT, OUTPUT);
  pinMode(CXSHUT, OUTPUT);
  pinMode(RXSHUT, OUTPUT);

  digitalWrite(LXSHUT, LOW);
  digitalWrite(CXSHUT, HIGH);
  digitalWrite(RXSHUT, LOW);
  CenterSensor.setTimeout(500);
  if(!CenterSensor.init()) {
    //Serial.println("Failed to detect and initialize center sensor!");
    while(1) {}
  }
  CenterSensor.setAddress(0x20);

  digitalWrite(LXSHUT, HIGH);
  LeftSensor.setTimeout(500);
  if(!LeftSensor.init()) {
    //Serial.println("Failed to detect and initialize left sensor!");
    while(1) {}
  }
  LeftSensor.setAddress(0x22);

  digitalWrite(RXSHUT, HIGH);
  RightSensor.setTimeout(500);
  if(!RightSensor.init()) {
    //Serial.println("Failed to detect and initialize right sensor!");
    while(1) {}  
  }
  RightSensor.setAddress(0x24);

  CenterSensor.startContinuous();
  LeftSensor.startContinuous();
  RightSensor.startContinuous();
  //Serial.println("IR Devices Ready");
}

void initializeSD() {
  if(!SD.begin(CS)) {
    //Serial.println("Initialization Failure...Check wiring");
    while(1) {}
  }
  //Serial.println("Initialization Complete");
  memoryLog = SD.open("Log.txt", FILE_WRITE);
}

void scan() {
  mpu.update();
  angle = mpu.getAngleZ();
  //Serial.print(angle);
  //Serial.print(',');

  left = (LeftSensor.readRangeContinuousMillimeters()*0.1);
  //Serial.print(left);
  //Serial.print(',');
  center = (CenterSensor.readRangeContinuousMillimeters()*0.1);
  //Serial.print(center);
  //Serial.print(',');
  right = (RightSensor.readRangeContinuousMillimeters()*0.1);
  //Serial.print(right);
  //Serial.print(',');

  if(left<20) {leftWall=1;} else {leftWall=0;}
  if(center<6) {centerWall=1;} else {centerWall=0;}
  if(right<20) {rightWall=1;} else {rightWall=0;}
  
}

void waitForStart() {
  scan();
  while(center>4) {scan();}
  for(int i = 0; i < 3; i++){
    digitalWrite(13, HIGH);
    delay(800);
    digitalWrite(13, LOW);
    delay(800);
  }
}

void Motor(int command) {
  switch(command) {
    case 1:
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWML, round(leftSpeed));
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMR, round(rightSpeed));
    break;

    case 2:
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWML, 50);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMR, 50);
    break;

    case 3:
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWML, 50);
    digitalWrite(BIN1, LOW );
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMR, 50);
    break;

    case 4:
    digitalWrite(STBY, LOW);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWML, 0);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMR, 0);
    break;

    default:
    break;
  }
}

int calcLoc(int x_loc, int y_loc) {
  return x_loc + (401*y_loc);
}

void initializeGrid() {
  if (SD.exists("map1.bin")) {
    SD.remove("map1.bin");
  }
  Map = SD.open("map1.bin", FILE_WRITE);
  
  if(Map) {
    for(int i = 0; i < 400; i++) {
      for(int j = 0; j < 400; j++) {
        Map.print('0');
      }
      Map.println();
    }
    Map.seek(calcLoc(199,199));
    Map.print('1');
    Map.close();
  } 
//  else {
//    //Error indication
//    digitalWrite(LED_BUILTIN, HIGH);
//    delay(1000);
//    digitalWrite(LED_BUILTIN, LOW);
//    delay(1000);
//  }
}

void enterMaze() {
  scan();
  while(center>5) {
    scan();
    Motor(FWD);
  }
  Motor(STOP);
  //Now we can initialize the grid and start point
  scan();
  leftStart = left;
  rightStart = right;
  centerStart = center;
  //initializeGrid();
  turn(86, LEFT_TURN);
}

void RightWallFollow() {
  scan();
  while (rightWall==0){
  //if(rightWall==0) {
      delay(1000);
      turn(86, RIGHT_TURN);
      Motor(FWD);
      delay(750);
      scan();
      if(rightWall==0) {
        turn(86, RIGHT_TURN);
      }
  } 
  if (centerWall == 1 && leftWall == 0) {
    delay(30);
    turn(86, LEFT_TURN);
  } else {
    scan();
    computePID();
    Motor(FWD);
  }
}

void turn(int turnAngle, int turnDirection) {
  mpu.update();
  int angleGoal;
  if(turnDirection == LEFT_TURN) {
       angleGoal = mpu.getAngleZ() + turnAngle;
  }
  if(turnDirection == RIGHT_TURN) {
       angleGoal = mpu.getAngleZ() - turnAngle;
  }

  if(angleGoal > 360) {
    angleGoal = angleGoal % 360;
  }
  //Serial.print(angleGoal);
  float currAngle = mpu.getAngleZ();
  while(abs(currAngle-angleGoal) > 3) {
    mpu.update();
    //Serial.println(currAngle-angleGoal);
    Motor(turnDirection);
    currAngle = mpu.getAngleZ();
  }
  Motor(STOP);
}

float lastError = 0;
float ErrorDiff;
void computePID() {
  float error = right - followDistance;
  ErrorDiff = error - lastError;
  leftSpeed = leftSpeed + (error*0.08);
  rightSpeed = rightSpeed - (error*0.08);
  lastError = error;
}

void MemoryLog() {
  memoryLog = SD.open("Log.txt", FILE_WRITE);
  memoryLog.print(angle);
  memoryLog.print(' ');
  memoryLog.print(left);
  memoryLog.print(' ');
  memoryLog.print(center);
  memoryLog.print(' ');
  memoryLog.print(right);
  memoryLog.print(' ');
  memoryLog.close();
}

void rotationalMap() {
  Motor(LEFT_TURN);
  scan();
  MemoryLog();
}
