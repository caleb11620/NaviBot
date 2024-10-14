//Libraries
//issues is the gyro not calculating right angle turns

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

#define followDistance 10

#define frontWallDistance  6
#define sideWallDistance  20

//Gyro Initialization
MPU6050 mpu(Wire);
float angle;
#define RightAngleTurn 86

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
#define SW3 8

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

int leftSpeed = 60;
int rightSpeed = 61;
int rotationSpeed = 45;

byte program; 

void setup() {
  Serial.begin(9600);
  Wire.begin();
  initializeDIP();
  program = readDIP();
  if(program==1) {
    initializeMotor();
    initializeGyro();
    initializeIR();
    //initializeSD();
    waitForStart();
    enterMaze();
    //initializeCounter();
  } else {
    Make_A_Map();
  }
}


void loop() {
  if(program == 1) {
    
    RightWallFollow();
    
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

//-------------------Functions-------------------//
void initializeDIP() {
  pinMode(SW3, INPUT);
}

byte readDIP() {
  return digitalRead(SW3);
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

  left = LeftSensor.readRangeContinuousMillimeters()*0.1;
  //Serial.print(left);
  //Serial.print(',');
  center = CenterSensor.readRangeContinuousMillimeters()*0.1;
  //Serial.print(center);
  //Serial.print(',');
  right = RightSensor.readRangeContinuousMillimeters()*0.1;
  //Serial.print(right);
  //Serial.print(',');
  
  if(left<sideWallDistance) {leftWall=1;} else {leftWall=0;}
  if(center<frontWallDistance) {centerWall=1;} else {centerWall=0;}
  if(right<sideWallDistance) {rightWall=1;} else {rightWall=0;}
  
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
    analogWrite(PWML, leftSpeed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMR, rightSpeed);
    break;

    case 2:
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWML, rotationSpeed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMR, rotationSpeed);
    break;

    case 3:
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWML, rotationSpeed);
    digitalWrite(BIN1, LOW );
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMR, rotationSpeed);
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
  if (SD.exists("Map.bin")) {
    SD.remove("Map.bin");
  }
  Map = SD.open("MAP.bin", FILE_WRITE);
  
  if(Map) {
    for(int i = 0; i < 400; i++) {
      for(int j = 0; j < 400; j++) {
        Map.print(0);
      }
      Map.println();
    }
    Map.seek(calcLoc(199,199));
    Map.print(1);
  } 
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
  turn(86, LEFT_TURN);
}

void RightWallFollow() {
  

  // while(true){
  //   scan();
  // if(centerWall == 1 && rightWall == 1){
  //   turn(86, LEFT_TURN);
    
  // }
  // if (centerWall == 0 && rightWall == 1){
  //   Motor(FWD);
    
  // }  
  if (centerWall == 1){
    turn(86, RIGHT_TURN);
    
   }
  //  if (rightWall == 0){
  
  //   delay(1000);
  //   //turn(90, RIGHT_TURN);
  //   Motor(FWD);
  //   delay(1000);
  //   Motor(STOP);
  //   delay(1000);
    
  //   }
}
    


void turn(int turnAngle, int turnDirection) {
  mpu.update();
  
  if (turnDirection == RIGHT_TURN) {
    turnAngle = -turnAngle;  // For right turn, angle will decrease
  }

  // Current angle from Z-axis
  float currentAngle = mpu.getAngleZ();
  float targetAngle = currentAngle + turnAngle;

  // Handle angle wrapping (e.g., from 359° to 0°)
  if (targetAngle > 360) {
    targetAngle = targetAngle - 360;
  } else if (targetAngle < 0) {
    targetAngle = 360 + targetAngle;
  }

  // Continuously update angle and control motors
  while (true) {
    mpu.update();
    currentAngle = mpu.getAngleZ();

    // Check if the angle is close enough to the target
    if (abs(currentAngle - targetAngle) <= 2) {
      Motor(STOP);
      break;
    }

    // Turn motor in the specified direction
    Motor(turnDirection);
    delay(10);  // Adjust delay for smoother control if needed
  }
  
  Motor(STOP);  // Stop the motor once the turn is complete


  // mpu.update();
  // if (turnDirection == RIGHT_TURN){
  //   turnAngle = turnAngle * -1;
  // }
  // int angleGoal = turnAngle + mpu.getAngleZ();

  // if(abs(angleGoal) > 360) {
  //   angleGoal = angleGoal % 360;
  // }

  // int currAngle = mpu.getAngleZ();
  // while(abs(currAngle-angleGoal) > 2 || abs(currAngle-angleGoal) < 2) { 
  //   mpu.update();
  //   currAngle = mpu.getAngleZ();
  //   Motor(turnDirection);
  // }
  // Motor(STOP);



}

float error, error_dt;
float prev_error = 0;
float K = 0.05;
float D = 0.1;

void computePD() {
  error = right - sideWallDistance;
  error_dt = error - prev_error;
  leftSpeed = leftSpeed + (K * error) + (D * error_dt);
  rightSpeed = rightSpeed - (K * error) - (D * error_dt);
  prev_error = error;
}

void MemoryLog(byte encoder) {
  memoryLog = SD.open("Log.txt", FILE_WRITE);
  memoryLog.print(left);
  memoryLog.print(' ');
  memoryLog.print(center);
  memoryLog.print(' ');
  memoryLog.print(right);
  memoryLog.print(' ');
  memoryLog.print(angle);
  memoryLog.print(' ');
  if(encoder == 1) {
    memoryLog.print(encoder);
    memoryLog.print(' ');
  } else {
    memoryLog.print('0');
    memoryLog.print(' ');
  }
  memoryLog.close();
  //set counter to zero
}

void rotationalMap() {
  Motor(LEFT_TURN);
  scan();
  MemoryLog(0);
}

float toRads(float Angle) {
  return (Angle/180)*PI;
}

float MapData[5];
void readLines() {
  memoryLog=SD.open("Log.txt", FILE_READ);
  String s;
  int i = 0;
  while(s=Map.readStringUntil(' ')) {
    MapData[i] = s.toFloat();
    i = i + 1;
    if (i==5) {
      i = 0;
      Map_Update(MapData[0], MapData[1], MapData[2], MapData[3], MapData[4]);
    }
  }
  memoryLog.close();
}

void Map_Update(int Lsensor, int Csensor, int Rsensor, float angle, int distance) {
  Map_Move(distance, angle);
  Map_IR(1,Lsensor, angle);
  Map_IR(2,Csensor, angle);
  Map_IR(3,Rsensor, angle);
}

void Map_Move(int distance, float angle) { 
  angle = angle * -1;
  for(int i = 0; i < distance; i++) {
    x = x + cos(toRads(angle));
    y = y + sin(toRads(angle));
    Map.seek(calcLoc(x,y));
    Map.print(1);
  }
}

void Map_IR(int sensor, int distance, float angle) {
  int prex = x;
  int prey = y;
  angle = angle * -1;
  for(int i = 0; i < distance;i++) {
    x = x + cos(toRads(angle+(45*(sensor - 2))));
    y = y + sin(toRads(angle+(45*(sensor - 2))));
    Map.seek(calcLoc(x,y));
    Map.print(1);
  }
  x = prex;
  y = prey;
}

void Make_A_Map() {
  initializeGrid();
  readLines();
  Map.close();
}

int count(byte zero) {
  if(zero == 1) {
    TCNT1 = 0;
  } else {
    int counter_value = TCNT1;
    TCNT1 = 0;
    return counter_value;
  }
}

void initializeCounter() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= B00000111;
  TCNT1 = 0;
  Serial.begin(9600);
}
