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

#define followDistance 10

#define frontWallDistance  6
#define sideWallDistance  12

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

#define SPEED 60

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
    initializeSD();
    waitForStart();
    enterMaze();
    //initializeCounter();
  } else {
    Make_A_Map();
  }
}

void loop() {
  if(program == 1) {
    rotationalMap();
    //RightWallFollow();
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
  //Serial.print("MPU6050 status: ");
  //Serial.println(status);
  while(status!=0) {}
  //Serial.println("Calculating offsets, do not move the MPU6050");
  mpu.calcOffsets();
  //Serial.println("Done!\n");
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


float scanGyro() {
  mpu.update();
  angle = mpu.getAngleZ();
  if(angle < 0) {
    angle = 360 + angle;
  } 
  if(angle > 360) {
    angle  = fmod(angle,360);
  }
  return angle;
}

void scan() {
  angle=scanGyro();
  //Serial.print(angle);
  //Serial.print(',');

  left = LeftSensor.readRangeContinuousMillimeters()*0.1;
  //Serial.print(left);
  //Serial.print(',');
  center = CenterSensor.readRangeContinuousMillimeters()*0.1;
  //Serial.print(center);
  //Serial.print(',');
  right = RightSensor.readRangeContinuousMillimeters()*0.1;
  //Serial.println(right);
  //Serial.print(',');
  
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

void Motor(int command, int leftspeed, int rightspeed) {
  switch(command) {
    case 1:
    digitalWrite(STBY, HIGH);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWML, leftspeed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMR, rightspeed);
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
  while(center>4) {
    scan();
    Motor(FWD, 60, 61);
  }
  Motor(STOP, 0, 0);
  //Now we can initialize the grid and start point
  scan();
  leftStart = left;
  rightStart = right;
  centerStart = center;
  turn(86, LEFT_TURN);
}

void RightWallFollow() {
   scan();
   if (right >= 20) {
    Motor(FWD, 60, 61);
    if(center < 25) {
      while(center > 4) {
         Motor(FWD, 60, 61);
         scan();
      }
    } else {
        delay(1000);
    }
    turn(80, RIGHT_TURN);
   } else if (center <= 5){
     turn(86, LEFT_TURN);
   } else if (right < 20) {
     computePD();
     Motor(FWD, leftSpeed, rightSpeed);
   }    
}
    
float currentAngle, targetAngle;

void turn(int turnAngle, int turnDirection) {
  if(turnDirection == 2) {
    targetAngle = scanGyro() + turnAngle;
  } else if(turnDirection==3) {
    targetAngle = scanGyro() - turnAngle;
  }
  if (targetAngle >= 360) {
      targetAngle = targetAngle - 360;
    }
  if(targetAngle <= 0) { 
      targetAngle = 360 + targetAngle;
    }
    
  // Continuously update angle and control motors
  while (abs(currentAngle - targetAngle) >= 4) {
    currentAngle = scanGyro();
    Motor(turnDirection, rotationSpeed, rotationSpeed);
  }
  Motor(STOP, 0, 0);  // Stop the motor once the turn is complete
}

float error, error_dt, integral, outputD;
float prev_error = 0;
float P = 1;
float D = 2;
float I = 0;


void computePD() {
    scan();
    if(right < 20) {
      error = right - 10;
      error_dt = error - prev_error;
      integral += error;
      outputD = P*error + D*error_dt + I*integral;
      leftSpeed = SPEED + outputD;
      rightSpeed = SPEED - outputD;
      prev_error = error;
    }
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
  Motor(LEFT_TURN, 0, 0);
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
