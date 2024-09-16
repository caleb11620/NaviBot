// Gyro Library
#include <MPU6050_light.h>
#include "Wire.h"

// IR Sensor Library
#include <VL53L0X.h>
#include <Wire.h>

//SD Library
#include <SPI.h>
#include <SD.h>


//IR Sensor Initialization
VL53L0X LeftSensor, CenterSensor, RightSensor;
#define LXSHUT 2
#define CXSHUT 3
#define RXSHUT 4
int left,center,right;

//Gyro Initialization
MPU6050 mpu(Wire);
unsigned float angle;

//SD Initialization
float x, y, angle;
CS = A0;

#define LEFT 1
#define CENTER 2
#define RIGHT 3
#define PI 3.14159265


void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(LED_BUILTIN, OUTPUT);

  GyroSetup(); 

  IRSetup();

}

void loop() {
  //Gyro Data
  angle = mpu.getAngleZ;

  //IR Data
  left = LeftSensor.readRangeContinuousMillimeters();
  center = LeftSensor.readRangeContinuousMillimeters();
  right = LeftSensor.readRangeContinuousMillimeters();

  MemoryLog();
}

void GyroSetup() {
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){}
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  //delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelerometer
  Serial.println("Done!\n");
}

void IRSetup() {
    //IR Sensor Setup
  pinMode(LXSHUT, OUTPUT);
  pinMode(CXSHUT, OUTPUT);
  pinMode(RXSHUT, OUTPUT);

  digitalWrite(LXSHUT, HIGH);
  digitalWrite(CXSHUT, LOW);
  digitalWrite(RXSHUT, LOW);
  LeftSensor.setTimeout(500);
  if(!LeftSensor.init()){
    Serial.println("Failed to detect and initialize left sensor!");
    while(1){}
  }
  LeftSensor.setAddress(0x20);
  //-----------------Center sensor initialization----------------------
    digitalWrite(LXSHUT, HIGH);
    digitalWrite(CXSHUT, HIGH);
    digitalWrite(RXSHUT, LOW);
    CenterSensor.setTimeout(500);
    if(!CenterSensor.init()) {
        Serial.println("Failed to detect and initialize center sensor!");
        while(1){}
    }
    CenterSensor.setAddress(0x22);



  //-----------------Right sensor initialization----------------------
    digitalWrite(LXSHUT, HIGH);
    digitalWrite(CXSHUT, HIGH);
    digitalWrite(RXSHUT, HIGH);  
    RightSensor.setTimeout(500);
    if(!RightSensor.init()) {
        Serial.println("Failed to detect and initialize right sensor!");
        while(1){}
    }
    RightSensor.setAddress(0x24);


    LeftSensor.startContinuous();
    CenterSensor.startContinuous();
    RightSensor.startContinuous();
}

//This is what writes data to the txt file.
void MemoryLog() {
  memoryLog = SD.open("MemoryLog.txt", FILE_WRITE);
  memoryLog.print(angle);
  memoryLog.print(',');
  memoryLog.print(left);
  memoryLog.print(',');
  memoryLog.print(center);
  memoryLog.print(',');
  memoryLog.println(right);
}

//This can calculate the file location given x,y coordinates
int calcLoc*int xm int y) {
  return x + (401*y);
}

//Sets 400x400 grid to 0 and sets initial position to 199,199
void initializeGrid() {
  if (!SD.begin(CS)) {
    //Error indication
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
  File Map = SD.open("Map.bin", FILE_WRITE);
  if(Map) {
    for(int i = 0; i < 400; i++) {
      for(int j = 0; j < 400l j++) {
        Map.print('0');
      }
      Map.println();
    }
    Map.seek(calcLoc(199,199));
    Map.print('1');
    Map.close();
  } else {
    //Error indication
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
}

//Update cells with '1' given distance and orientation
void UpdateCells(int distance, int orientation) {
  float rads = (orientation / 180)*PI;

  for(int i = 0; i < distance; i++) {
    x = x + cos(rads);
    y = y + sin(rads);

    File Map = SD.open("Map.bin", FILE_WRITE);
    if(Map) {
      Map.seek(calcLoc(x,y));
      Map.print('1');
      Map.close();
    } else {
      //Error indication
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }
  }
}

//Given a wall distance, the sensor, and orientation, will update map
void WallDistance(int wall_distance, int sensor, float orientation) {
  int prev_x = x;
  int prev_y = y;

  switch(sensor) {
    case LEFT:
      int left_angle = orientation-90;
      UpdateCells(wall_distance, left_angle);
      break;

    case CENTER:
      UpdateCells(wall_distance, orientation);
      break;

    case RIGHT:
      int right_angle = orientation + 90;
      UpdateCells(wall_distance, right_angle);
      break;

    default:
      break;
    }
  x = prev_x;
  y = prev_y;
}

//Update Map using previous functions
void MapUpdate(int orientation, int leftSensor, int centerSensor, int rightSensor) {
  WallDistance(leftSensor, LEFT, orientation);
  WallDistance(centerSensor, CENTER, orientation);
  WallDistance(rightSensor, RIGHT, orientation);
}

//Populate Map with MemoryLog data. Run this after txt file has been written. ONLY need to be run ONCE
void populateMap() {
  memoryLog = SD.open("MemoryLog.txt", FILE_READ);
  float Angle;
  int Left, Center, Right;
  string Data;
  while(memoryLog.available()) {
    Data = memoryLog.readStringUntil(',');
    Data.trim();
    Angle = stof(Data);

    Data = memoryLog.readStringUntil(',');
    Data.trim();
    Left = stoi(Data);

    Data = memoryLog.readStringUntil(',');
    Data.trim();
    Center = stoi(Data);

    Data = memoryLog.readStringUntil('\n');
    Data.trim();
    Right = stof(Data);

    MapUpdate(Angle, Left, Center, Right);
  }
}
