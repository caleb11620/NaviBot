#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>

// changes in the rotational speed and delay on the rightDistancewall follow
//Libraries
//issues is the gyro not calculating rightDistanceangle turns
#include <MPU6050_light.h>
#include "Wire.h"
#include <VL53L0X.h>
#include <SPI.h>



//IR Sensor Initialization
VL53L0X LeftSensor, CenterSensor, RightSensor;
#define LXSHUT 2
#define CXSHUT 3
#define RXSHUT 4
float leftDistance;
float rightDistance;
float center;
byte leftWall, centerWall, rightWall;

#define followDistance 10
#define frontWallDistance 6
#define sideWallDistance 12

//Gyro Initialization
MPU6050 mpu(Wire);
float angle;
#define RightAngleTurn 86

//SD initialization

float x = 199;
float y = 199;
const int chipSelect = A0;
File memoryLog;
File Map;
File temp;
int c = 10;


#define LEFT 1
#define CENTER 2
#define RIGHT 3
#define PI 3.14159265

//DIP Initialization
#define SW3 8
#define SW2 7
#define SW1 6

//Motor Initialization
#define PWML 10
#define AIN1 A7
#define AIN2 A6

#define PWMR 9
#define BIN1 A2
#define BIN2 A1

#define STBY A3

#define FWD 1
#define LEFT_TURN 2
#define RIGHT_TURN 3
#define STOP 4

#define SPEED 60

int leftSpeed = 59;
int rightSpeed = 61;
int rotationSpeed = 60;

//RGB Indicator Initialization
#define RED 14
#define GREEN 15
#define BLUE 16

//Core 0 Counter Initialization
TaskHandle_t Task1;
const int led1 = 2;
#define PULSE_INPUT_PIN D5
volatile unsigned int pulseCount = 0;
int program;

void setup() {
//  Serial.begin(115200);
//  while(!Serial);
//  Serial.println("Serial initialized");
  Wire.begin();
  initializeDIP();
  program = readDIP();
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  switch (program) {
    case 0:
      //Right Wall Follow Initialization
      initializeMotor();
      initializeGyro();
      initializeIR();
      initializeSD();
      waitForStart();
      enterMaze();
      initializeCounter();
      temp = SD.open("/temp.txt", FILE_WRITE);
      temp.close();
      break;

    case 1:
      //Left Wall Follow Initialization
      initializeMotor();
      initializeGyro();
      initializeIR();
      initializeSD();
      waitForStart();
      enterMaze();
      initializeCounter();
      temp = SD.open("/temp.txt", FILE_WRITE);
      temp.close();
      break;

    case 10:
      //Rotational Map Initialization
      initializeMotor();
      initializeGyro();
      initializeIR();
      initializeSD();
      waitForStart();
      analogWrite(RED, 100);
      analogWrite(GREEN, 256);
      analogWrite(BLUE, 256);
      digitalWrite(STBY, HIGH);
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
      analogWrite(PWML, 60);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMR, 60);
      memoryLog = SD.open("/Log.txt", FILE_WRITE);
      //Truncate Previous Map
      memoryLog.close();
      for (int i = 0; i < 83; i++) {
        RotationalMap();
      }
      Motor(STOP, 0, 0);
      analogWrite(RED, 256);
      analogWrite(GREEN, 100);
      analogWrite(BLUE, 256);
      break;

    case 11:
      initializeSD();
      analogWrite(RED, 100);
      analogWrite(GREEN, 256);
      analogWrite(BLUE, 256);
      Make_A_Map();
      analogWrite(RED, 256);
      analogWrite(GREEN, 100);
      analogWrite(BLUE, 256);
      break;

    case 111:
      initializeSD();
      //RED LED
      //A*
      //GREEN LED
      initializeMotor();
      initializeGyro();
      initializeIR(); 
      waitForStart();
      enterMaze();
      initializeCounter();
      break;

    default:
      break;
  }
}

void loop() {
  switch (program) {
    case 0:
      RightWallFollow();
      break;

    case 1:
      LeftWallFollow();
      break;

    case 10:
      break;

    case 11:
      break;

    case 111:
      for(int i = 0; i < vector.len(); i++) {
       //direction = get vector direection
       //degree = get vector degree change
       //turn(degree change, left/right)
       //distance = get vecor distance
       pulseCount = 0;
       //while pulseCount < distance: MotorFwd       
      }
      break;

    default:
      break;
  }
}

//-------------------Functions-------------------//
void initializeDIP() {
  pinMode(SW3, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW1, INPUT);
}

int readDIP() {
  int a = digitalRead(SW1);
  int b = digitalRead(SW2);
  int c = digitalRead(SW3);
  return ((c * 100) + (b * 10) + a);
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
  while (status != 0) {}
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
  if (!CenterSensor.init()) {
    //Serial.println("Failed to detect and initialize center sensor!");
    while (1) {}
  }
  CenterSensor.setAddress(0x20);

  digitalWrite(LXSHUT, HIGH);
  LeftSensor.setTimeout(500);
  if (!LeftSensor.init()) {
    //Serial.println("Failed to detect and initialize leftDistancesensor!");
    while (1) {}
  }
  LeftSensor.setAddress(0x22);

  digitalWrite(RXSHUT, HIGH);
  RightSensor.setTimeout(500);
  if (!RightSensor.init()) {
    //Serial.println("Failed to detect and initialize rightDistancesensor!");
    while (1) {}
  }
  RightSensor.setAddress(0x24);

  CenterSensor.startContinuous();
  LeftSensor.startContinuous();
  RightSensor.startContinuous();
  //Serial.println("IR Devices Ready");
}

void initializeSD() {
  if (!SD.begin(chipSelect)) {
    //Serial.println("Initialization Failure...Check wiring");
    while (1) {}
  }
  memoryLog = SD.open("/Log.txt", FILE_WRITE);
  memoryLog.close();
}

float scanGyro() {
  mpu.update();
  angle = mpu.getAngleZ();
  angle = fmod(angle, 360);
  if (angle < 0) {
    angle = 360 + angle;
  }
  return angle;
}

void scan() {
  angle = scanGyro();
  //Serial.print(angle);
  //Serial.print(',');

  leftDistance = LeftSensor.readRangeContinuousMillimeters() * 0.1;
  //Serial.print(left);
  //Serial.print(',');
  center = CenterSensor.readRangeContinuousMillimeters() * 0.1;
  //Serial.print(center);
  //Serial.print(',');
  rightDistance = RightSensor.readRangeContinuousMillimeters() * 0.1;
  //Serial.println(right);
  //Serial.print(',');
}

void waitForStart() {
  scan();
  while (center > 4) {
    scan();
  }
  int RGB[3];
  RGB[0] = 200;
  RGB[1] = 256;
  RGB[2] = 256;
  analogWrite(RED, RGB[0]);
  analogWrite(GREEN, RGB[1]);
  analogWrite(BLUE, RGB[2]);
  delay(1000);
  RGB[0] = 200;
  RGB[1] = 200;
  RGB[2] = 256;
  analogWrite(RED, RGB[0]);
  analogWrite(GREEN, RGB[1]);
  analogWrite(BLUE, RGB[2]);
  delay(1000);
  RGB[0] = 256;
  RGB[1] = 200;
  RGB[2] = 256;
  analogWrite(RED, RGB[0]);
  analogWrite(GREEN, RGB[1]);
  analogWrite(BLUE, RGB[2]);
  delay(1000);
}

void Motor(int command, int leftspeed, int rightspeed) {
  switch (command) {
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
      analogWrite(PWML, rotationSpeed-10);
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMR, rotationSpeed);
      break;

    case 3:
      digitalWrite(STBY, HIGH);
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
      analogWrite(PWML, rotationSpeed);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
      analogWrite(PWMR, rotationSpeed-10);
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

long location;
long calcLoc(float x_loc, float y_loc) {
  location = ceil(x_loc) + (ceil(y_loc) * 401);
  if (location < 0) {
    location = location * -1;
  }
  return location;
}

void initializeGrid() {
  Map = SD.open("/Map.bin", FILE_WRITE);
  if (Map) {
    for (int i = 0; i < 400; i++) {
      for (int j = 0; j < 400; j++) {
        Map.print('0');
      }
      Map.print('\n');
    }
    Map.seek(calcLoc(199, 199));
    Map.print('1');
  }
  Map.close();
}

void enterMaze() {
  scan();
  while (center > 4.75) {
    scan();
    Motor(FWD, 58, 61);
    MemoryLog(true);
  }
  Motor(STOP, 0, 0);
  //Now we can initialize the grid and start point
  scan();
  if (program == 0) {
    turn(86, LEFT_TURN);
  } else if (program == 1) {
    turn(86, RIGHT_TURN);
  }
  pulseCount = 0;
}

void RightWallFollow() {
  scan();
  if (center < 5) {
    turn(87, LEFT_TURN);
    pulseCount = 0;
  } else if (rightDistance >= 20) {
    Motor(FWD, 56, 61);
    if (center < 22) {
      while (center > 5) {
        Motor(FWD, 56, 61);
        scan();
        MemoryLog(true);
      }
    } else {
      delay(800);
      MemoryLog(true);
    }
    turn(85, RIGHT_TURN);
    pulseCount = 0;
  } else if (rightDistance < 13) {
    computePD();
    temp = SD.open("/temp.txt", FILE_APPEND);
    temp.println(rightDistance);
    temp.close();
    Motor(FWD, leftSpeed, rightSpeed);
    MemoryLog(true);
  }
}

void LeftWallFollow() {
  scan();
  if (center <= 5) {
    turn(85, RIGHT_TURN);
    pulseCount = 0;
  } else if (leftDistance >= 20) {
    Motor(FWD, 56, 61);
    if (center < 22) {
      while (center > 5) {
        Motor(FWD, 56, 61);
        scan();
        MemoryLog(true);
      }
    } else {
      delay(800);
      MemoryLog(true);
    }
    turn(87, LEFT_TURN);
    pulseCount = 0;
  } else if (leftDistance < 13) {
    computePD();
    temp = SD.open("/temp.txt", FILE_APPEND);
    temp.println(leftDistance);
    temp.close();
    Motor(FWD, leftSpeed, rightSpeed);
    MemoryLog(true);
  }
}

float currentAngle, targetAngle;

void turn(int turnAngle, int turnDirection) {
  if (turnDirection == 2) {
    targetAngle = scanGyro() + turnAngle;
  } else if (turnDirection == 3) {
    targetAngle = scanGyro() - turnAngle;
  }
  if (targetAngle >= 360) {
    targetAngle = targetAngle - 360;
  }
  if (targetAngle <= 0) {
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
float P = 2;
float D = 4;
float I = 0;

void computePD() {
  scan();
  if (program == 0) {
    if (rightDistance < 20) {
      error = rightDistance - 9;
      error_dt = error - prev_error;
      if(error_dt < 3) {
        integral += error;
        outputD = P * error + D * error_dt + I * integral;
        leftSpeed = SPEED + outputD;
        rightSpeed = SPEED - outputD;       
      }
      prev_error = error;
    }

  } else {
    if (leftDistance < 20) {
      error = leftDistance - 9;
      error_dt = error - prev_error;
      integral += error;
      outputD = P * error + D * error_dt + I * integral;
      leftSpeed = SPEED - outputD;
      rightSpeed = SPEED + outputD;
      prev_error = error;
    }
  }
}

void MemoryLog(bool includeEncoder) {
  memoryLog = SD.open("/Log.txt", FILE_APPEND);
  memoryLog.print(leftDistance);
  memoryLog.print(' ');
  memoryLog.print(center);
  memoryLog.print(' ');
  memoryLog.print(rightDistance);
  memoryLog.print(' ');
  memoryLog.print(angle);
  memoryLog.print(' ');
  if (includeEncoder == true) {
    memoryLog.print(pulseCount);
    memoryLog.print(' ');
    pulseCount = 0;
  } else if (includeEncoder == false) {
    memoryLog.print('0');
    memoryLog.print(' ');
  }
  memoryLog.close();
  //set counter to zero
}

void RotationalMap() {
  scan();
  MemoryLog(0);
}

float toRads(float Angle) {
  return (Angle / 180) * PI;
}

void Map_Update(int Lsensor, int Csensor, int Rsensor, float angle, int distance) {
  //  Map_Move(distance, angle);
  Map_IR(1, Lsensor, angle);
  Map_IR(2, Csensor, angle);
  Map_IR(3, Rsensor, angle);
}

void Map_Move(int distance, float angle) {
  for (int i = 0; i < distance; i++) {
    x = x + cos(toRads(angle));
    y = y + sin(toRads(angle));
    Map.seek(calcLoc(x, y));
    Map.print('1');
  }
}

void Map_IR(int sensor, int distance, float angle) {
  float prex = x;
  float prey = y;
  distance = distance + c;
  for (int i = 0; i < distance; i++) {
    x = x + cos(toRads(angle - (45 * (sensor - 2))));
    y = y + sin(toRads(angle - (45 * (sensor - 2))));
    if (x > 0 && x < 399) {
      if (y > 0 && y < 399) {
        Map.seek(calcLoc(x, y));
        Map.print('1');
      }
    }
  }
  x = prex;
  y = prey;
}

float MapData[5];
void Make_A_Map() {
  initializeGrid();
  memoryLog = SD.open("/Log.txt");
  String s;
  int i = 0;
  int loc = 0;
  while (s = memoryLog.readStringUntil(' ')) {
    loc = loc + s.length() + 1;
    MapData[i] = s.toFloat();
    i = i + 1;
    if (i == 5) {
      i = 0;
      memoryLog.close();
      Map = SD.open("/Map.bin", "r+");
      Map_Update(MapData[0], MapData[1], MapData[2], MapData[3], MapData[4]);
      Map.close();
      memoryLog = SD.open("/Log.txt");
      memoryLog.seek(loc);
    }
    if (MapData[0] == 0 && MapData[1] == 0 && MapData[2] == 0 && MapData[3] == 0 && MapData[4] == 0) {
      break;
    }
  }
  Map.close();
}

void initializeCounter() {
    attachInterrupt(digitalPinToInterrupt(PULSE_INPUT_PIN), pulse, FALLING);

//  xTaskCreatePinnedToCore(
//    Task1Code,
//    "Task 1",
//    10000,
//    NULL,
//    0,
//    &Task1,
//    0);
}


void Task1Code(void*) {
  attachInterrupt(digitalPinToInterrupt(PULSE_INPUT_PIN), pulse, FALLING);
  while(true) {
    mpu.update();
    delay(50);
    }
}

void pulse() {
  pulseCount++;
}
