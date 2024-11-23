#include "debug_macros.h"
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

// ASTAR
#include "Astar.h"
#include "Bitmap.h"
#include <tuple>

struct outcome {
  Turn turn;
  int angle;
  int distance = 1;
};

std::vector<outcome> solution;

//IR Sensor Initialization
VL53L0X LeftSensor, CenterSensor, RightSensor;
#define LXSHUT 2
#define CXSHUT 3
#define RXSHUT 4
float leftDistance;
float rightDistance;
float centerDistance;
byte leftWall, centerWall, rightWall;

#define followDistance 9
#define frontThreshold 4.5

//Gyro Initialization
MPU6050 mpu(Wire);
float angle;

//SD initialization

float x = 199;
float y = 199;
const int chipSelect = A0;
File memoryLog;
File Map;
File MapFiltered;


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

#define RIGHT_TURN_ANGLE 83
#define LEFT_TURN_ANGLE 85

#define TURN_DELAY 775

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

// Function Prototypes
void initializeDIP();
int readDIP();
void initializeMotor();
void initializeGyro();
void initializeIR();
void initializeSD();
void waitForStart();
void enterMaze();
void initializeCounter();
void RotationalMap();
void Make_A_Map();
void Motor(int command, int leftspeed, int rightspeed);
void RightWallFollow();
void LeftWallFollow();

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Serial initialized");
  #endif
  Wire.begin();
  initializeDIP();
  program = readDIP();
  DEBUG_PRINTF("readDIP: %d", program);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  switch (program) {
    case 0:
    {
      //Right Wall Follow Initialization
      initializeMotor();
      initializeGyro();
      initializeIR();
      initializeSD();
      memoryLog = SD.open("/Log.txt", FILE_WRITE);
      memoryLog.close();
      waitForStart();
      enterMaze();
      initializeCounter();
      break;
    }
    case 1:
    {
      //Left Wall Follow Initialization
      initializeMotor();
      initializeGyro();
      initializeIR();
      initializeSD();
      memoryLog = SD.open("/Log.txt", FILE_WRITE);
      memoryLog.close();
      waitForStart();
      enterMaze();
      initializeCounter();
      break;
    }
    case 10:
    {
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
    }
    case 11:
    {
      initializeSD();
      analogWrite(RED, 100);
      analogWrite(GREEN, 256);
      analogWrite(BLUE, 256);
      Make_A_Map();
      analogWrite(RED, 256);
      analogWrite(GREEN, 100);
      analogWrite(BLUE, 256);
      break;
    }
    case 111: // ASTAR
     {
      initializeSD();
      DEBUG_PRINTLN("Init SD passed");
      analogWrite(RED, 100);
      analogWrite(GREEN, 256);
      analogWrite(BLUE, 256);
      printMemoryStats();
      Bitmap bmp;
      Astar astar;
      DEBUG_PRINTLN("1");
      Map = SD.open("/Map.bin", FILE_READ);
      bmp.read(Map);
      Map.close();
      DEBUG_PRINTLN("2: bmp.removeEmptyRowsAndColumns");
      bmp.removeEmptyRowsAndColumns();
      DEBUG_PRINTLN("3: astardata = bmp.getData");
      auto astardata = bmp.getData();
      DEBUG_PRINTLN("4: call astar.interpretBitmap");
      printMemoryStats();
      astar.interpretBitmap(astardata);
      DEBUG_PRINTLN("a* interpret data");
      printMemoryStats();

      Node* start = astar.determineStartNode();
      Node* Goal = astar.determineGoalNode();

      DEBUG_PRINTLN("map determined start and goal node");
      DEBUG_PRINTF("start (x,y): %d %d\n", start->x, start->y);
      DEBUG_PRINTF("end (x,y): %d %d\n", Goal->x, Goal->y);


      // HEURISTIC TYPES: MANHATTAN, EUCLIDEAN, CHEBYSHEV, OCTILE,
      // MANHATTAN: ONLY 90 DEGREE TURNS
      // EUCLIDEAN, CHEBYSHEV, OCTILE: REQUIRES 45 DEGREE TURNS
      astar.setHeuristicType(HeuristicType::MANHATTAN);
      DEBUG_PRINTLN("set Heuristic Manhattan");

      std::vector<Node*> path = astar.algorithm(start, Goal);
      DEBUG_PRINTLN("path returned");
      DEBUG_PRINTF("path size: %d\n", path.size());
      DEBUG_PRINTF("path[0]: (x,y): %d,%d\n",path[0]->x,path[0]->y);
      DEBUG_PRINTF("path[1]: (x,y): %d,%d\n",path[1]->x,path[1]->y);
      printMemoryStats();
      Heading head = Heading::N; // assuming the bot begins facing 'N'
      Heading newHeading = head;
      DEBUG_PRINTLN("entering forloop");
      for (int i = 0; i < path.size()-1; ++i) {
        DEBUG_PRINTLN("be4 auto");
        auto [direction, angle, newHeading] = astar.calculateSolutionVars(path[i]->x, path[i]->y, path[i+1]->x, path[i+1]->y, head);
        head = newHeading;
        DEBUG_PRINTLN("be4 outcome");
        outcome val = {direction, angle, 1};
        DEBUG_PRINTF("outcome vals: direction: %c \t angle: %d \n", static_cast<char>(val.turn), val.angle);
        solution.push_back(val);
      }
      DEBUG_PRINTLN("solutionVars calculated");
      // when direction is unchanging combine & erase steps by adding 'distance'
      for (int i = 0; i < solution.size()-1; ++i) {
        if (solution[i].turn == Turn::FORWARD && solution[i].turn == solution[i+1].turn) {
          solution[i].distance++;
          solution.erase(solution.begin() + i+1);
          --i;
        }
      }
      DEBUG_PRINTLN("consecutive FORWARD vars concatenated");
      astar.cleanup();
      DEBUG_PRINTLN("PASS CLEANUP");
      analogWrite(RED, 256);
      analogWrite(GREEN, 100);
      analogWrite(BLUE, 256);
      initializeMotor();
      DEBUG_PRINTLN("PASS MOTOR");
      initializeGyro();
      DEBUG_PRINTLN("PASS GYRO");
      initializeIR(); 
      DEBUG_PRINTLN("PASS IR");
      waitForStart();
      DEBUG_PRINTLN("PASS WAIT");
      //enterMaze();
      DEBUG_PRINTLN("PASS ENTER");
      initializeCounter();
      DEBUG_PRINTLN("PASS COUNTER");
      break;
    }

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

    case 111: // ASTAR
    {
      for (int i = 0; i < solution.size()-1; i++) {
        // GET DIRECTION
        char runDirection = static_cast<char>(solution[i].turn);
        int runDirectionTemp;
        switch (runDirection) {
          case 'F':
            runDirectionTemp = FWD;
            break;
          case 'R':
            runDirectionTemp = RIGHT_TURN;
            break;
          case 'L':
            runDirectionTemp = LEFT_TURN;
            break;
          default:
            runDirectionTemp = FWD;
            break;
        }
        // GET ANGLE
        int runAngle = solution[i].angle;
        // GET DISTANCE
        int runDistance = solution[i].distance;
        DEBUG_PRINT(runAngle);
        DEBUG_PRINT(" ");
        DEBUG_PRINT(runDirectionTemp);
        DEBUG_PRINT(" ");
        DEBUG_PRINTLN(runDistance);

        if (runDirectionTemp != FWD) {
          if (runDirectionTemp == RIGHT_TURN) {
            turn(RIGHT_TURN_ANGLE, RIGHT_TURN);
          }
          if (runDirection == LEFT_TURN) {
            turn(LEFT_TURN_ANGLE, LEFT_TURN);
          }
        }

        pulseCount = 0;
        while (pulseCount < runDistance) {
          Motor(FWD, 56, 61);
        }
       }
      exit(0);
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
  centerDistance= CenterSensor.readRangeContinuousMillimeters() * 0.1;
  //Serial.print(center);
  //Serial.print(',');
  rightDistance = RightSensor.readRangeContinuousMillimeters() * 0.1;
  //Serial.println(right);
  //Serial.print(',');
}

void waitForStart() {
  scan();
  while (centerDistance> 4) {
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
        Map.print('1');
      }
      Map.print('\n');
    }
    Map.seek(calcLoc(49, 199));
    Map.print('0');
  }
  Map.close();
}

void enterMaze() {
  scan();
  while (centerDistance > frontThreshold) {
    scan();
    Motor(FWD, 58, 61);
    MemoryLog(true);
  }
  Motor(STOP, 0, 0);
  //Now we can initialize the grid and start point
  scan();
  if (program == 0) {
    turn(LEFT_TURN_ANGLE, LEFT_TURN);
  } else if (program == 1) {
    turn(RIGHT_TURN_ANGLE, RIGHT_TURN);
  }
  pulseCount = 0;
}

void RightWallFollow() {
  scan();
  if (centerDistance< frontThreshold) {
    turn(LEFT_TURN_ANGLE, LEFT_TURN);
    pulseCount = 0;
  } else if (rightDistance >= 20) {
    Motor(FWD, 56, 61);
    if (centerDistance< 22) {
      while (centerDistance> frontThreshold) {
        Motor(FWD, 56, 61);
        scan();
        MemoryLog(true);
      }
    } else {
      delay(TURN_DELAY);
      MemoryLog(true);
    }
    turn(RIGHT_TURN_ANGLE, RIGHT_TURN);
    pulseCount = 0;
  } else if (rightDistance < 13) {
    computePD();
    Motor(FWD, leftSpeed, rightSpeed);
    MemoryLog(true);
  }
}

void LeftWallFollow() {
  scan();
  if (centerDistance < frontThreshold) {
    turn(RIGHT_TURN_ANGLE, RIGHT_TURN);
    pulseCount = 0;
  } else if (leftDistance >= 20) {
    Motor(FWD, 56, 61);
    if (centerDistance < 22) {
      while (centerDistance > frontThreshold) {
        Motor(FWD, 56, 61);
        scan();
        MemoryLog(true);
      }
    } else {
      delay(TURN_DELAY);
      MemoryLog(true);
    }
    turn(LEFT_TURN_ANGLE, LEFT_TURN);
    pulseCount = 0;
  } else if (leftDistance < 13) {
    computePD();
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
      error = rightDistance - followDistance;
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
      error = leftDistance - followDistance;
      error_dt = error - prev_error;
      if(error_dt < 3) {
        integral += error;
        outputD = P * error + D * error_dt + I * integral;
        leftSpeed = SPEED - outputD;
        rightSpeed = SPEED + outputD;
      }
      prev_error = error;
    }
  }
}

void MemoryLog(bool includeEncoder) {
  memoryLog = SD.open("/Log.txt", FILE_APPEND);
  memoryLog.print(leftDistance);
  memoryLog.print(' ');
  memoryLog.print(centerDistance);
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

float angleCorrection(float angle) {
    int mult = round(angle / 90);
    angle = mult * 90;
    return angle;
}

void Map_Update(int Lsensor, int Csensor, int Rsensor, float angle, int distance) {
  angle = -1*angleCorrection(angle);
  if(Csensor > 4) {Csensor = 4;}
  if(Lsensor > 4) {Lsensor = 4;}
  if(Rsensor > 4) {Rsensor = 4;}
  //Map_IR(1, Lsensor, angle);
  //Map_IR(2, Csensor, angle);
  //Map_IR(3, Rsensor, angle);
  Map_Move(distance*2, angle);
}

void Map_Move(int distance, float angle) {
  for (int i = 0; i < distance; i++) {
    x = x + cos(toRads(angle));
    y = y + sin(toRads(angle));
    Map.seek(calcLoc(x, y));
    Map.print('0');
  }
}

void Map_IR(int sensor, int distance, float angle) {
  float prex = x;
  float prey = y;
  for (int i = 0; i < distance; i++) {
    x = x + cos(toRads(angle - (45 * (sensor - 2))));
    y = y + sin(toRads(angle - (45 * (sensor - 2))));
    if (x > 0 && x < 399) {
      if (y > 0 && y < 399) {
        Map.seek(calcLoc(x, y));
        Map.print('0');
      }
    }
  }
  x = prex;
  y = prey;
}

float MapData[5];
void Make_A_Map() {
  initializeGrid();
  int start_x = x;
  int start_y = y;
  memoryLog = SD.open("/Log.txt", FILE_READ);
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
  memoryLog.close();
  int end_x = x;
  int end_y = y;
  Map = SD.open("/Map.bin", "r+");
  Map.seek(calcLoc(start_x, start_y));
  Map.print('2');
  Map.seek(calcLoc(end_x, end_y));
  Map.print('3');
  Map.close();
}

void initializeCounter() {
    attachInterrupt(digitalPinToInterrupt(PULSE_INPUT_PIN), pulse, FALLING);
}

void pulse() {
  pulseCount++;
}

void printMemoryStats() {
  multi_heap_info_t heapInfo;
  heap_caps_get_info(&heapInfo, MALLOC_CAP_8BIT);

  DEBUG_PRINTLN("Memory Stats:");
  DEBUG_PRINTF("Total Allocated Size: %d\n", heapInfo.total_allocated_bytes);
  DEBUG_PRINTF("Total Free Size: %d\n", heapInfo.total_free_bytes);
  DEBUG_PRINTF("Largest Free Block: %d\n", heapInfo.largest_free_block);
  // checking for psram
  DEBUG_PRINTF("Free PSRAM: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}
