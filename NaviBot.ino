//----------------------------LIBRARY--------------------
#include <Adafruit_MotorShield.h> //Library for motorshield
#include <Wire.h> //Library for I2C connection
#include <NewPing.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


//--------------------------GYRO----------------------------

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
//Default I2C (IIC) address is 0x68
//AD0 low = 0x68
//AD0 high = 0x69

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float angle_initial, angle_final, angle_goal;
float rawYaw, yaw, turnAngle;


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

//Boolean wall values, distance measurements, and echo pulse widths
bool left = false;
bool center = false;
bool right = false;

float SideThreshold = 10;
float FrontThreshold = 11;

float cmLeft, cmCenter, cmRight;

//---------------------MOTOR DRIVER---------------------

//Initialization of motorshie ld and recieving pointers for the motors in use
Adafruit_MotorShield Motors = Adafruit_MotorShield();
Adafruit_DCMotor *motorR = Motors.getMotor(1);
Adafruit_DCMotor *motorL = Motors.getMotor(3);

#define FWD 1
#define BACK 2
#define LEFT 3
#define RIGHT 4
#define STOP 5
#define UTURN 6

int rightSpeed = 60;
int leftSpeed = 58;

int rightTurn = 73;
int leftTurn = 82;

//-----------------------5 SEC COUNTER--------------------

#define ledPin LED_BUILTIN


//-------------------------------PID------------------------------

const float Kp = 5;
const float Ki = 0;
const float Kd = 4;

const float setpoint = 3;

float error;
float derivative;
float prev_error = 0;
float integral = 0;
float output;

int NavMethod = 3;

void setup(){
  //Serial communication for debugging
  Serial.begin(9600);
  Serial.print("NaviBot Test");

  //----------------GYRO INITIALIZATION----------------

  #if I2CDEV-IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    FastWire::setup(400, true);
  #endif

  //Initialize device
  Serial.println(F("Initializiing I2C devices..."));
  mpu.initialize();

  //Verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful"):  F("MPU6050 connection failed"));

  //Load and configure DMP
  devStatus = mpu.dmpInitialize();

  //Supply your own gyro offsets here
  // mpu.setXGyroOffset(220);
  //mpu.setYGyroOffset(76);
  //mpu.setZGyroOffset(-85);

  if (devStatus == 0) {
    //Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    //turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    //set our DMP Ready flag so the main loop() knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    //get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    //ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //if programming failed, do nothing
  if(!dmpReady) return;

  //-----------------------MOTOR INITIALIZATION----------------
  
  //Verifies that motors are connected or breaks
  if (!Motors.begin()) {
    //Serial.println("Could not find motorshield, check wiring! While(1).");
    while(1);
  }
  //Successful connection
  //Serial.println("NaviBot motors connected");

  //Sets speed for motors and releases them (brings the terminal voltages low)
  motorL->setSpeed(leftSpeed);
  motorR->setSpeed(rightSpeed);
  motorL->run(RELEASE);
  motorR->run(RELEASE);
  //At this point, everything is initialized.

  //----------------------------PID INITIALIZATION----------------
  


  //---------------------5 SECOND COUNTOWN--------------------------
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  MotorCommand(STOP, 0);
  scan();
  while (cmCenter > 5 || NavMethod == 3) {
    scan();
    if (cmLeft < 4) {
      NavMethod = 0;
      digitalWrite(LED_BUILTIN, HIGH);
    } else if (cmRight < 4) {
      NavMethod = 1;
      digitalWrite(LED_BUILTIN, HIGH);
    } 
  }

  int count = 0;

  while (count < 4) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    count = count + 1;
  }

  //--------------------ENTER THE MAZE------------------------------
  
  scan(); 
  while (center == false) {
    scan();
    MotorCommand(FWD, 0);
    //Serial.println("ENTERING MAZE");
  }
  if (NavMethod == 0) {
       MotorCommand(LEFT, leftTurn);
  }
  if (NavMethod == 1) {
      MotorCommand(LEFT, leftTurn);
  }
}


void loop() {

  scan(); //Sends out a scan

  if (NavMethod == 0) {
    LeftWallFollow();
  } else {
    RightWallFollow();
  }
  delay(50);
}


void scan() {

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
  if (cmLeft <= SideThreshold) {
    left = true;
  } else {
    left = false;
  }

  if (cmCenter <=  FrontThreshold) {
    center = true;
  } else {
    center = false;
  }

  if (cmRight <= SideThreshold) {
    right = true;
  } else {
    right = false;
  }
  delay(50);
}


void MotorCommand(int n, int turnAngle) {
  // 1 - FORWARD
  // 2 - BACKWARD
  // 3 - LEFT
  // 4 - RIGHT
  // 5 - STOP
  // 6 - UTURN
  switch (n) {
    case 1:
    motorL->setSpeed(leftSpeed);
    motorR->setSpeed(rightSpeed); 
    motorL->run(FORWARD);
    motorR->run(BACKWARD);
    break;

    case 2:
    motorL->setSpeed(leftSpeed);
    motorR->setSpeed(rightSpeed);
    motorL->run(BACKWARD);
    motorR->run(FORWARD);
    break;

    case 3:
    motorL->run(RELEASE);
    motorR->run(BACKWARD);
    motorR->setSpeed(60);
    motorL->setSpeed(30);
    turn(-1*turnAngle);
    break;

    case 4:
    motorL->run(FORWARD);
    motorR->run(RELEASE);
    motorR->setSpeed(30);
    motorL->setSpeed(60);
    turn(turnAngle);
    break;

    case 5:
    motorL->run(RELEASE);
    motorR->run(RELEASE);
    break;

    case 6: 
    motorL->run(FORWARD);
    motorR->run(BACKWARD);
    motorL->setSpeed(35);
    motorR->setSpeed(35);
    turn(-1*turnAngle);
    break;

    default:
    break;
  }
}


void turn(float angle_turn) {
  angle_initial = mpuGetYaw();
  angle_final = angle_initial;
  angle_goal = angle_initial + angle_turn;
  if (angle_goal > 360) {
    angle_goal = angle_goal - 360;
  }
  if (angle_goal < 0) {
    angle_goal = angle_goal + 360;
  }
  while(angle_final > angle_goal+3 or angle_final < angle_goal-3) {
    angle_final = mpuGetYaw();
    Serial.print("Goal: ");
    Serial.print(angle_goal);
    Serial.print("\t");
    Serial.print("Current Angle: ");
    Serial.print(angle_final);
    Serial.println("Right");
    angle_final = mpuGetYaw();
  } 
}


float mpuGetYaw() {
  //read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    rawYaw = (ypr[0] * 180/M_PI);
    if (rawYaw < 0) {
      rawYaw = rawYaw * -1;
      yaw = (180 - rawYaw) + 180;
    } else {
      yaw = rawYaw;
    }
  }
  return yaw;
}


void LeftWallFollow() {
  if (left == false) {
    MotorCommand(FWD, 0);
    delay(55);
    MotorCommand(LEFT, leftTurn);
    MotorCommand(FWD, 0);
    delay(30);
    while (left == false) {
      scan();
      MotorCommand(FWD, 0);
    }
  } else if (center == true && right == false) {
    MotorCommand(RIGHT, rightTurn);
  } else if (cmCenter < 5 && right == true) {
    MotorCommand(UTURN, 180);
  } else {
    MotorCommand(FWD, 0);
    //PID Code for Parallel Travel
    computePID(cmLeft);
    if (output < 10) {
    motorL->setSpeed(leftSpeed + output);
    motorR->setSpeed(rightSpeed - output);
    }
  }
}


void RightWallFollow() {
  if (right == false) {
    delay(55);
    MotorCommand(RIGHT, rightTurn);
    MotorCommand(FWD, 0);
    delay(30);
    while (right == false) {
      MotorCommand(FWD, 0);
      scan();
    }
  } else if (center == true && left == false) {
    MotorCommand(LEFT, leftTurn);
  } else if (cmCenter < 7 && left == true) {
    MotorCommand(UTURN, 180);
  } else {
    MotorCommand(FWD, 0);
    computePID(cmRight);
    motorL->setSpeed(leftSpeed - output);
    motorR->setSpeed(rightSpeed + output);
  }
}


void computePID(int cmdist) {
  error = setpoint - cmdist;
  integral = integral + error;
  derivative = error - prev_error;

  output = Kp * error + Ki*integral + Kd * derivative;
  prev_error = error;
  Serial.println(output);
}
