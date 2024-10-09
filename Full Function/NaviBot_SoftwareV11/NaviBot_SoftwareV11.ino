//----------------------------LIBRARY--------------------
#include <Adafruit_MotorShield.h> //Library for motorshield
#include <Wire.h> //Library for I2C connection
#include <NewPing.h>
#include "I2Cdev.h" 
//#include "MPU6050.h"
#include "math.h"
#include <MPU6050_light.h> 


//--------------------------GYRO----------------------------

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
 
#endif
//Default I2C (IIC) address is 0x68
//AD0 low = 0x68
//AD0 high = 0x69

MPU6050 mpu(Wire);
unsigned long timer = 0;
int target = 0; //tracking angles for turning

int16_t ax, ay, az;
int16_t gx, gy, gz;





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

float MAX_DIST = 200.0;
float SideThreshold = 10;
float FrontThreshold = 11; 
float cmLeft, cmCenter, cmRight;

//---------------------MOTOR DRIVER---------------------

//Initialization of motorshie ld and recieving pointers for the motors in use
Adafruit_MotorShield Motors = Adafruit_MotorShield();
Adafruit_DCMotor *motorR = Motors.getMotor(1);
Adafruit_DCMotor *motorL = Motors.getMotor(4);

#define FWD 1
#define BACK 2
#define LEFT 3
#define RIGHT 4
#define STOP 5
#define UTURN 6

int rightSpeed = 60;
int leftSpeed = 58;

int rightTurn = 81;
int leftTurn = 84;

//-----------------------5 SEC COUNTER--------------------

#define ledPin LED_BUILTIN


//-------------------------------PID------------------------------

const float Kp = 8;
const float Ki = 0;
const float Kd = 10;

const float setpoint = 2.6;

float error;
float derivative;
float prev_error = 0;
float integral = 0;
float output;

int NavMethod = 3;

void setup(){
  //Serial communication for debugging
  Serial.begin(115200);
  Wire.begin();
  //Serial.print("NaviBot Test");
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  //delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero 

  //----------------GYRO INITIALIZATION----------------
 /* #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    
  */

  
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
    MotorCommand(FWD, 0);
    scan();  
  }/*
  if (NavMethod == 0) {
    MotorCommand(RIGHT, rightTurn);
  } else {
    MotorCommand(LEFT, leftTurn);
  }*/
}


void loop() {
  getAngles();
  scan(); //Sends out a scan

  if (NavMethod == 0) {
    LeftWallFollow();
  } else {
    RightWallFollow();
  }
  delay(100);
}

float getAngles(){
  mpu.update();
  //if((millis()-timer)>2000){ // print data every 500ms
	//Serial.println(mpu.getAngleZ());
	//timer = millis();  
  }
  

void scan() {
  // Update cmLeft, cmCenter, and cmRight
  // If they read zero, they're actually the max distance the sensor can ping.
  cmLeft = leftSensor.ping_cm();
  if(cmLeft == 0) { cmLeft = MAX_DIST;}
  cmCenter = centerSensor.ping_cm();
  if(cmCenter == 0) { cmCenter = MAX_DIST;}
  cmRight = rightSensor.ping_cm();
  if(cmRight == 0) { cmRight = MAX_DIST;}

  // Update boolean wall values
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
  delay(50); //changed from 50
  //Serial.println(cmCenter);
  if(millis()-timer > 1000){
      Serial.print("Left: ");
      Serial.print(cmLeft);
      Serial.print("  Center: ");
      Serial.print(cmCenter);
      Serial.print("  Right: ");
      Serial.print(cmRight);
      Serial.println("");
      timer = millis();
      }
  if(cmCenter == 200){
      pinMode(4, OUTPUT);
      pinMode(5, OUTPUT);
      delay(10);
      pinMode(4, HIGH);
      pinMode(5, HIGH);
      pinMode(4, LOW);
      pinMode(5, LOW);
      delay(10);
      pinMode(4, INPUT);
      pinMode(5, INPUT);
      delay(10);  
  }         
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
    if(millis()-timer > 2000){
      Serial.println("Forward March");
      timer = millis();
      } 
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
    /*if(millis()-timer > 1000){
      Serial.println("Left Turn");
      timer = millis();
      }*/
    Serial.println("Left turn");   
    motorL->run(RELEASE);
    motorR->run(BACKWARD);
    motorR->setSpeed(50);
    motorL->setSpeed(20);
    turn(1*turnAngle);
    motorL->run(RELEASE);
    motorR->run(RELEASE);
    n = 1;
    break;

    case 4:
    Serial.println("Right Turn");
    motorL->run(FORWARD);
    motorR->run(RELEASE);
    motorL->setSpeed(50);
    motorR->setSpeed(20);
    turn(-1*turnAngle);
    motorL->run(RELEASE);
    motorR->run(RELEASE);
    break;

    case 5:
    motorL->run(RELEASE);
    motorR->run(RELEASE);
    break;

    case 6:
    if(millis()-timer > 1000){
      Serial.println("U-Turn");
      timer = millis();
      } 
    motorL->run(BACKWARD);
    motorR->run(BACKWARD);
    motorL->setSpeed(35);
    motorR->setSpeed(35);
    turn(-1*turnAngle);
    motorL->run(RELEASE);
    motorR->run(RELEASE);
    break;

    default:
    break;
  }
}
 
void turn(float angle_turn) {
  float angle_initial, angle_final, angle_goal;   
  angle_initial = mpu.getAngleZ(); 
  Serial.print("   angle initial: ");//print out initial goal
  Serial.print(angle_initial);//
  angle_final = angle_initial;   
  angle_goal = angle_initial + angle_turn; 
  Serial.print("   angle goal: ");//print out initial goal
  Serial.print(angle_goal);//
  while(angle_final >= angle_goal+4 || angle_final <= angle_goal-4) { //code never moves past this while loop???     
    if(millis()-timer > 1000){
      Serial.println("  angle final: ");//print angle final
      Serial.println(angle_final);//
      timer = millis();
      }
    angle_final = mpu.getAngleZ();    
    mpu.update();              
  } 
  Serial.println("Exiting While Loop");
}

void LeftWallFollow(){
  //Serial.println("LWF");
  if(center == true){ //If there is a wall in front
    if(right == true && left == true){
      Serial.println("Walls all around");
      MotorCommand(UTURN, 180);
      //scan();
    }
    else if(right == true && left == false){ //wall in front and to the right
      Serial.println("Wall in front and right");
      MotorCommand(LEFT, leftTurn);
      //scan();
    }
    else if (right == false && left == true){ //wall in front and to the left
      Serial.println("Wall in front and left");
      MotorCommand(RIGHT, rightTurn);
      //scan();
    }
    else{ //wall only in the front
      Serial.println("Wall Only in Front");
      MotorCommand(LEFT, leftTurn); //left turn
      //scan();
    }
  }
  else{ //no wall in front
    MotorCommand(FWD, 0); //move forward
    scan();
  }
}

void RightWallFollow(){ 
  //Serial.println("RWF"); 
  if(center == true){ //If there is a wall in front
    if(right == true && left == true){
      Serial.println("Walls all around");
      MotorCommand(UTURN, 180);
      scan();
    }
    else if(right == true && left == false){ //wall in front and to the right
      Serial.println("Wall front and right");
      MotorCommand(LEFT, leftTurn);
      scan();
    }
    else if (right == false && left == true){ //wall in front and to the left
      Serial.println("Wall in Front and left");
      MotorCommand(RIGHT, rightTurn);
      scan();
    }
    else{ //wall only in the front
      Serial.println("Wall Only in Front");
      MotorCommand(RIGHT, rightTurn); //left turn
      scan();
    }
  }
  else{ //no wall in front
    MotorCommand(FWD, 0); //move forward
    scan();
  }
}

void computePID(int cmdist) {
  error = setpoint - cmdist;
  integral = integral + error;
  derivative = error - prev_error;

  output = Kp * error + Ki*integral + Kd * derivative;
  prev_error = error;
//  Serial.println(output);
}
