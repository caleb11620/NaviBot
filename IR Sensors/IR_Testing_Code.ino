#include <VL53L0X.h>
#include <Wire.h>

//-------------------Sensor Variables-------------------
VL53L0X LeftSensor, CenterSensor, RightSensor;
#define LXSHUT 2
#define CXSHUT 3
#define RXSHUT 4


void setup() {
    Serial.begin(9600);
    Wire.begin();

    pinMode(LXSHUT, OUTPUT);
    pinMode(CXSHUT, OUTPUT);
    pinMode(RXSHUT, OUTPUT);

  //-----------------Left sensor initialization----------------------
    digitalWrite(LXSHUT, HIGH);
    digitalWrite(CXSHUT, LOW);
    digitalWrite(RXSHUT, LOW);
    LeftSensor.setTimeout(500);
    if(!LeftSensor.init()) {
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

void loop() {
    Serial.print("Left Sensor: ");
    Serial.print(LeftSensor.readRangeContinuousMillimeters());
    if (LeftSensor.timeoutOccurred()) { Serial.print(" LEFT SENSOR TIMEOUT"); }

    Serial.print("Center Sensor: ");
    Serial.print(CenterSensor.readRangeContinuousMillimeters());
    if (CenterSensor.timeoutOccurred()) { Serial.print(" CENTER SENSOR TIMEOUT"); }

    Serial.print("Right Sensor: ");
    Serial.print(RightSensor.readRangeContinuousMillimeters());
    if (RightSensor.timeoutOccurred()) { Serial.print(" RIGHT SENSOR TIMEOUT"); }

    Serial.println();
}