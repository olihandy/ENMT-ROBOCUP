#include <Wire.h>  
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35

// The number of sensors in your system.
const uint8_t sensorCountL1 = 3;  //sensorcount
const uint8_t sensorCountL0 = 4;  //sensorcount 

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL1[sensorCountL1] = {0, 1, 2};  //Only three needed for three sensors, xshut ports
const uint8_t xshutPinsL0[sensorCountL0] = {3, 4, 5, 6};  //Only two needed for two sensors, xshut ports
//Left 6, Right 7

SX1509 io;  // Create an SX1509 object to be used throughout
VL53L1X sensorsL1[sensorCountL1];
VL53L0X sensorsL0[sensorCountL0];

//IMU Related variables
int CurrentposX = 0;
int CurrentposY = 0;
int previousMove = 0; //0 for left, 1 for right
int PrevturnTime = 0;
int Xposlist[50] = { 0 };
int Yposlist[50] = { 0 };
int AverageAccelerationX = 0;
int AverageAccelerationY = 0;
int CurrentOrienZ = 0;
int AverageOrienZ = 0;
int OrienZlist[50] = { 0 };

//This means the pins, not entire ports which the cables connect to
const int InductionPin = 27; //Induction sensor pin to check
int InductionDetected = 0;
const int CollectPin = 15;

void setup() {
  //Induction sensor
  pinMode(InductionPin, INPUT);

  //TOF
  if (!io.begin(SX1509_ADDRESS)) {
    Serial.println("Failed to to talk to IO Expander for TOFs");
    while (1) {};
  }

  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C
  Wire1.begin();
  Wire1.setClock(400000);  // use 400 kHz I2C

  Serial.println("Setup Wires");

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCountL1; i++) {
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }
  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCountL0; i++) {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
  }

  for (uint8_t i = 0; i < sensorCountL0; i++) {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);

    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init()) {
      Serial.print("Failed to detect and initialize sensor L0 ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);

    sensorsL0[i].startContinuous(50);
  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);

    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);

      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);

    sensorsL1[i].startContinuous(50);
  }

  Serial.println("Configured TOFs");

  //Color sensor
  //digitalWrite(ColorOnPin,HIGH);

  //IMU
  //   Serial.println("Orientation Sensor Test"); Serial.println("");

  //   /* Initialise the sensor */
  //   if (!bno.begin())
  //   {
  //     /* There was a problem detecting the BNO055 ... check your connections */
  //     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //     while (1);
  //   }
  //   Serial.print("IMU detected");
  //  digitalWrite(IMUDetection,HIGH);
  //   delay(1000);
}

//int colorRead() {

//}

// double printEvent(sensors_event_t* event) { //problems getting right event
//   double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
//   if (event->type == SENSOR_TYPE_ORIENTATION) {
//     Serial.print("Orient:");
//     x = event->orientation.x;
//     y = event->orientation.y;
//     z = event->orientation.z;
//   } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//     Serial.print("Linear:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   } else {
//     Serial.print("Unk:");
//   }

//   Serial.print("\tx= ");
//   Serial.print(x);
//   Serial.print(" |\ty= ");
//   Serial.print(y);
//   Serial.print(" |\tz= ");
//   Serial.println(z);
//   Serial.print("\n");
//   double XYZList[3] = {x, y, z};
//   return *XYZList;
// }

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

// sensors_event_t orientationData , accelerometerData; //, angVelocityData , linearAccelData, magnetometerData,  gravityData;
// double ori[3] = {};
// double acc[3] = {}; //comment out if not using IMU

// void IMUGetPos(void) {
//   //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
//   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); //degrees
//   //bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); //rad/s
//   //bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); //m/s^2
//   //bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
//   bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER); //Gravity detected
//   //bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY); //Gravity preset

//   *ori = printEvent(&orientationData);
//   //printEvent(&angVelocityData);
//   //printEvent(&linearAccelData);
//   //printEvent(&magnetometerData);
//   *acc = printEvent(&accelerometerData);
//   //printEvent(&gravityData);

//   int8_t boardTemp = bno.getTemp();
//   Serial.println();
//   Serial.print(F("temperature: "));
//   Serial.println(boardTemp);

//   uint8_t system, gyro, accel, mag = 0;
//   bno.getCalibration(&system, &gyro, &accel, &mag);
//   Serial.println();
//   Serial.print("Calibration: Sys=");
//   Serial.print(system);
//   Serial.print(" Gyro=");
//   Serial.print(gyro);
//   Serial.print(" Accel=");
//   Serial.print(accel);
//   Serial.print(" Mag=");
//   Serial.println(mag);

//   Serial.println("--");

//   //delay(BNO055_SAMPLERATE_DELAY_MS);
// }

uint16_t TOFReads(void) {
  uint16_t TopRight = sensorsL1[0].read()/10;  //Long range TOF reads
  uint16_t TopLeft = sensorsL1[1].read()/10;
  uint16_t TopMiddle = sensorsL1[2].read()/10;

  uint16_t MiddleLeft = sensorsL0[0].readRangeContinuousMillimeters()/10;
  uint16_t MiddleRight = sensorsL0[1].readRangeContinuousMillimeters()/10;
  uint16_t BottomLeft = sensorsL0[2].readRangeContinuousMillimeters()/10;
  uint16_t BottomRight = sensorsL0[3].readRangeContinuousMillimeters()/10;
  return TopRight, TopLeft, TopMiddle, MiddleLeft, MiddleRight, BottomLeft, BottomRight; 
}

void loop() {
  // put your main code here, to run repeatedly:

}
