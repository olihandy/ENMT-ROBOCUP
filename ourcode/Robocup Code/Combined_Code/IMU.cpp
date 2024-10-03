#include "IMU.h"
#include <Wire.h>  
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <stdio.h>  
#include <time.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <arduino.h>

// Global variables for IMU data
float CurrentPosX = 0;                 // Current position in X-axis
float CurrentPosY = 0;                 // Current position in Y-axis
int PreviousMove = 0;                // Previous movement direction (0 = left, 1 = right)
float PreviousTurnTime = 0;            // Previous turn time
static float XAccelerationList[50] = { 0 };   // List for X accelerations
static float YAccelerationList[50] = { 0 };   // List for Y accelerations
float AverageAccelerationX = 0;        // Average acceleration in X-axis
float AverageAccelerationY = 0;        // Average acceleration in Y-axis
double PreviousTime = 0.0;           // Previous time for calculations
float CurrentOrientationX = 0;          // Current orientation in Z-axis
float AverageAngleX = 0;                // Average angular Z
static float OrientationXList[50] = { 0 }; // List for angular accelerations in Z-axis
double StartAngle = 0;                // Starting angle
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; // Sample rate delay
sensors_event_t orientationData, linearAccelData;

float acc[2];
float ori;

// Create an instance of the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // Initialize sensor with I2C address 0x28


void setup_IMU(void) {
  Serial.println("Starting Orientation Sensor Test...");
  
  // Initialize the sensor
  if (!bno.begin()) {
    Serial.print("Error: No BNO055 detected. Check your wiring or I2C address!");
    while (1);  // Loop indefinitely if the sensor is not detected
  }

  Serial.println("IMU detected successfully!");
  delay(1000);  // Delay for stability
}


void printEvent(sensors_event_t* event) { //problems getting right event
  double x_ori = -1000000, x_acc = -1000000, y_acc = -1000000;
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x_ori = event->orientation.x;

    ori = x_ori;

  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x_acc = event->acceleration.x;
    y_acc = event->acceleration.y;

    acc[0] = x_acc;
    acc[1] = y_acc;

    Serial.print("\tAcceleration= ");
    Serial.print("\t");Serial.print(acc[0]);Serial.print("\t");Serial.print(acc[1]);Serial.print("\t");
  } else {
    Serial.print("Unk:");
  }
}


void IMUGetPos(void) {
  // Get orientation data in degrees
  sensors_event_t orientationData;
  sensors_event_t linearAccelData;

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); // Get orientation
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); // Get linear acceleration

  // Print the obtained data
  printEvent(&orientationData);
  printEvent(&linearAccelData);
}


void MovingAverageFilter(void) {
  int element1 = 0;
  int element2 = 0;
  int element3 = 0;

  int averagingelement1 = 0;
  int averagingelement2 = 0;
  int averagingelement3 = 0;


 for (element1; element1 < 50; element1++){                 //shifting the window of detected accelerations for X
    XAccelerationList[element1+1] = XAccelerationList[element1];
  }
  XAccelerationList[0] = acc[0];  //Getting data from x accelerations
  AverageAccelerationX = 0;
  for (averagingelement1; averagingelement1 < 50; averagingelement1++) {  //Averaging the list of detect x accelerations
    AverageAccelerationX += XAccelerationList[averagingelement1];
  }
  AverageAccelerationX /= 50; //50 is elements in moving average filter

  for (element2; element2 < 50; element2++) {  //shifting the window of detected accelerations for Y
    YAccelerationList[element2 + 1] = YAccelerationList[element2];
  }
  YAccelerationList[0] = acc[1]; //Getting data from y accelerations
  AverageAccelerationY = 0;
  for (averagingelement2; averagingelement2 < 50; averagingelement2++) {  //Averaging the list of detect y accelerations
    AverageAccelerationY += YAccelerationList[averagingelement2];
  }
  AverageAccelerationY /= 50;

  for (element3; element3 < 50; element3++) {  //shifting the window of detected accelerations for Z orientations
    OrientationXList[element3 + 1] = OrientationXList[element3];
  }
  OrientationXList[0] = ori; //Getting data
  AverageAngleX = 0;
  for (averagingelement3; averagingelement3 < 50; averagingelement3++) {  //Averaging the list of detect Z orientations
    AverageAngleX += OrientationXList[averagingelement3];
  }
  AverageAngleX /= 50;
}

// Main function to process IMU data
void IMU(void) {
  Serial.println("-----------------------------------------------------------------------");
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  IMUGetPos();                  // Retrieve position and orientation data
  // MovingAverageFilter();        // Apply moving average filter

  double timeDifference = (millis() - PreviousTime) / 1000.0; // Calculate time difference in seconds  

  // Update current positions using average acceleration and time
  Serial.println("");
  CurrentPosX += (acc[0]  * timeDifference * timeDifference); // Update X position
  Serial.print("X Position: ");
  Serial.println(CurrentPosX);

  CurrentPosY += (acc[1] * timeDifference * timeDifference); // Update Y position
  Serial.print("Y Position: ");
  Serial.println(CurrentPosY);

  Serial.print("Orientation X: ");
  Serial.println(ori);
  
  Serial.println("-----------------------------------------------------------------------");
  PreviousTime = millis();      // Update previous time


}

