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
  static float OrientationXList[50] = { 0 }; // List for angular accelerations in Z-axis
  static float OrientationZList[50] = { 0 };
  double StartAngle = 0;                // Starting angle
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; // Sample rate delay
  sensors_event_t orientationData;
  float AverageAngleX = 0;
  float AverageAngleZ = 0;
  float ori[3];
  float PreviousTime;
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
    double x_ori = -1000000, z_ori = -1000000;
    if (event->type == SENSOR_TYPE_ORIENTATION) {
      Serial.print("Orient:");
      x_ori = event->orientation.x;
      z_ori = event->orientation.y;
      ori[0] = x_ori;
      ori[2] = z_ori;
    } else {
      Serial.print("Unk:");
    }
  }


  void IMUGetOrientation(void) {
    // Get orientation data in degrees
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); // Get orientation
    // Print the obtained data
    printEvent(&orientationData);
  }


  // Main function to process IMU data
  void IMU(void) {
    Serial.println("-----------------------------------------------------------------------");

    IMUGetOrientation();                  // Retrieve position and orientation data

    PreviousTime = millis();      // Update previous time
    double timeDifference = ((millis() - PreviousTime) / 1000.0); // Calculate time difference in seconds  

    AverageAngleX = ori[0];
    AverageAngleZ = ori[2];
    // Update current positions using average acceleration and time
    Serial.print("Orientation X: ");
    Serial.println(AverageAngleX);
    Serial.print("Orientation Z: ");
    Serial.println(AverageAngleZ);
    
    Serial.println("-----------------------------------------------------------------------");


  }
