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
    IMUGetOrientation();                  // Retrieve position and orientation data
  }

  void IMU_print(void) {
    Serial.print("Orientation X: ");
    Serial.println(ori[0]);
    Serial.print("Orientation Z: ");
    Serial.println(ori[2]);
  }
