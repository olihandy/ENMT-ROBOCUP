#include "IMU.h"

// Global variables for IMU data
float CurrentPosX = 0;                 // Current position in X-axis
float CurrentPosY = 0;                 // Current position in Y-axis
int PreviousMove = 0;                // Previous movement direction (0 = left, 1 = right)
int PreviousTurnTime = 0;            // Previous turn time
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
float ori[3];
float acc[3];

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
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;

    ori[0] = x;
    ori[1] = y;
    ori[2] = z;

  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    acc[0] = x;
    acc[1] = y;
    acc[2] = z;
    Serial.print("\tAcceleration= ");
    Serial.print("\t");Serial.print(acc[0]);Serial.print("\t");Serial.print(acc[1]);Serial.print("\t");Serial.print(acc[2]);
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
    Serial.println("");
    Serial.print("\t");Serial.print(acc[0]);Serial.print("\t");Serial.print(acc[1]);Serial.print("\t");Serial.print(acc[2]);

  // Shift X acceleration data
  for (uint16_t i = 50; i > 0; i--) {
    XAccelerationList[i] = XAccelerationList[i - 1];
  }
  XAccelerationList[0] = acc[0];  // Store new X acceleration
  // Calculate average for X
  AverageAccelerationX = 0;
  for (uint16_t i = 0; i < 50; i++) {
    AverageAccelerationX += XAccelerationList[i];

  }
  AverageAccelerationX /= 50; // Average X acceleration

  // Serial.println("Ave x acc");
  // Serial.println(AverageAccelerationX);
  
  
  
  // Shift Y acceleration data
  for (uint16_t i = 50; i > 0; i--) {
    YAccelerationList[i] = YAccelerationList[i - 1];
  }
  YAccelerationList[0] = acc[1]; // Store new Y acceleration
  // Calculate average for Y
  AverageAccelerationY = 0;
  for (uint16_t i = 0; i < 50; i++) {
    AverageAccelerationY += YAccelerationList[i];
  }
  AverageAccelerationY /= 50; // Average Y acceleration
  // Serial.println("Ave y acc");
  // Serial.println(AverageAccelerationY);



  // Shift angular acceleration data
  for (uint16_t i = 50; i > 0; i--) {
    OrientationXList[i] = OrientationXList[i - 1];
  }
  OrientationXList[0] = ori[0]; // Store new Z orientation
  // Calculate average for Z orientation
  AverageAngleX = 0;
  for (uint16_t i = 0; i < 50; i++) {
    AverageAngleX += OrientationXList[i];
  }
  AverageAngleX /= 50; // Average Z orientation

}

// Main function to process IMU data
void IMU(void) {
  Serial.println("-----------------------------------------------------------------------");
  
  IMUGetPos();                  // Retrieve position and orientation data
  MovingAverageFilter();        // Apply moving average filter

  double timeDifference = (millis() - PreviousTime) / 1000.0; // Calculate time difference in seconds
  PreviousTime = millis();      // Update previous time

  // Update current positions using average acceleration and time
  Serial.println("");
  CurrentPosX += static_cast<int>(AverageAccelerationX * pow(timeDifference, 2)); // Update X position
  Serial.println(AverageAccelerationX);
  Serial.print("X Position: ");
  Serial.println(CurrentPosX);

  CurrentPosY += static_cast<int>(AverageAccelerationY * pow(timeDifference, 2)); // Update Y position
  Serial.println(AverageAccelerationY);
  Serial.print("Y Position: ");
  Serial.println(CurrentPosY);

  CurrentOrientationX = static_cast<int>(AverageAngleX); // Update Z orientation
  Serial.println(CurrentOrientationX);
  Serial.print("Orientation X: ");
  Serial.println(AverageAngleX);
  Serial.println("-----------------------------------------------------------------------");

}

