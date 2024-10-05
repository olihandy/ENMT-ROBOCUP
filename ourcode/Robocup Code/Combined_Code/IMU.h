#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>

// Global variables for IMU data
// extern float CurrentPosX;                 // Current position in X-axis
// extern float CurrentPosY;                 // Current position in Y-axis
// extern int PreviousMove;                  // Previous movement direction (0 = left, 1 = right)
// extern int PreviousTurnTime;              // Previous turn time
// extern float XAccelerationList[50];       // List for X accelerations
// extern float YAccelerationList[50];       // List for Y accelerations
// extern float AverageAccelerationX;        // Average acceleration in X-axis
// extern float AverageAccelerationY;        // Average acceleration in Y-axis
// extern double PreviousTime;                // Previous time for calculations
// extern float CurrentOrientationX;         // Current orientation in Z-axis
// extern float AverageAngleX;                // Average angular Z
// extern float AngularAccelerationXList[50]; // List for angular accelerations in Z-axis
// extern double StartAngle;                  // Starting angle
// extern uint16_t BNO055_SAMPLERATE_DELAY_MS; // Sample rate delay

// // Sensor data structures
// extern sensors_event_t orientationData, linearAccelData;
// extern float ori[3];                     // Orientation data (x, y, z angles)
// extern float acc[3];                     // Acceleration data (x, y, z)

// Create an instance of the BNO055 sensor
extern Adafruit_BNO055 bno;                // Initialize sensor with I2C address 0x28

// Function prototypes
void setup_IMU(void);
void IMUGetPos(void);
void printEvent(sensors_event_t* event);
void MovingAverageFilter(void);
void IMU(void);
void IMUprint(void);

#endif // IMU_H
