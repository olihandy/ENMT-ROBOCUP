#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <utility/imumaths.h>

// Create an instance of the BNO055 sensor
extern Adafruit_BNO055 bno;                // Initialize sensor with I2C address 0x28

// Function prototypes
void setup_IMU(void);
void IMUGetOrientation(void);
void printEvent(sensors_event_t* event);
void MovingAverageFilter(void);
void IMU(void);
void IMU_print(void);

#endif // IMU_H