#ifndef PRINT_READINGS_H
#define PRINT_READINGS_H

#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Constants
extern const int ElectroMagnet1Pin;
extern const int ElectroMagnet2Pin;
extern const int ElectroMagnet3Pin;
extern const int FrontInductionPin;
extern const int BackInductionPin;
extern const int numElectroMagnets;
extern const int numInductiveSensors;
extern const uint8_t sensorCountL1;
extern const uint8_t sensorCountL0;
extern const uint8_t xshutPinsL1[];
extern const uint8_t xshutPinsL0[];
extern SX1509 io;
extern VL53L1X sensorsL1[];
extern VL53L0X sensorsL0[];

// Variable declarations
extern bool ElectroMagnet1On;
extern bool ElectroMagnet2On;
extern bool ElectroMagnet3On;
extern int elapsed_time;

// Function prototypes
void setupSensors();
void GetTOF(uint16_t TOFreadings[]);
void GetElectroMagnet(bool electromagnetStates[]);
void GetInduction(bool inductionSensorStates[]);
void PrintInformation(uint16_t TOFreadings[], bool electromagnetStates[], bool inductionSensorStates[], int programState);

#endif  //PRINT_READINGS_H
