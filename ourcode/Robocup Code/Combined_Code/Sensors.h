#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>  
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

extern int programState;
extern int elapsed_time;

//ELECTROMAGNET
extern const int ElectroMagnet1Pin;
extern const int ElectroMagnet2Pin;
extern const int ElectroMagnet3Pin;
extern bool ElectroMagnet1On;
extern bool ElectroMagnet2On;
extern bool ElectroMagnet3On;
extern const int numElectroMagnets;


//INDUCTION
extern const int FrontInductionPin; 
extern const int BackInductionPin;
extern const int numInductiveSensors;


//TOF
extern const byte SX1509_ADDRESS;
extern const int VL53L0X_ADDRESS_START;
extern const int VL53L1X_ADDRESS_START;

//number of sensors in system.
extern const uint8_t sensorCountL1;
extern const uint8_t sensorCountL0; 

// The Arduino pin connected to the XSHUT pin of each sensor.
extern const uint8_t xshutPinsL1[];
extern const uint8_t xshutPinsL0[];

extern SX1509 io;  // Create an SX1509 object to be used throughout
extern VL53L1X sensorsL1[];
extern VL53L0X sensorsL0[];

extern uint16_t TopRight, TopLeft, TopMiddle, MiddleLeft, MiddleRight, BottomLeft, BottomRight;
extern const int numReadings;

// Function prototypes
void setupSensors();
void GetTOF(uint16_t TOFreadings[]);
void GetElectroMagnet(bool electromagnetStates[]);
void GetInduction(bool inductionSensorStates[]);
void PrintInformation(uint16_t TOFreadings[], bool electromagnetStates[], bool inductionSensorStates[], int programState);

#endif  //PRINT_READINGS_H
