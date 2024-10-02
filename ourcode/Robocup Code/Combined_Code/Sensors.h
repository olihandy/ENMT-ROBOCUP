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
#include "SensorBuffering.h"
#include <Adafruit_TCS34725.h>


extern int elapsed_time;
extern unsigned long lastChangeTime; // Timestamp of the last change
const extern unsigned long timeoutDuration; // 5 seconds

//ELECTROMAGNET
extern int FrontElectromagnetPin;
extern int MiddleElectromagnetPin;
extern int BackElectromagnetPin;
extern const int numElectroMagnets;
extern bool electromagnetStates[];


//INDUCTION
const extern int FrontInductionPin; 
const extern int BackInductionPin;
const extern int numInductiveSensors;
extern bool inductionSensorStates[];


// TOF Sensor Configuration
const extern  byte SX1509_ADDRESS;
const extern int VL53L0X_ADDRESS_START;
const extern int VL53L1X_ADDRESS_START;

//number of sensors in system.
const extern byte SX1509_ADDRESS;
const extern int VL53L0X_ADDRESS_START;
const extern int VL53L1X_ADDRESS_START;
const extern uint8_t sensorCountL1;
const extern  uint8_t sensorCountL0;


// SX1509 and TOF Sensors
extern SX1509 io;
extern VL53L1X sensorsL1[];
extern VL53L0X sensorsL0[];

// TOF Buffer Configuration
const extern int numReadings;
extern uint16_t TOFreadings[];
extern circBuf_t TOFbuffers[];

// Color Sensor Variables
extern Adafruit_TCS34725 tcs;
extern uint16_t colorlist[4];
extern uint16_t red_Start;
extern uint16_t green_Start;
extern uint16_t blue_Start;
extern uint16_t clear_Start;

// Function Prototypes
void setupSensors();
uint16_t* GetTOF();
bool* GetElectroMagnet();
bool* GetInduction();
void PrintInformation();
void UpdateTOFReadings();
uint32_t GetAverageTOFReading(int sensorIndex);
uint16_t* GetAverageTOF();
void colorSensorDetect(uint16_t* returnlist);
void colorStart();
bool ColorCompareHome();
void CheckAndPrintColorMatch();

#endif  // SENSORS_H
