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

// Global variables for time tracking
extern int elapsed_time;                   // Time elapsed since start
extern unsigned long lastChangeTime;       // Timestamp of the last change
extern const unsigned long timeoutDuration; // Duration for timeouts (e.g., 5 seconds)

extern uint16_t averagedTOFreadings[];


// Electromagnet pins and states
extern int FrontElectromagnetPin;
extern int MiddleElectromagnetPin;
extern int BackElectromagnetPin;
extern const int numElectroMagnets;
extern bool electromagnetStates[];         // State of electromagnets

// Induction sensor pins and states
extern const int FrontInductionPin; 
extern const int BackInductionPin;
extern const int numInductiveSensors;
extern bool inductionSensorStates[];       // State of induction sensors

// Go Button pin
extern const int GoButtonPin;              

// TOF Sensor Configuration
extern const byte SX1509_ADDRESS;          // I/O expander address
extern const int VL53L0X_ADDRESS_START;    // Start address for VL53L0X sensors
extern const int VL53L1X_ADDRESS_START;    // Start address for VL53L1X sensors

// Number of sensors in the system
extern const uint8_t sensorCountL1;       // Count for L1 sensors
extern const uint8_t sensorCountL0;       // Count for L0 sensors

// SX1509 and TOF Sensors
extern SX1509 io;                          // I/O expander object
extern VL53L1X sensorsL1[];                // Array for VL53L1X sensors
extern VL53L0X sensorsL0[];                // Array for VL53L0X sensors

// TOF Buffer Configuration
const extern int numReadings;              // Number of readings for TOF sensors
extern uint16_t TOFreadings[];             // TOF readings array
extern circBuf_t TOFbuffers[];             // Circular buffers for TOF readings

// Color Sensor Variables
extern Adafruit_TCS34725 tcs;              // Color sensor object
extern uint16_t colorlist[4];              // List to hold color readings
extern uint16_t red_Start;                 // Starting red value
extern uint16_t green_Start;               // Starting green value
extern uint16_t blue_Start;                // Starting blue value
extern uint16_t clear_Start;               // Starting clear value

// Function Prototypes
void setupSensors();
void GetTOF();
void GetElectroMagnet();
void GetInduction();
void PrintInformation();
void UpdateTOFReadings();
void colorSensorDetect(uint16_t* returnlist);
void colorStart();
bool ColorCompareHome();
void CheckAndPrintColorMatch();

#endif  // SENSORS_H
