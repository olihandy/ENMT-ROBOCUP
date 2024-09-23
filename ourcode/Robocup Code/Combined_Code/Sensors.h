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

extern int elapsed_time;

//ELECTROMAGNET
const extern int ElectroMagnet1Pin;
const extern int ElectroMagnet2Pin;
const extern int ElectroMagnet3Pin;
extern bool ElectroMagnet1On;
extern bool ElectroMagnet2On;
extern bool ElectroMagnet3On;
const extern int numElectroMagnets;


//INDUCTION
const extern int FrontInductionPin; 
const extern int BackInductionPin;
const extern int numInductiveSensors;


//TOF
const extern  byte SX1509_ADDRESS;
const extern int VL53L0X_ADDRESS_START;
const extern int VL53L1X_ADDRESS_START;

//number of sensors in system.
const extern uint8_t sensorCountL1;
const extern uint8_t sensorCountL0; 

// The Arduino pin connected to the XSHUT pin of each sensor.
extern const uint8_t xshutPinsL1[];
extern const uint8_t xshutPinsL0[];

extern SX1509 io;  // Create an SX1509 object to be used throughout
extern VL53L1X sensorsL1[];
extern VL53L0X sensorsL0[];

const extern int numReadings;

extern uint16_t TOFreadings[];
extern bool electromagnetStates[];
extern bool inductionSensorStates[];

// Function prototypes
void setupSensors();
uint16_t* GetTOF();
bool* GetElectroMagnet();
bool* GetInduction();
void PrintInformation();

#endif  //PRINT_READINGS_H
