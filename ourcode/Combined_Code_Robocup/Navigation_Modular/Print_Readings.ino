#include <Wire.h>  
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//ELECTROMAGNET
const int ElectroMagnet1Pin = 25;
const int ElectroMagnet2Pin = 24;
const int ElectroMagnet3Pin = 14;
bool ElectroMagnet1On = false;
bool ElectroMagnet2On = false;
bool ElectroMagnet3On = false;


//INDUCTION
const int FrontInductionPin = 27; //Induction sensor pin to check
const int BackInductionPin = 26;
bool FrontInductionDetected = 0;
bool BackInductionDetected = 0;


//TOF
const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35

//number of sensors in system.
const uint8_t sensorCountL1 = 3;
const uint8_t sensorCountL0 = 4; 
// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL1[sensorCountL1] = {0, 1, 2};
const uint8_t xshutPinsL0[sensorCountL0] = {3, 4, 5, 6}

SX1509 io;  // Create an SX1509 object to be used throughout
VL53L1X sensorsL1[sensorCountL1];
VL53L0X sensorsL0[sensorCountL0];

uint16_t TopRight, TopLeft, TopMiddle, MiddleLeft, MiddleRight, BottomLeft, BottomRight;
const int numReadings = 7;

void setup() {
  // put your setup code here, to run once:
  Serial.print("Setup");

  //ElectroMagnet
  pinMode(ElectroMagnet1Pin, OUTPUT);
  pinMode(ElectroMagnet2Pin, OUTPUT);
  pinMode(ElectroMagnet3Pin, OUTPUT);  
  
  //Induction sensor
  pinMode(FrontInductionPin, INPUT);
  pinMode(BaclInductionPin, INPUT);

  //TOF
  if (!io.begin(SX1509_ADDRESS)) {
    Serial.println("Failed to to talk to IO Expander for TOFs");
    while (1) {};
  }

  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C
  Wire1.begin();
  Wire1.setClock(400000);  // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCountL1; i++) {
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCountL0; i++) {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
  }

  for (uint8_t i = 0; i < sensorCountL0; i++) {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);

    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init()) {
      Serial.print("Failed to detect and initialize sensor L0 ");
      Serial.println(i);
      while (1);
    }
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);
    sensorsL0[i].startContinuous(50);
  }
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);
    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);

      while (1);
    }
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);
    sensorsL1[i].startContinuous(50);
  }

  Serial.println("Configured TOFs");


}

void GetTOF(uint16_t TOFreadings[]) {
  readings[0] = sensorsL1[0].read() / 10;  // TopRight
  readings[1] = sensorsL1[1].read() / 10;  // TopLeft
  readings[2] = sensorsL1[2].read() / 10;  // TopMiddle
  readings[3] = sensorsL0[0].readRangeContinuousMillimeters() / 10;  // MiddleLeft
  readings[4] = sensorsL0[1].readRangeContinuousMillimeters() / 10;  // MiddleRight
  readings[5] = sensorsL0[2].readRangeContinuousMillimeters() / 10;  // BottomLeft
  readings[6] = sensorsL0[3].readRangeContinuousMillimeters() / 10;  // BottomRight
}

void PrintInformation(uint16_t TOFreadings[], int programState) {
  Serial.print("Top L ");
  Serial.print(readings[1]);
  if (sensorsL1[0].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
  Serial.print('\t');

  Serial.print("M ");
  Serial.print(readings[2]);
  if (sensorsL1[2].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
  Serial.print('\t');

  Serial.print("R ");
  Serial.print(readings[0]);
  if (sensorsL1[1].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
  Serial.print('\t');

  Serial.print("Mid  R ");
  Serial.print(readings[4]);
  if (sensorsL0[0].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
  Serial.print('\t');

  Serial.print("L ");
  Serial.print(readings[3]);
  if (sensorsL0[1].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
  Serial.print('\t');

  Serial.print("Bottom  R ");
  Serial.print(readings[6]);
  if (sensorsL0[2].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
  Serial.print('\t');

  Serial.print("L ");
  Serial.print(readings[5]);
  if (sensorsL0[3].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
  Serial.print('\t');


  if(programState == 0) { //printing stuff relating to task
    Serial.print("Moving Forward");
    full_forward(timedelay); //can go full forward
  } else if(programState == 1) {
    Serial.print("Turning");
  } else if(programState == 2) {
    Serial.print("Weight Found");
  } else {
    Serial.print("Going Home");
  } 

  Serial.print("\t");

  Serial.print(ElectroMagnet1On);
  Serial.print("\t");
  Serial.print(ElectroMagnet2On);
  Serial.print("\t");
  Serial.print(ElectroMagnet3On);
  Serial.print("\t");

  Serial.print(elapsed_time);
  Serial.print("\t");
  Serial.print("       ");
}

void loop() {
  // Get the readings from the sensors
  uint16_t TOFreadings[numReadings];

  // Get sensor readings and store them in the array
  GetTOF(readings);

  // Example program state, you can update this according to your logic
  int programState = 0;

  // Call PrintInformation with the array of readings and program state
  PrintInformation(TOFreadings, programState);

  delay(10); // Adjust delay as needed
}

