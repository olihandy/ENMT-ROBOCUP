#include "Sensors.h"
#include "SensorBuffering.h"

#define BUFFER_SIZE 3

// TOF
const int numReadings = 7;

// Define circular buffers for each TOF sensor
circBuf_t TOFbuffer0;
circBuf_t TOFbuffer1;
circBuf_t TOFbuffer2;
circBuf_t TOFbuffer3;
circBuf_t TOFbuffer4;
circBuf_t TOFbuffer5;
circBuf_t TOFbuffer6;

uint16_t averagedTOFreadings[numReadings];

unsigned long elapsed_time = 0;
unsigned long lastChangeTime = 0; // Timestamp of the last change
const unsigned long timeoutDuration = 5000; // 2 seconds

const int maxSensorValue = 115;
const int SensorErrorValue = 819;

// ELECTROMAGNET
extern int FrontElectromagnetPin;
extern int MiddleElectromagnetPin;
extern int BackElectromagnetPin;
const int numElectroMagnets = 3;

// INDUCTION
const int FrontInductionPin = 27; 
const int BackInductionPin = 26;
const int numInductiveSensors = 2;

// TOF
const byte SX1509_ADDRESS = 0x3F;
const int VL53L0X_ADDRESS_START = 0x30;
const int VL53L1X_ADDRESS_START = 0x35;

// number of sensors in the system.
const uint8_t sensorCountL1 = 3;
const uint8_t sensorCountL0 = 4; 

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL1[sensorCountL1] = {0, 1, 2};
const uint8_t xshutPinsL0[sensorCountL0] = {3, 4, 5, 6};

SX1509 io;  // Create an SX1509 object to be used throughout
VL53L1X sensorsL1[sensorCountL1];
VL53L0X sensorsL0[sensorCountL0];

// Color Sensor setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
const int ColorSensorPin = 41;
uint16_t colorlist[4];
uint16_t clear_Start;
uint16_t red_Start;
uint16_t green_Start;
uint16_t blue_Start;


uint16_t TOFreadings[numReadings];
bool electromagnetStates[numElectroMagnets];
bool inductionSensorStates[numInductiveSensors];

void setupSensors() {
  Serial.begin(9600);

  // ElectroMagnet
  pinMode(FrontElectromagnetPin, OUTPUT);
  pinMode(MiddleElectromagnetPin, OUTPUT);
  pinMode(BackElectromagnetPin, OUTPUT);  
  
  // Induction sensor
  pinMode(FrontInductionPin, INPUT);
  pinMode(BackInductionPin, INPUT);

  // TOF
  if (!io.begin(SX1509_ADDRESS)) {
    Serial.println("Failed to talk to IO Expander for TOFs");
    while (1) {};
  }
  
  // Initialize circular buffers for each TOF sensor
  initCircBuf(&TOFbuffer0, BUFFER_SIZE);
  initCircBuf(&TOFbuffer1, BUFFER_SIZE);
  initCircBuf(&TOFbuffer2, BUFFER_SIZE);
  initCircBuf(&TOFbuffer3, BUFFER_SIZE);
  initCircBuf(&TOFbuffer4, BUFFER_SIZE);
  initCircBuf(&TOFbuffer5, BUFFER_SIZE);
  initCircBuf(&TOFbuffer6, BUFFER_SIZE);

  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C
  Wire1.begin();
  Wire1.setClock(400000);  // use 400 kHz I2C

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCountL1; i++) {
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCountL1; i++) {
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);
    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init()) {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);
      // while (1);
    }
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);
    sensorsL1[i].startContinuous(50);
  }

  for (uint8_t i = 0; i < sensorCountL0; i++) {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCountL0; i++) {
    io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);

    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init()) {
      Serial.print("Failed to detect and initialize sensor L0 ");
      Serial.println(i);
      // while (1);
    }
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);
    sensorsL0[i].startContinuous(50);
  }

  Serial.println("Configured TOFs");

  // Initialize color sensor
  if (tcs.begin(ColorSensorPin, &Wire1)) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void colorSensorDetect(uint16_t* returnlist) {
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED
  delay(60);  // takes 50ms to read 
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);  // turn off LED

  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;

  returnlist[0] = clear;
  returnlist[1] = red;
  returnlist[2] = green;
  returnlist[3] = blue;
}

void colorStart() {
  colorSensorDetect(colorlist);  // Read the initial color
  clear_Start = colorlist[0];
  red_Start = colorlist[1];
  green_Start = colorlist[2];
  blue_Start = colorlist[3]; 
}

bool ColorCompareHome() {
  colorSensorDetect(colorlist);
  // Define separate tolerances for clear and colors
  const int clearTolerance = 30;  // Higher tolerance for clear channel
  const int colorTolerance = 15;   // Tighter tolerance for RGB channels

  // Array of starting colors and current colors for comparison
  uint16_t startColors[] = {clear_Start, red_Start, green_Start, blue_Start};
  const char* colorNames[] = {"Clear", "Red", "Green", "Blue"};
  int tolerances[] = {clearTolerance, colorTolerance, colorTolerance, colorTolerance};

  // Loop through each color and compare with the starting color
  for (int i = 0; i < 4; i++) {
    int difference = abs(colorlist[i] - startColors[i]);
    if (difference > tolerances[i]) {
      // Serial.print(colorNames[i]);
      // Serial.print(" value is out of tolerance by ");
      // Serial.println(difference);  // Print the difference for debugging
      return false;  // One or more colors are out of tolerance
    }
  }
  return true;
}

void GetTOF(void) {

  // Update circular buffers
  writeCircBuf(&TOFbuffer0, sensorsL1[0].read(false));
  writeCircBuf(&TOFbuffer1, sensorsL1[1].read(false));
  writeCircBuf(&TOFbuffer2, sensorsL1[2].read(false));
  writeCircBuf(&TOFbuffer3, sensorsL0[0].readRangeContinuousMillimeters());
  writeCircBuf(&TOFbuffer4, sensorsL0[1].readRangeContinuousMillimeters());
  writeCircBuf(&TOFbuffer5, sensorsL0[2].readRangeContinuousMillimeters());
  writeCircBuf(&TOFbuffer6, sensorsL0[3].readRangeContinuousMillimeters());

  // Calculate averaged values from circular buffers
  averagedTOFreadings[0] = averageCircBuf(&TOFbuffer0);
  averagedTOFreadings[1] = averageCircBuf(&TOFbuffer1);
  averagedTOFreadings[2] = averageCircBuf(&TOFbuffer2);
  averagedTOFreadings[3] = averageCircBuf(&TOFbuffer3);
  averagedTOFreadings[4] = averageCircBuf(&TOFbuffer4);
  averagedTOFreadings[5] = averageCircBuf(&TOFbuffer5);
  averagedTOFreadings[6] = averageCircBuf(&TOFbuffer6);
}

void GetElectroMagnet(void) {
    electromagnetStates[0] = digitalRead(FrontElectromagnetPin);
    electromagnetStates[1] = digitalRead(MiddleElectromagnetPin);
    electromagnetStates[2] = digitalRead(BackElectromagnetPin);
}

void GetInduction(void) {
    inductionSensorStates[0] = !digitalRead(FrontInductionPin);
    inductionSensorStates[1] = !digitalRead(BackInductionPin);
}

void PrintInformation() {
    if (ColorCompareHome()) {
      Serial.println("Color matches the starting color.");
    } else {
      Serial.println("Color does NOT match the starting color.");
    }
    // Get average TOF readings
    Serial.print("Top  L ");
    Serial.print(averagedTOFreadings[1]);
    if (sensorsL1[0].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print("\t");

    Serial.print("M ");
    Serial.print(averagedTOFreadings[0]);
    if (sensorsL1[2].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print("\t");

    Serial.print("R ");
    Serial.print(averagedTOFreadings[2]);
    if (sensorsL1[1].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print("\t");

    Serial.print("Middle  R ");
    Serial.print(averagedTOFreadings[4]);
    if (sensorsL0[0].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print("\t");

    Serial.print("L ");
    Serial.print(averagedTOFreadings[3]);
    if (sensorsL0[1].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print("\t");

    Serial.print("Bottom  R ");
    Serial.print(averagedTOFreadings[6]);
    if (sensorsL0[2].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print("\t");

    Serial.print("L ");
    Serial.print(averagedTOFreadings[5]);
    if (sensorsL0[3].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print("\t");

    Serial.print(electromagnetStates[2]);
    Serial.print(electromagnetStates[1]);
    Serial.print(electromagnetStates[0]);
    Serial.print("\t");

    Serial.print("Front Induction Sensor: ");
    Serial.print(inductionSensorStates[0]);
    Serial.print("\t");  
    Serial.print("Back Induction Sensor: ");
    Serial.print(inductionSensorStates[1]);
    Serial.print("\t");

    Serial.print(elapsed_time);
    Serial.print("\t");
}
