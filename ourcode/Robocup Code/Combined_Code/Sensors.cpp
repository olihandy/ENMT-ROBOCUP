#include "Sensors.h"
#include "SensorBuffering.h"

#define BUFFER_SIZE 10
const int numReadings = 7;

circBuf_t TOFbuffers[numReadings];

int elapsed_time = 0;
unsigned long lastChangeTime = 0; // Timestamp of the last change
const unsigned long timeoutDuration = 5000; // 5 seconds

const int sensorErrorValue = 819;
const int maxSensorValue = 115;

// ELECTROMAGNET
const int FrontElectromagnetPin = 20;
const int MiddleElectromagnetPin = 24;
const int BackElectromagnetPin = 14;
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


uint16_t TOFreadings[numReadings];
bool electromagnetStates[numElectroMagnets];
bool inductionSensorStates[numInductiveSensors];

void setupSensors() {
    Serial.print("Setup");

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
    for (int i = 0; i < numReadings; i++) {
        initCircBuf(&TOFbuffers[i], BUFFER_SIZE);
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
        io.digitalWrite(xshutPinsL0[i], HIGH);
        delay(10);

        sensorsL0[i].setTimeout(500);
        if (!sensorsL0[i].init()) {
            Serial.print("Failed to detect and initialize sensor L0 ");
            Serial.println(i);
            while (1);
        }
        sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);
        sensorsL0[i].startContinuous(50);
    }
    for (uint8_t i = 0; i < sensorCountL1; i++) {
        io.digitalWrite(xshutPinsL1[i], HIGH);
        delay(10);
        sensorsL1[i].setTimeout(500);
        if (!sensorsL1[i].init()) {
            Serial.print("Failed to detect and initialize sensor L1 ");
            Serial.println(i);
            while (1);
        }
        sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);
        sensorsL1[i].startContinuous(50);
    }

    Serial.println("Configured TOFs");
}

uint16_t* GetTOF() {
    // Read top sensors (L1)
    TOFreadings[0] = sensorsL1[0].read() / 10; // Top Left
    TOFreadings[1] = sensorsL1[1].read() / 10; // Top Middle
    TOFreadings[2] = sensorsL1[2].read() / 10; // Top Right

    // Read bottom sensors (L0)
    TOFreadings[3] = sensorsL0[0].readRangeContinuousMillimeters() / 10; // Middle Left
    TOFreadings[4] = sensorsL0[1].readRangeContinuousMillimeters() / 10; // Middle Right
    TOFreadings[5] = sensorsL0[2].readRangeContinuousMillimeters() / 10; // Bottom Left
    TOFreadings[6] = sensorsL0[3].readRangeContinuousMillimeters() / 10; // Bottom Right

    // Replace any erroneous 819 values in the bottom 4 sensors (indices 3-6)
    for (int i = 3; i < 7; i++) {
        if (TOFreadings[i] == sensorErrorValue) {
            TOFreadings[i] = maxSensorValue; // Set fallback value if 819 is found
        }
    }

    return TOFreadings; // Return pointer to TOF readings
}

void UpdateTOFReadings() {
    // Get TOF readings
    uint16_t* readings = GetTOF();

    // Write new readings into the circular buffers, skipping erroneous 819 values
    for (int i = 0; i < numReadings; i++) {
        if (readings[i] != sensorErrorValue) {  // Ignore readings of 819
            writeCircBuf(&TOFbuffers[i], readings[i]);
        }
    }
}

// Function to calculate the average TOF reading from the circular buffer for a specific sensor index
uint32_t GetAverageTOFReading(int sensorIndex) {
    uint32_t sum = 0;
    int count = 0;

    // Loop through the buffer to calculate the sum and count valid readings
    for (int i = 0; i < TOFbuffers[sensorIndex].size; i++) {
        if (TOFbuffers[sensorIndex].data[i] != sensorErrorValue) { // Skip erroneous readings
            sum += TOFbuffers[sensorIndex].data[i];
            count++;
        }
    }

    // Return average or maxSensorValue if no valid readings were found
    return (count > 0) ? (sum / count) : maxSensorValue;
}

// Function to retrieve average readings for all TOF sensors
uint16_t* GetAverageTOF() {
    static uint16_t averageTOF[numReadings];

    for (int i = 0; i < numReadings; i++) {
        averageTOF[i] = GetAverageTOFReading(i);
    }

    return averageTOF; // Return pointer to average TOF readings
}

bool* GetElectroMagnet() {
    electromagnetStates[0] = digitalRead(FrontElectromagnetPin);
    electromagnetStates[1] = digitalRead(MiddleElectromagnetPin);
    electromagnetStates[2] = digitalRead(BackElectromagnetPin);
    return electromagnetStates; // Return pointer to electromagnet states
}

bool* GetInduction() {
    inductionSensorStates[0] = !digitalRead(FrontInductionPin);
    inductionSensorStates[1] = !digitalRead(BackInductionPin);
    return inductionSensorStates; // Return pointer to induction sensor states
}

void PrintInformation() {
    elapsed_time = millis() / 1000;
    UpdateTOFReadings(); // Update TOF readings
    GetElectroMagnet(); // Update electromagnet states
    GetInduction(); // Update induction sensor states

    // Get average TOF readings
    uint16_t* averageReadings = GetAverageTOF();

    Serial.print("Top  L ");
    Serial.print(averageReadings[1]);
    if (sensorsL1[0].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print("\t");

    Serial.print("M ");
    Serial.print(averageReadings[0]);
    if (sensorsL1[2].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print("\t");

    Serial.print("R ");
    Serial.print(averageReadings[2]);
    if (sensorsL1[1].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print("\t");

    Serial.print("Middle  R ");
    Serial.print(averageReadings[4]);
    if (sensorsL0[0].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print("\t");

    Serial.print("L ");
    Serial.print(averageReadings[3]);
    if (sensorsL0[1].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print("\t");

    Serial.print("Bottom  R ");
    Serial.print(averageReadings[6]);
    if (sensorsL0[2].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print("\t");

    Serial.print("L ");
    Serial.print(averageReadings[5]);
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
