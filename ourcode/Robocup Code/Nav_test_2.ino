#include <Wire.h>  
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <stdio.h>  
#include <time.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int ElectroMagnet1Pin = A12; //Electromagnet pins
const int ElectroMagnet2Pin = A0;
const int ElectroMagnet3Pin = A10;
bool ElectroMagnet1On = false;
bool ElectroMagnet2On = false;
bool ElectroMagnet3On = false;

//This means the pins, not entire ports which the cables connect to
const int InductionPin = 27; //Induction sensor pin to check

double elapsed_time = 0;
const double two_minutes_in_seconds = 120.0;
// Record the start time
int8_t programState = 0;  //0 is moving around, no weight detected, 1 is wall is detected, 2 is weight detected, 3 is returning back home


//Setup of LEDs

const int LED1 = 4;  //Green on top set
const int LED2 = 3;  //Yellow
const int LED3 = 2;  //Red
const int LED4 = 5;  //Green on bottom set


//TOF Setup
const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35

// The number of sensors in your system.
const uint8_t sensorCountL0 = 4;  //sensorcount 
const uint8_t sensorCountL1 = 3;  //sensorcount

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL1[sensorCountL1] = {0, 1, 2};  //Only three needed for three sensors, xshut ports
const uint8_t xshutPinsL0[sensorCountL0] = {3, 4, 5, 6};  //Only two needed for two sensors, xshut ports
//Left 6, Right 7

SX1509 io;  // Create an SX1509 object to be used throughout
VL53L1X sensorsL1[sensorCountL1];
VL53L0X sensorsL0[sensorCountL0];


//Motor setup
Servo myservoA, myservoB;  // create servo object to control a servo
int stop_speed = 1500;     // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
int full_forward_speed = 1900;
int half_forward_speed = 1700;
int full_reverse_speed = 1100;
int half_reverse_speed = 1300;

//Stepper Motor Setup
int MAdirpin = 32;
int MAsteppin = 33;
int MBdirpin = 30;
int MBsteppin = 31;


int timedelay = 10;  //time in milliseconds, do not comment this out
// static long durationA, durationB, TopLeft, TopRight;

void setup()  //Need one setup function
{
  //LEDs
  Serial.print("Setup");
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);  
  digitalWrite(LED4, LOW);   //These will only be called once


  //ElectroMagnet
  pinMode(ElectroMagnet1Pin, OUTPUT);
  pinMode(ElectroMagnet2Pin, OUTPUT);
  pinMode(ElectroMagnet3Pin, OUTPUT);

  //Induction sensor
  pinMode(InductionPin, INPUT);


  if (!io.begin(SX1509_ADDRESS)) {
    Serial.println("Failed to to talk to IO Expander for TOFs");
    while (1) {};
  }

  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C
  Wire1.begin();
  Wire1.setClock(400000);  // use 400 kHz I2C

  Serial.println("Setup Wires");

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCountL1; i++) {
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }
  // Disable/reset all sensors by driving their XSHUT pins low.
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

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);

    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);

      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);

    sensorsL1[i].startContinuous(50);
  }

  Serial.println("Configured TOFs");

  //Motors
  // myservoA.attach(7);  // attaches the servo  to the servo object using pin 0
  myservoB.attach(8);  // attaches the servo  to the servo object using pin 1 PROBLEM SERVO IDK WHY

  Serial.println("Configured Motors");

  //Stepper Motors
  pinMode(MAdirpin,OUTPUT);
  pinMode(MAsteppin,OUTPUT);
  pinMode(MBdirpin,OUTPUT);
  pinMode(MBsteppin,OUTPUT);
  digitalWrite(MAdirpin,LOW);
  digitalWrite(MBdirpin,LOW);

  Serial.print("End of setup");
}

void full_reverse(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_reverse_speed);

  delay(timedelay);
}

void stop(int timedelay) {
  myservoA.writeMicroseconds(stop_speed);
  myservoB.writeMicroseconds(stop_speed);

  delay(timedelay);
}

void full_forward(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_forward_speed);

}

void full_turn_left(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay);
}

void forward_left(int timedelay) {
  myservoA.writeMicroseconds(half_forward_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay);
}

void full_turn_right(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(timedelay);
}

void forward_right(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(half_forward_speed);
}


//=========================================================================================================================================================================================================================================================
//=========================================================================================================================================================================
//========================================================================================================================================================================================================================================================
//=========================================================================================================================================================================
//========================================================================================================================================================================================================================================================

void PrintInfo(int programState, uint16_t TopLeft, uint16_t TopMiddle, uint16_t TopRight, uint16_t MiddleLeft, uint16_t MiddleRight, uint16_t BottomRight, uint16_t BottomLeft) {
  
  //Electromagnets
  Serial.print(ElectroMagnet1On);
  Serial.print(ElectroMagnet2On);
  Serial.print(ElectroMagnet3On);
  Serial.print("     ");
  //Sensor readings
  Serial.print("Top L ");
  Serial.print(TopLeft);
  if (sensorsL1[0].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print('\t');

  Serial.print("M ");
  Serial.print(TopMiddle);
  if (sensorsL1[2].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print('\t');

  Serial.print("R ");
  Serial.print(TopRight);
  if (sensorsL1[1].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print('\t');

  Serial.print("Mid  R ");
  Serial.print(MiddleRight);
  if (sensorsL0[0].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print('\t');

  Serial.print("L ");
  Serial.print(MiddleLeft);
  if (sensorsL0[1].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print('\t');

  Serial.print("Bottom  R ");
  Serial.print(BottomRight);
  if (sensorsL0[2].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print('\t');

  Serial.print("L ");
  Serial.print(BottomLeft);
  if (sensorsL0[3].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print('\t');

  if(programState == 0) { //printing stuff relating to task
    Serial.print("Moving Forward");
  } else if(programState == 1) {
    Serial.print("Turning");
  } else if(programState == 2) {
    Serial.print("Weight Found");
  } else {
    Serial.print("Going Home");
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    digitalWrite(LED3,LOW);
    digitalWrite(LED4,HIGH);
  } 
  Serial.print("       ");
  Serial.println();  
}

void loop() {
  //Setting up
  uint16_t TopLeft = sensorsL1[0].read()/10;
  uint16_t TopRight = sensorsL1[1].read()/10;
  uint16_t TopMiddle = sensorsL1[2].read()/10;

  uint16_t MiddleRight = sensorsL0[0].readRangeContinuousMillimeters()/10;
  uint16_t MiddleLeft = sensorsL0[1].readRangeContinuousMillimeters()/10;
  uint16_t BottomRight = sensorsL0[2].readRangeContinuousMillimeters()/10;
  uint16_t BottomLeft = sensorsL0[3].readRangeContinuousMillimeters()/10;

  elapsed_time = millis()/1000;

  programState = 0;
  // 0 = Foward, 1 = turning, 2 = weight found, 3 = going home
  if (TopLeft < 20 || TopRight < 20 || TopMiddle < 10) {
    programState = 1;
  }  
  if ((MiddleLeft > 20 && BottomLeft < 20) || (MiddleRight > 20 && BottomRight < 20)) { //Object like weight is found
    programState = 2;
  }  
  if ((elapsed_time > 1000) || (ElectroMagnet1On & ElectroMagnet2On & ElectroMagnet3On) ) {
    programState = 3;
  }

  PrintInfo(programState, TopLeft, TopMiddle, TopRight, MiddleLeft, MiddleRight, BottomRight, BottomLeft);

  if(programState == 0) {
    digitalWrite(LED1,HIGH); //Green top set, on
    full_forward(timedelay); //can go full forward
  }
  
  if (programState == 1) {
    digitalWrite(LED1,LOW);
    
    if(TopMiddle > TopLeft || TopMiddle > TopRight) {

      if (TopLeft < TopRight) {
        full_turn_right(timedelay);
        digitalWrite(LED3,LOW);
        digitalWrite(LED2,HIGH);

      } else {

        full_turn_left(timedelay);
        digitalWrite(LED3,LOW);
        digitalWrite(LED2,HIGH);

      }
    } else if((TopMiddle < TopLeft) && (TopMiddle < TopRight)) {
      full_turn_left(timedelay);
    }
  }
  
  if (programState == 2) {
    if(BottomLeft > BottomRight)
      forward_left(timedelay);
    } else {
      forward_right(timedelay);
    }

}
