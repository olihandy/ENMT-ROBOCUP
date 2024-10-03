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
#include <arduino.h>

const int ElectroMagnet1Pin = 25; //Electromagnet pins, changed from A as the front and rear never turn on
const int ElectroMagnet2Pin = 24;
const int ElectroMagnet3Pin = 14;
bool ElectroMagnet1On = false;
bool ElectroMagnet2On = false;
bool ElectroMagnet3On = false;

//Can't call functions globally, Global variables are set up here
//InitColorReading =
int CurrentposX = 0;
int CurrentposY = 0;
int previousMove = 0; //0 for left, 1 for right
int PrevturnTime = 0;
int Xacclist[50] = { 0 };
int Yacclist[50] = { 0 };
int AverageAccelerationX = 0;
int AverageAccelerationY = 0;
double prevtime = 0.0;
int CurrentOrienZ = 0;
int AverageAngZ = 0;
int AngaccZList[50] = { 0 };
double Startangle = 0;

//This means the pins, not entire ports which the cables connect to
const int InductionPin = 27; //Induction sensor pin to check
int InductionDetected = 0;
const int CollectPin = 15;

double elapsed_time = 0;
const double two_minutes_in_seconds = 120.0;
// Record the start time
int8_t programState = 0;  //0 is moving around, no weight detected, 1 is wall is detected, 2 is weight detected, 3 is returning back home

//For going home
int FoundWall = 0; //0 not found, 1 right wall found, 2 left wall found
int FoundHome = 0; //if found home

//Setup of LEDs
const int LED1 = 4;  //Green on top set
const int LED2 = 3;  //Yellow
const int LED3 = 2;  //Red
const int LED4 = 5;  //Green on bottom set

//Positioning Variable Setups
int StraightTotalTravelxperp = 0;
int StraightTotalTravelypara = 0;

//TOF Setup
const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35

// The number of sensors in your system.
const uint8_t sensorCountL1 = 3;  //sensorcount
const uint8_t sensorCountL0 = 4;  //sensorcount 

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
int full_reverse_speed = 1900;
int full_forward_speed = 1100;
int half_reverse_speed = 1750;
int half_forward_speed = 1250;


//Stepper Motor Setup
int MAdirpin = 32;
int MAsteppin = 33;
int MBdirpin = 30;
int MBsteppin = 31;
int StepperPosition = 14000;
int StepperState = 0; //0 for moving down, 1 for stopping to make electromsagnets work, 2 for back up then resets to 0 afterwards


int timedelay = 10;  //time in milliseconds, do not comment this out
// static long durationA, durationB, TopLeft, TopRight;


//Encoder Setup
enum PinAssignments {
  encoder1PinA = 2,
  encoder1PinB = 3,
  
  encoder2PinA = 4,
  encoder2PinB = 5,
};

volatile unsigned int encoderPos1 = 0;
unsigned int lastReportedPos1 = 1;
volatile unsigned int encoderPos2 = 0;
unsigned int lastReportedPos2 = 1;

boolean A_set1 = false;
boolean B_set1 = false;
boolean A_set2 = false;
boolean B_set2 = false;



//IMU setup

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); //searches to find IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);



//Color Sensor setup
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

uint16_t colorlist[4];
uint16_t red_Start;
uint16_t green_Start;
uint16_t blue_Start;
uint16_t clear_Start;



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

  //Encoder
  pinMode(encoder1PinA, INPUT);       //Set encoder pins as inputs
  pinMode(encoder1PinB, INPUT); 
  pinMode(encoder2PinA, INPUT); 
  pinMode(encoder2PinB, INPUT); 
  myservoA.attach(0);  // attaches the servo  to the servo object useing pin 0
  myservoB.attach(1);  // attaches the servo  to the servo object useing pin 1
  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, CHANGE);  //Set up an interrupt for each encoder
  attachInterrupt(digitalPinToInterrupt(4), doEncoder2A, CHANGE);


  //ElectroMagnet
  pinMode(ElectroMagnet1Pin, OUTPUT);
  pinMode(ElectroMagnet2Pin, OUTPUT);
  pinMode(ElectroMagnet3Pin, OUTPUT);

  //Induction sensor
  pinMode(InductionPin, INPUT);

  //Color sensor
  Serial.begin(9600);
  Serial.println("Color View Test!");

  Wire1.begin();
  Wire.begin();

  if (tcs.begin(41, &Wire1)) 
  {
    Serial.println("Found sensor");
  } else 
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  //TOF
  // while (!Serial) {}    NO USB FOR RUNNING CODE
  // Serial.begin(115200);

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
  myservoA.attach(7);  // attaches the servo  to the servo object using pin 0
  myservoB.attach(8);  // attaches the servo  to the servo object using pin 1

  Serial.println("Configured Motors");

  //Stepper Motors
  pinMode(MAdirpin,OUTPUT);
  pinMode(MAsteppin,OUTPUT);
  pinMode(MBdirpin,OUTPUT);
  pinMode(MBsteppin,OUTPUT);



  //IMU
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.print("IMU detected");
  delay(1000);

  Serial.print("End of setup");
}

void colorSensorDetect(uint16_t* returnlist) {
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED
  
  //Serial.print("Clear:\t"); Serial.print(clear);
  //Serial.print("\tR:\t"); Serial.print(red);
  //Serial.print("\tG:\t"); Serial.print(green);
  //Serial.print("\tB:\t"); Serial.print(blue);

  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;
  //Serial.print("\t\t"); //t is space
  //Serial.print((int)r, HEX); Serial.print("\t"); Serial.print((int)g, HEX); Serial.print("\t"); Serial.print((int)b, HEX); //Need to move these to modularized code
  //Serial.println();
  returnlist[0] = clear;
  returnlist[1] = red;
  returnlist[2] = green;
  returnlist[3] = blue;
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

  delay(timedelay);
}

void half_forward(int timedelay) {
  myservoA.writeMicroseconds(half_forward_speed);
  myservoB.writeMicroseconds(half_forward_speed);

  delay(timedelay);
}

void full_turn_right(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay);
}

void forward_right(int timedelay) {
  myservoA.writeMicroseconds(half_forward_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay);
}

void directioncheck_right(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay * 3);  //Will be replaced by IMU code
}

void full_turn_left(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(timedelay);
}

void forward_left(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(half_forward_speed);
}

sensors_event_t orientationData , linearAccelData; //, angVelocityData , linearAccelData, magnetometerData,  gravityData;
double ori[3];
double acc[3]; //comment out if not using IMU

void printEvent(sensors_event_t* event) { //problems getting right event
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    ori[0] = x;
    ori[1] = y;
    ori[2] = z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    acc[0] = x;
    acc[1] = y;
    acc[2] = z;
  } else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
  Serial.print("\n");
  
  // return *XYZList;
}



void IMUGetPos(void) {
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); //degrees
  //bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); //rad/s
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); //m/s^2
  //bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER); //Gravity detected with acceleration
  //bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY); //Gravity preset

  printEvent(&orientationData);
  //printEvent(&angVelocityData);
  //printEvent(&linearAccelData);
  //printEvent(&magnetometerData);
  printEvent(&linearAccelData);
  //printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();  //Can remove this
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);  
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);
  
  Serial.println("--");
  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

void colorStart() {
  clear_Start = colorlist[0];
  red_Start = colorlist[1];
  green_Start = colorlist[2];
  blue_Start = colorlist[3]; 
}

bool ColorCompareHome() { //Clear is unfiltered photodiode
  if (((colorlist[1]-red_Start)<10) && ((colorlist[1]-red_Start)>(-10))) {
    if (((colorlist[2]-green_Start)<10) && ((colorlist[2]-green_Start)>(-10))) {
      if (((colorlist[3]-blue_Start)<10) && ((colorlist[3]-blue_Start)>(-10))) { //Multiplied by colorlist[0]/clear_start?
        bool ColorCompareHome = true;
      } else {
        bool ColorCompareHome = false;
      }
    } else {
      bool ColorCompareHome = false;
    }
  } else {
    bool ColorCompareHome = false;
  }
  return ColorCompareHome;
}


//=========================================================================================================================================================================================================================================================
//=========================================================================================================================================================================
//========================================================================================================================================================================================================================================================
//=========================================================================================================================================================================
//========================================================================================================================================================================================================================================================


void loop() {
  elapsed_time = millis()/1000; //Seconds since program has been running

  uint16_t TopRight = sensorsL1[0].read()/10;  //Long range TOF reads
  uint16_t TopLeft = sensorsL1[1].read()/10;
  uint16_t TopMiddle = sensorsL1[2].read()/10;

  uint16_t MiddleLeft = sensorsL0[0].readRangeContinuousMillimeters()/10;
  uint16_t MiddleRight = sensorsL0[1].readRangeContinuousMillimeters()/10;
  uint16_t BottomLeft = sensorsL0[2].readRangeContinuousMillimeters()/10;
  uint16_t BottomRight = sensorsL0[3].readRangeContinuousMillimeters()/10;

  
  //Getting current acceleration and therefore position:
  //Have weighted average of IMU and Encoder
  
  IMUGetPos();
  // Serial.println(acc[0]); //Most recent x orientation/acceleration obtained
  // Serial.println(ori[0]);
  // Serial.println(acc[1]); //y orientation/acceleration obtained
  // Serial.println(ori[1]);
  // Serial.println(acc[2]); //Angular Z orientation/acceleration obtained
  // Serial.println(ori[2]);
  for (uint16_t element=0; element<=48; element++){                 //shifting the window of detected accelerations for X
    Xacclist[element+1] = Xacclist[element];
  }
  Xacclist[0] = acc[0];  //Getting data from x accelerations
  AverageAccelerationX = 0;
  for (uint16_t averagingelement = 0; averagingelement<=49; averagingelement++) {  //Averaging the list of detect x accelerations
    AverageAccelerationX += Xacclist[averagingelement];
  }
  AverageAccelerationX = AverageAccelerationX / 50; //50 is elements in moving average filter

  for (uint16_t element = 0; element<=48; element++) {  //shifting the window of detected accelerations for Y
    Yacclist[element + 1] = Yacclist[element];
  }
  Yacclist[0] = acc[1]; //Getting data from y accelerations
  AverageAccelerationY = 0;
  for (uint16_t averagingelement = 0; averagingelement<=49; averagingelement++) {  //Averaging the list of detect y accelerations
    AverageAccelerationY += Yacclist[averagingelement];
  }
  AverageAccelerationY = AverageAccelerationY / 50;

  for (uint16_t element = 0; element<=48; element++) {  //shifting the window of detected accelerations for Z orientations
    AngaccZList[element + 1] = AngaccZList[element];
  }
  AngaccZList[0] = ori[2]; //Getting data
  AverageAngZ = 0;
  for (uint16_t averagingelement = 0; averagingelement<=49; averagingelement++) {  //Averaging the list of detect Z orientations
    AverageAngZ += AngaccZList[averagingelement];
  }
  AverageAngZ = AverageAngZ / 50;
  Serial.println("List");
  Serial.println(double(acc[0]));
  Serial.println(double(acc[1]));
  // Serial.println(double(acc[2]));
  Serial.println("-");
  // Serial.println(double(ori[0]));
  // Serial.println(double(ori[1]));
  Serial.println(double(ori[2]));
  Serial.println(AverageAccelerationX); 
  Serial.println(AverageAccelerationY); 
  Serial.println(AverageAngZ); 
  Serial.println(Yacclist[0]);
  Serial.println(AngaccZList[0]);
  Serial.println(Xacclist[49]);
  Serial.println(Yacclist[49]);
  Serial.println(AngaccZList[49]);


  double Timedif = float(millis())/1000 - prevtime;
  Serial.println("Timedif:");
  Serial.println(Timedif);
  prevtime = millis()/1000;  //Time difference for integration

  CurrentposX += (int(AverageAccelerationX) * pow(Timedif, 2));  //Getting the x position from moving average filter, need pow function in Arduino for powers, differentiation
  Serial.print("X: ");
  Serial.print(CurrentposX);
  CurrentposY += (int(AverageAccelerationY) * pow(Timedif, 2));  //y
  Serial.print("Y: ");
  Serial.print(CurrentposY);
  CurrentOrienZ = int(AverageAngZ);  //orientation in Z
  Serial.print("  OrienZ: ");
  Serial.print(AverageAngZ);
  Serial.print("\n");

  //Encoder Part, will rely on the IMU in turns      
  if ((lastReportedPos1 != encoderPos1)||(lastReportedPos2 != encoderPos2)) 
  {
    Serial.print("Movement");
    lastReportedPos1 = encoderPos1;
    lastReportedPos2 = encoderPos2;
  }

  Serial.println("Encoder Data: ");
  Serial.println(lastReportedPos2);
  Serial.println(lastReportedPos1);
  if (((lastReportedPos1-lastReportedPos2) > 10) || ((lastReportedPos2-lastReportedPos1) > 10)) {
    //Set IMU Positions to 0, and mTravels to 0
    Serial.print("IMU used in turns for angles");
    //IMU gets 
    
    StraightTotalTravelxperp += 0; //can use different trig or import from IMU
    StraightTotalTravelypara += 0;
  } else {
    Serial.print("Index:");
    Serial.print(encoderPos1, DEC); //These are in counts which are Pulses Per Revolution*4
    double mTravelEncoderLeft = (encoderPos1/4)*(0.421/663);
    Serial.print(":");
    Serial.print(encoderPos2, DEC);
    double mTravelEncoderRight = (encoderPos2/4)*(0.421/663);
    Serial.println();
    StraightTotalTravelxperp += (mTravelEncoderLeft+mTravelEncoderRight)/2*sin(double(CurrentOrienZ)); //Should accumulate every iteration
    StraightTotalTravelypara += (mTravelEncoderLeft+mTravelEncoderRight)/2*cos(double(CurrentOrienZ));
  }
  double mTravelEncoderLeft = 0;
  double mTravelEncoderRight = 0;
  Serial.println("Overall positions");
  Serial.println(StraightTotalTravelxperp);
  Serial.println(StraightTotalTravelypara);
  lastReportedPos1 = 0;
  lastReportedPos2 = 0;
  encoderPos1 = 0;
  encoderPos2 = 0;
  

  //Color sensor detection
  colorSensorDetect(colorlist);
  Serial.print("Color: ");
  Serial.print(colorlist[0]);
  Serial.print("\t");
  Serial.print(colorlist[1]);
  Serial.print("\t");
  Serial.print(colorlist[2]);
  Serial.print("\t");
  Serial.print(colorlist[3]);
  Serial.print("\t");

  if (elapsed_time==0) {
    colorStart();
    Startangle = AverageAngZ;
  }

  
  
  //Electromagnet status
  Serial.print("ElectroMagnet Status: ");
  Serial.print(ElectroMagnet1On);
  Serial.print(ElectroMagnet2On);
  Serial.print(ElectroMagnet3On);
  Serial.print("     ");

  //State machine

  if (elapsed_time>100) {
    programState == 3;
  } else if (((MiddleLeft-BottomLeft>50) || (MiddleRight-BottomRight>50) || (InductionDetected == 1)) && (elapsed_time <= 100)) { //Object like weight is found
    programState = 2;
  } else if ((TopLeft < 20 || TopRight < 20 || TopMiddle < 20) && (elapsed_time <= 100)) { //state to turn
    programState = 1;
  } else if ((elapsed_time > 100) || (ElectroMagnet1On && ElectroMagnet2On && ElectroMagnet3On)) { //state to go home
    programState = 3;
  }
  Serial.print(elapsed_time);
  Serial.print("\t");

  // if (digitalRead(CollectPin)==1) { //For collection if weights still have to be in robot
  //   programState=4;
  //   analogWrite(ElectroMagnet1Pin, 50);
  //   analogWrite(ElectroMagnet2Pin, 50);
  //   analogWrite(ElectroMagnet3Pin, 50);
  //   ElectroMagnet1On = true;
  //   ElectroMagnet2On = true;
  //   ElectroMagnet3On = true;
  //   Serial.print("4 Activated  ");
  //   while(1){
  //     stop(timedelay);
  //   }
  // }


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
    digitalWrite(LED1,HIGH); //Green top set, on
    full_forward(timedelay); //can go full forward
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
  

  if (programState == 1) {
    digitalWrite(LED1,LOW);
    if (TopLeft < 20 && TopMiddle > 20 && TopRight > 20) { //&& PrevturnTime-millis()>2000
      full_turn_right(timedelay);
      digitalWrite(LED3,LOW);
      digitalWrite(LED2,HIGH); //Yellow
      Serial.print(" Right");
    } else if (TopLeft > 20 && TopMiddle > 20 && TopRight < 20) { //&& PrevturnTime-millis()>2000
      full_turn_left(timedelay);
      digitalWrite(LED3,LOW);
      digitalWrite(LED2,HIGH); //YELLOW
      Serial.print(" Left");
    } else if (TopLeft > 20 && TopMiddle > 20 && TopRight > 20) { //open space
      digitalWrite(LED3,LOW); //off
      digitalWrite(LED2,LOW); //off
      programState = 0;
    } else if (TopLeft > 20 && TopMiddle < 20 && TopRight > 20) { //A thin slab of wall in front of the robot
      digitalWrite(LED3,LOW); //off
      digitalWrite(LED2,HIGH); //off
      if (TopLeft<TopRight) { 
        full_turn_right(timedelay);
        Serial.print("Slab Turn Right");
      } else {
        full_turn_left(timedelay);
        Serial.print("Slab Turn Left");
      }
    } else if (TopLeft < 20 && TopMiddle < 20 && TopRight < 20) { //Solid Wall ahead, IMU will do this
      if (TopLeft<TopRight) {
        full_turn_right(timedelay);
        Serial.print("Solid Wall Turn Right");
      } else {
        full_turn_left(timedelay);
        Serial.print("Solid Wall Turn Left");
      }
      Serial.print("180 degree turn");
    } else if (TopLeft < 20 && TopMiddle > 20 && TopRight < 20) {
      full_forward(timedelay);
      Serial.print("Space in Between");
    } else { //A large wall in front of just more than half of the robot
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,HIGH); //Red stopped
      if (TopLeft<TopRight) {
        full_turn_right(timedelay);
        Serial.print("Wall Turn Right");
      } else {
        full_turn_left(timedelay);
        Serial.print("Wall Turn Left");
      }
    }
  } else if (programState == 2) {
    //going to and picking up weights. ISR may not be needed due to TOF angular range
    if (TopLeft < 20 && TopRight > 20) {  //If walls to the sides are found by the top TOF sensors
      full_turn_left(timedelay);
      digitalWrite(LED3,LOW);
      digitalWrite(LED2,HIGH); //Yellow
      Serial.print("To weight avoid right");
    }
    if (TopLeft > 20 && TopRight < 20) {
      full_turn_right(timedelay);
      digitalWrite(LED3,LOW);
      digitalWrite(LED2,HIGH); //YELLOW
      Serial.print("To weight avoid left");
    }

    

    if (InductionDetected == 1) {
      if ((StepperPosition>0) && (StepperState == 0)) //Steppers down
      {
        digitalWrite(MAdirpin,LOW); //HIGH for up, LOW for down
        digitalWrite(MBdirpin,LOW);
        Serial.print("\tStepper going down");
        for(int j=1000;j>0;j--)            //Move 1000 steps
        {
          digitalWrite(MAsteppin,LOW);
          digitalWrite(MBsteppin,LOW);
          delayMicroseconds(20);
          digitalWrite(MAsteppin,HIGH);
          digitalWrite(MBsteppin,HIGH);
          delay(1);
        }
        //delay(1);
        StepperPosition-=1000;
        Serial.print(StepperPosition);
      } else if ((StepperPosition<=0) && (StepperState == 0)) { //When Steppers are fully down
        StepperState = 1;
        Serial.print("\tTurning Magnets on\t");
        if (ElectroMagnet1On == true) { //Relevant Electromagnet on
          if (ElectroMagnet2On == true) {
            if (ElectroMagnet3On == false) {
              analogWrite(ElectroMagnet3Pin, 0.6*255);  //the front 3rd electromagnet
              ElectroMagnet3On = true;
            }
          } else {
            analogWrite(ElectroMagnet2Pin, 0.6*255); //90% duty cycle
            ElectroMagnet2On = true;
          }
        } else {
          analogWrite(ElectroMagnet1Pin, 0.6*255);
          ElectroMagnet1On = true;
        }
        half_forward(timedelay*5);
        StepperState = 2;
      }
      if (StepperState == 2) {
        Serial.print("\tStepper going up\t");
        digitalWrite(MAdirpin,HIGH); //Steppers up
        digitalWrite(MBdirpin,HIGH);
        if (StepperPosition<14000)            //Move 1000 steps
        {
          for(int j=0;j<=1000;j++)            //Move 1000 steps
          {
            digitalWrite(MAsteppin,LOW);
            digitalWrite(MBsteppin,LOW);
            delayMicroseconds(20);
            digitalWrite(MAsteppin,HIGH);
            digitalWrite(MBsteppin,HIGH);
            delay(1);
          }
          StepperPosition+=1000;
          Serial.print(StepperPosition);
        } else { //When steppers are fully up
          programState=0;
          InductionDetected = 0;
          StepperState = 0;
        }
      }
    } else if ((MiddleLeft-BottomLeft>50) || (MiddleRight-BottomRight>50)) { //If weight found condition is still satisfied
      if ((MiddleRight - BottomRight)<(MiddleLeft - BottomLeft) && (BottomLeft>20)) {
        full_turn_left(timedelay); //should turn to the closer sensor where the weight is
        Serial.print("\tTo weight left");
      } else if ((MiddleRight - BottomRight)>(MiddleLeft - BottomLeft) && (BottomRight>20)) {
        full_turn_right(timedelay);
        Serial.print("\tTo weight right");
      } else if ((BottomRight<20 || BottomLeft<20)  && (MiddleRight> 20 && MiddleLeft> 20)) { //If weight is within 20cm from bottom sensors
        Serial.print("\t20 cm from weight");
        half_forward(timedelay);
        //Check inductive proximity sensor and if it activates then drive the relevant electromagnet
        if (digitalRead(InductionPin) == 0) { //Checks to read
          Serial.print("Induction detected Weight");
          InductionDetected = 1;
        }
      }
    } else { //If it looses the weight on the sensors
      programState = 0;
    } 
  } else if (programState == 4) {
    Serial.println("Going Home");
    if ((ColorCompareHome() == true) || (FoundHome == 1)) {
      FoundHome == 1;
      stop(timedelay);
    }
    if (FoundWall == 0 && FoundHome == 0) {
      if (TopRight>20 && TopLeft>20) {
        Serial.print("Moving Forward Home");

        int homeangle_to_robot = int(atan2(StraightTotalTravelypara, StraightTotalTravelxperp) * 180 / PI); // Calculate angle to home
        homeangle_to_robot += Startangle; // Adjust by initial angle (Startangle)

        Serial.println(Startangle);
        Serial.println(homeangle_to_robot);

        // Adjust the home angle to point the robot toward home
        int heading_home_angle = homeangle_to_robot - 180;

        // Wrap the heading_home_angle to stay within 0-360 degrees
        if (heading_home_angle < 0) {
          heading_home_angle += 360;
        } else if (heading_home_angle >= 360) {
          heading_home_angle -= 360;
        }

        // Determine if we need to turn left, right, or move forward
        int angle_diff = AverageAngZ - heading_home_angle; // Calculate difference between current angle and target home heading

        // Wrap angle_diff to handle circular nature of angles
        if (angle_diff > 180) {
          angle_diff -= 360; // Ensure it's within -180 to 180 range
        } else if (angle_diff < -180) {
          angle_diff += 360;
        }

        // Decision making: Turn left, right, or move forward based on the angle difference
        if (angle_diff > 10) {
          full_turn_left(timedelay); // If too far right, turn left
        } else if (angle_diff < -10) {
          full_turn_right(timedelay); // If too far left, turn right
        } else {
          full_forward(timedelay); // If within the desired range, move forward
        }
      }
      if (TopRight < 20 && TopMiddle<50) {
        full_turn_left(timedelay);
      } else if (TopLeft < 20 && TopMiddle<50) {
        full_turn_right(timedelay);
      }
      if (TopRight < 30 && TopMiddle>50){
        FoundWall = 1; //If a right wall is found
      }
      if (TopLeft < 30 && TopMiddle>50){
        FoundWall = 2; //If a left wall is found
      }
    } else if (FoundWall == 1 && FoundHome == 0) {
      full_forward(timedelay);
      if (FoundWall = 1) {
        if (TopRight > 30){
          full_turn_right(timedelay);
        } else if (TopRight<5) {
          full_turn_left(timedelay);
        }
      } else {
        if (TopLeft > 30){
          full_turn_left(timedelay);
        } else if (TopLeft<5) {
          full_turn_right(timedelay);
        }
      }
    }
  }
  Serial.println(); //Next line
}

// Interrupt on A changing state
void doEncoder1A(){
  // Test transition
  A_set1 = digitalRead(encoder1PinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPos1 += (A_set1 != B_set1) ? +1 : -1;
  
  B_set1 = digitalRead(encoder1PinB) == HIGH;
  // and adjust counter + if B follows A
  encoderPos1 += (A_set1 == B_set1) ? +1 : -1;
}


// Interrupt on A changing state
void doEncoder2A(){
  // Test transition
  A_set2 = digitalRead(encoder2PinA) == HIGH;
  // and adjust counter + if A leads B
  encoderPos2 += (A_set2 != B_set2) ? +1 : -1;
  
   B_set2 = digitalRead(encoder2PinB) == HIGH;
  // and adjust counter + if B follows A
  encoderPos2 += (A_set2 == B_set2) ? +1 : -1;
}