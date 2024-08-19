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

//Can't call functions globally, Global variables are set up here
//InitColorReading =
int CurrentposX = 0;
int CurrentposY = 0;
int Xposlist[50] = { 0 };
int Yposlist[50] = { 0 };
int AverageAccelerationX = 0;
int AverageAccelerationY = 0;
//int CurrentposZ = 0;
//int PrevPositionX = 0;
//int PrevpositionY = 0;
//int* OrienlistX;
//int* OrienlistY;
int prevtime = 0;
//int PrevOrienZ = 0;
//int CurrentOrienX;
//int CurrentOrienY;
int CurrentOrienZ = 0;
int AverageOrienZ = 0;
int OrienZlist[50] = { 0 };

const int ElectroMagnet1Pin = A12; //Electromagnet pins
const int ElectroMagnet2Pin = A0;
const int ElectroMagnet3Pin = A10;

//This means the pins, not entire ports which the cables connect to
const int InductionPin = 26; //Induction sensor pin to check

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
const uint8_t sensorCountL0 = 2;  //sensorcount, 2 L1s, 2 l0s
const uint8_t sensorCountL1 = 2;  //sensorcount, 2 L1s, 2 l0s

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL1[sensorCountL1] = {0, 1};  //Only two needed for two sensors, xshut ports
const uint8_t xshutPinsL0[sensorCountL0] = {4, 7};  //Only two needed for two sensors, xshut ports

SX1509 io;  // Create an SX1509 object to be used throughout
VL53L1X sensorsL1[sensorCountL1];
VL53L0X sensorsL0[sensorCountL0];


//Motor setup
Servo myservoA, myservoB;  // create servo object to control a servo
int stop_speed = 1500;     // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
int full_forward_speed = 1900;
int full_reverse_speed = 1100;


//Ultrasound setup
// const int AtrigPin = 3;
// const int AechoPin = 2;

// const int BtrigPin = 5;
// const int BechoPin = 4;

int timedelay = 10;  //time in milliseconds, do not comment this out
// static long durationA, durationB, Acm, Bcm;



//IMU setup

// /* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
//    which provides a common 'type' for sensor data and some helper functions.

//    To use this driver you will also need to download the Adafruit_Sensor
//    library and include it in your libraries folder.

//    You should also assign a unique ID to this sensor for use with
//    the Adafruit Sensor API so that you can identify this particular
//    sensor in any data logs, etc.  To assign a unique ID, simply
//    provide an appropriate value in the constructor below (12345
//    is used by default in this example).

//    Connections
//    ===========
//    Connect SCL to analog 5
//    Connect SDA to analog 4
//    Connect VDD to 3.3-5V DC
//    Connect GROUND to common ground

//    History
//    =======
//    2015/MAR/03  - First release (KTOWN)
// */

// /* Set the delay between fresh samples */
// uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// //                                   id, address
// // Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); //searches to find IMU
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

// //Color Sensor setup
// #include <Adafruit_TCS34725.h>

// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);



void setup()  //Need one setup function
{
  //LEDs
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

  //Color sensor
  //digitalWrite(ColorOnPin,HIGH);

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
      while (1)
        ;
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
      Serial.print("Failed to detect and initialize sensor ");
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

  //Motor
  myservoA.attach(0);  // attaches the servo  to the servo object using pin 0
  myservoB.attach(1);  // attaches the servo  to the servo object using pin 1

  Serial.println("Configured Motors");

  //Ultrasound
  // pinMode(AtrigPin, OUTPUT);            //Setup ultrasound pins
  // pinMode(AechoPin, INPUT);

  // pinMode(BtrigPin, OUTPUT);            //Setup ultrasound pins
  // pinMode(BechoPin, INPUT);

  // digitalWrite(AtrigPin, LOW);
  // delayMicroseconds(2);

  // digitalWrite(BtrigPin, LOW);
  // delayMicroseconds(2);

  // Serial.println("Configured Ultrasonics");

  //IMU
  //   Serial.println("Orientation Sensor Test"); Serial.println("");

  //   /* Initialise the sensor */
  //   if (!bno.begin())
  //   {
  //     /* There was a problem detecting the BNO055 ... check your connections */
  //     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //     while (1);
  //   }
  //   Serial.print("IMU detected");
  //  digitalWrite(IMUDetection,HIGH);
  //   delay(1000);


  //   //Inductive sensor
  //   //digitalWrite(InductionOnPin,HIGH);
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

void full_turn_left(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay);
}

void directioncheck_left(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay * 3);  //Will be replaced by IMU code
}

void full_turn_right(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(timedelay);
}

//Following two functions are for ultrasound
// long A_read(void) {
//   digitalWrite(AtrigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(AtrigPin, LOW);
//   durationA = pulseIn(AechoPin, HIGH);
//   return durationA;
// }

// long B_read(void) {
//   digitalWrite(BtrigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(BtrigPin, LOW);
//   durationB = pulseIn(BechoPin, HIGH);
//   return durationB;
// }

//int colorRead() {

//}

// double printEvent(sensors_event_t* event) { //problems getting right event
//   double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
//   if (event->type == SENSOR_TYPE_ORIENTATION) {
//     Serial.print("Orient:");
//     x = event->orientation.x;
//     y = event->orientation.y;
//     z = event->orientation.z;
//   } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//     Serial.print("Linear:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   } else {
//     Serial.print("Unk:");
//   }

//   Serial.print("\tx= ");
//   Serial.print(x);
//   Serial.print(" |\ty= ");
//   Serial.print(y);
//   Serial.print(" |\tz= ");
//   Serial.println(z);
//   Serial.print("\n");
//   double XYZList[3] = {x, y, z};
//   return *XYZList;
// }

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

// sensors_event_t orientationData , accelerometerData; //, angVelocityData , linearAccelData, magnetometerData,  gravityData;
// double ori[3] = {};
// double acc[3] = {}; //comment out if not using IMU

// void IMUGetPos(void) {
//   //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
//   bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); //degrees
//   //bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); //rad/s
//   //bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); //m/s^2
//   //bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
//   bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER); //Gravity detected
//   //bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY); //Gravity preset

//   *ori = printEvent(&orientationData);
//   //printEvent(&angVelocityData);
//   //printEvent(&linearAccelData);
//   //printEvent(&magnetometerData);
//   *acc = printEvent(&accelerometerData);
//   //printEvent(&gravityData);

//   int8_t boardTemp = bno.getTemp();
//   Serial.println();
//   Serial.print(F("temperature: "));
//   Serial.println(boardTemp);

//   uint8_t system, gyro, accel, mag = 0;
//   bno.getCalibration(&system, &gyro, &accel, &mag);
//   Serial.println();
//   Serial.print("Calibration: Sys=");
//   Serial.print(system);
//   Serial.print(" Gyro=");
//   Serial.print(gyro);
//   Serial.print(" Accel=");
//   Serial.print(accel);
//   Serial.print(" Mag=");
//   Serial.println(mag);

//   Serial.println("--");

//   //delay(BNO055_SAMPLERATE_DELAY_MS);
// }



void loop() {
  elapsed_time = millis()/1000; //Seconds since program has been running
  //Getting current acceleration and therefore position:
  //CurrentOrienX = SENSOR_TYPE_ACCELEROMETER->orientation.x
  //CurrentOrienY = SENSOR_TYPE_ACCELEROMETER->orientation.y

  //Have weighted average of IMU and Encoder


  // for (uint16_t element=1; element<49; element++){                 //shifting the window of detected accelerations for X
  //   if (element == 48) {
  //     Xposlist[element] = 0;
  //   } else {
  //     Xposlist[element+1] = Xposlist[element+1];
  //   }
  // }
  // IMUGetPos();
  // Xposlist[0] = acc[0];
  // for (uint16_t averagingelement = 0; averagingelement < 49; averagingelement++) {  //Averaging the list of detect x accelerations
  //   AverageAccelerationX += Xposlist[averagingelement];
  //   AverageAccelerationX = AverageAccelerationX / 50;
  // }

  // for (uint16_t element = 1; element < 49; element++) {  //shifting the window of detected accelerations for Y
  //   if (element == 48) {
  //     Yposlist[element] = 0;
  //   } else {
  //     Yposlist[element + 1] = Yposlist[element + 1];
  //   }
  // }
  // Yposlist[0] = acc[1];
  // for (uint16_t averagingelement = 0; averagingelement < 49; averagingelement++) {  //Averaging the list of detect y accelerations
  //   AverageAccelerationY += Yposlist[averagingelement];
  //   AverageAccelerationY = AverageAccelerationY / 50;
  // }

  // for (uint16_t element = 1; element < 49; element++) {  //shifting the window of detected accelerations for Z orientations
  //   if (element == 48) {
  //     OrienZlist[element] = 0;
  //   } else {
  //     OrienZlist[element + 1] = OrienZlist[element + 1];
  //   }
  // }
  // OrienZlist[0] = ori[2];
  // for (uint16_t averagingelement = 0; averagingelement < 49; averagingelement++) {  //Averaging the list of detect Z orientations
  //   AverageOrienZ += OrienZlist[averagingelement];
  //   AverageOrienZ = AverageOrienZ / 50;
  // }

  // int Timedif = millis()/1000 - prevtime;
  // Serial.print(Timedif);
  // prevtime = millis()/1000;  //Time difference for integration

  // CurrentposX += (AverageAccelerationX * pow(Timedif * 50, 2));  //Getting the x position from moving average filter, need pow function in Arduino for powers
  // Serial.print("X: ");
  // Serial.print(CurrentposX);
  // CurrentposY += (AverageAccelerationY * pow(Timedif * 50, 2));  //y
  // Serial.print("Y: ");
  // Serial.print(CurrentposY);
  // CurrentOrienZ = AverageOrienZ;  //orientation in Z
  // Serial.print("  OrienZ: ");
  // Serial.print(AverageOrienZ);
  // Serial.print("\n");


  uint16_t Acm = sensorsL1[0].read()/10;  //Long range TOF reads
  uint16_t Bcm = sensorsL1[1].read()/10;
  uint16_t Ccm = sensorsL0[0].readRangeContinuousMillimeters()/10;
  uint16_t Dcm = sensorsL0[1].readRangeContinuousMillimeters()/10;

  Serial.print(Ccm);
  if (sensorsL0[0].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print('\t');
  Serial.print(Dcm);
  if (sensorsL0[1].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
    Serial.print('\t');

  Serial.print(Acm);
  if (sensorsL1[0].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print('\t');

  Serial.print(Bcm);
  if (sensorsL1[1].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
    Serial.print('\t');

  //Looks for walls with TOFs
  // for (uint8_t i = 0; i < sensorCountL0; i++)
  // {
  //   Serial.print(sensorsL0[i].readRangeContinuousMillimeters());
  //   if (sensorsL0[i].timeoutOccurred()) { Serial.print(" TIMEOUT L0"); }
  //   Serial.print('\t');
  // }

  // for (uint8_t i = 0; i < sensorCountL1; i++)
  // {
  //   Serial.print(sensorsL1[i].read());
  //   if (sensorsL1[i].timeoutOccurred()) { Serial.print(" TIMEOUT L1"); }
  //     Serial.print('\t');
  // }
  Serial.println();



  if (elapsed_time < 100) {
    if (programState == 0) {
      
      digitalWrite(LED1,HIGH); //Green top set, on
      Serial.print("State 0\n");
      full_forward(timedelay); //can go full forward



      if (Acm<20 || Bcm<20) {
        programState = 1;
      }
    } else if (programState == 1) {
      Serial.print("State 1\n");
      digitalWrite(LED1,LOW);
      if (Acm < 20 && Bcm > 20) {
        full_turn_left(timedelay);
        digitalWrite(LED3,LOW);
        digitalWrite(LED2,HIGH); //Yellow
        Serial.print("Left\n");
      } else if(Acm > 20 && Bcm < 20) {
        full_turn_right(timedelay);
        digitalWrite(LED3,LOW);
        digitalWrite(LED2,HIGH); //YELLOW
        Serial.print("Right\n");
      } else if (Acm > 20 && Bcm > 20) {
        digitalWrite(LED3,LOW); //off
        digitalWrite(LED2,LOW); //off
        programState = 0;
        Serial.print("Open space ahead\n");
      } else {
        stop(timedelay);
        Serial.print("Wall\n");
        digitalWrite(LED2,LOW);
        digitalWrite(LED3,HIGH); //Red stopped
        if (Acm<Bcm) {
          full_turn_left(timedelay);
          if (Acm<20 || Bcm<20) {
            full_turn_left(timedelay*6)
          }
        } else {
          full_turn_right(timedelay);
          if (Acm<20 || Bcm<20) {
            full_turn_left(timedelay*6)
          }
        }
        //If TOFs on side both have objects and ones on front have object in front, then it rorates 180 degrees and heads out
      }

      if (Acm-Ccm>50 || Bcm-Dcm>50 && Bcm>Dcm || Acm>Ccm) { //Object lower than weight is found
        programState = 2;
      }


        //If robot is against a wall where does it go? The following is future code for later if two ultrasonic sensors are used
        //int directionCheckarray[20] = {0}; //An array of distances as the robot rotates when it detects a wall
        //int8_t it_num = 0;
        //int32_t MaxCM = 0;
        //int8_t MaxCMElement = 0;
        //if (Acm>Bcm) {
          //full_turn_left(timedelay);
          //Get IMU heading
          //if IMU heading is less than 360 degrees
            //A_read();
            //B_read();
            //directionCheckarray[it_num] = [(Acm+Bcm)/2];
            //if ((Acm+Bcm)/2 > (MaxCMElement)){
              //MaxCM = (Acm+Bcm)/2;
              //MaxCMElement = Acm;
            //}
            //it_num++;
            //full_turn_left(timedelay);
          //for (i=0; i<(it_num-MaxCMElement), i++){
            //full_turn_right(timedelay);
          //}
        //} else {
          //full_turn_right(timedelay);
        //}
        //programState = 0;

      //Detecting weights using lower TOF:
      //If an object is found closer than the ultrasonic distance, it is an interrupt with a higher proiority than the turning process, will stop rotating and go forward at full power, then leaves the interruyptr
    } else if (programState == 2) {
      //going to and picking up weights. ISR may not be needed due to TOF angular range

      if (Acm < 20 && Bcm > 20) {  //If walls to the sides are found by the top TOF sensors
        full_turn_left(timedelay);
        digitalWrite(LED3,LOW);
        digitalWrite(LED2,HIGH); //Yellow
        full_forward(timedelay/16);
        full_turn_right(timedelay);
        Serial.print("Left\n");
      }
      if (Acm > 20 && Bcm < 20) {
        full_turn_right(timedelay);
        digitalWrite(LED3,LOW);
        digitalWrite(LED2,HIGH); //YELLOW
        full_forward(timedelay/16);
        full_turn_left(timedelay);
        Serial.print("Right\n");
      }

      if (Acm-Ccm>50 || Bcm-Dcm>50 && Bcm>Dcm || Acm>Ccm) { //If weight found condition is still satisfied
          if (Ccm<Dcm && Ccm>20 && Dcm>20) {
          full_turn_left(timedelay/16); //should turn to the closer sensor
          full_forward(timedelay/16);
        } else if (Ccm>Dcm && Ccm>20 && Dcm>20) {
          full_turn_right(timedelay/16);
          full_forward(timedelay/16);
        } else if (Ccm<20 && Dcm<20) {
          full_forward(timedelay);
          //Check inductive proximity sensor and if it activates then drive the relevant electromagnet
          if (digitalRead(InductionPin) == 0) {
            if (ElectroMagnet1On == True) {
              if (ElectroMagnet2On == True) {
                if (ElectroMagnet3On == False) {
                  digitalWrite(ElectroMagnet13Pin, HIGH);  //the front 3rd electromagnet
                }
              }
              digitalWrite(ElectroMagnet2Pin, HIGH);
            }
            digitalWrite(ElectroMagnet1Pin, HIGH);
          }
        }
      } else {
        programState = 0;
      }
      
    }
  } else {
    //In the state to find the home base
    programState = 3;
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    digitalWrite(LED3,LOW);
    digitalWrite(LED4,HIGH); //Green bottom set
    //digitalWrite(State2LEDpin,LOW);
    //digitalWrite(State01LEDpin,LOW);
    //digitalWrite(State3LEDpin,HIGH);
    Serial.print("Must find home base\n");
  }
  //PrevPositionX = CurrentposX;
  //PrevOrienZ = CurrentOrienZ;
}




//sensors_event_t orientationData;
//bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//or_x = orientationData->orientation.x
//or_y = orientationData->orientation.y
//or_z = orientationData->orientation.z
