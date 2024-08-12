//Setup of LEDs
// int State01LEDpin = 
// int State2LEDpin = 
// int State3LEDpin = 

// int LeftorRightProximityPin = 
// int LowerTOFWeightDetection = 
// int IMUDetection = 

// int TOFOnPin = 
// int InductionOnPin = 
// int ColorOnPin = 

// int MotorsNeededPin =
// int ElectroMagnetNeededPin = 
// int StepperNeededPin = 



#include <Wire.h> //TOF Setup
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3F;
#define VL53L1X_ADDRESS_START 0x30

// The number of sensors in your system.
const uint8_t sensorCount = 2;  //sensorcount, 2 L1s

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {3, 4};  //Only two needed for two sensors, xshut ports

SX1509 io; // Create an SX1509 object to be used throughout
VL53L1X sensors[sensorCount];


#include <stdio.h> //Motor setup
#include <time.h>  

//Motor Setup
#include <Servo.h>

Servo myservoA,myservoB;     // create servo object to control a servo
int stop_speed = 1500;        // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
int full_forward_speed = 1900;
int full_reverse_speed = 1100;


//Ultrasound setup
const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;

int timedelay = 10; //time in milliseconds
static long durationA, durationB, Acm,Bcm;



//IMU setup
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

//Color Sensor setup
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);




 
void setup() //Need one setup function
{ 
  //LEDs
  // pinMode(State01LEDpin,OUTPUT);
  //digitalWrite(State01LEDpin,LOW);
  // pinMode(State2LEDpin,OUTPUT);
  //digitalWrite(State2LEDpin,LOW);
  // pinMode(State3LEDpin,OUTPUT);
  //digitalWrite(State3LEDpin,LOW);

  // pinMode(LeftorRightProximityPin,OUTPUT);
  //digitalWrite(LeftorRightProximityPin,LOW);
  // pinMode(LowerTOFWeightDetection,OUTPUT);
  //digitalWrite(LowerTOFWeightDetection,LOW);
  // pinMode(IMUDetection,OUTPUT);
  //digitalWrite(IMUDetection,LOW);

  // pinMode(TOFOnPin,OUTPUT);
  //digitalWrite(TOFOnPin,LOW);
  // pinMode(InductionOnPin,OUTPUT);
  //digitalWrite(InductionOnPin,LOW);
  // pinMode(ColorOnPin,OUTPUT);
  //digitalWrite(ColorOnPin,LOW);

  // pinMode(MotorsNeededPin,OUTPUT);
  //digitalWrite(MotorsNeededPin,LOW);
  // pinMode(ElectroMagnetNeededPin,OUTPUT);
  //digitalWrite(ElectroMagnetNeededPin,LOW);
  // pinMode(StepperNeededPin,OUTPUT);
  //digitalWrite(StepperNeededPin,LOW);

  //Color sensor
  //digitalWrite(ColorOnPin,HIGH);
  
  //TOF
  // while (!Serial) {}    NO USB FOR RUNNING CODE
  // Serial.begin(115200);

  if(!io.begin(SX1509_ADDRESS)){
    Serial.println("Failed to to talk to IO Expander for TOFs");
    while(1) {};
  }

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  Wire1.begin();
  Wire1.setClock(400000); // use 400 kHz I2C

  Serial.println("Setup Wires");

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
    //pinMode(xshutPins[i], OUTPUT);
    //digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPins[i], HIGH);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);

      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x30 + i);

    sensors[i].startContinuous(50);
  }

  Serial.println("Configured TOFs");
  //digitalWrite(TOFOnPin,HIGH);

  //Motor
  myservoA.attach(0);  // attaches the servo  to the servo object using pin 0
  myservoB.attach(1);  // attaches the servo  to the servo object using pin 1

  Serial.println("Configured Motors");

  //Ultrasound
  pinMode(AtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(AechoPin, INPUT);

  pinMode(BtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(BechoPin, INPUT);
  
  digitalWrite(AtrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);

  Serial.println("Configured Ultrasonics");

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


  //Inductive sensor
  //digitalWrite(InductionOnPin,HIGH);
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
  delay(timedelay*3); //Will be replaced by IMU code

}

void full_turn_right(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(timedelay);  
}

long A_read(void) {
  digitalWrite(AtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(AtrigPin, LOW);
  durationA = pulseIn(AechoPin, HIGH);
  return durationA;
}

long B_read(void) {
  digitalWrite(BtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(BtrigPin, LOW);
  durationB = pulseIn(BechoPin, HIGH);
  return durationB;
}

//int colorRead() {

//}

double printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
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
  double XYZList[3] = {x, y, z};
  return *XYZList;
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
} 

sensors_event_t orientationData , accelerometerData; //, angVelocityData , linearAccelData, magnetometerData,  gravityData;
double ori[3] = {};
double acc[3] = {};

void IMUGetPos(void) {
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); //degrees
  //bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); //rad/s
  //bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); //m/s^2
  //bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER); 
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER); //Gravity detected
  //bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY); //Gravity preset

  *ori = printEvent(&orientationData);
  //printEvent(&angVelocityData);
  //printEvent(&linearAccelData);
  //printEvent(&magnetometerData);
  *acc = printEvent(&accelerometerData);
  //printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");

  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

//Can't call functions globally
//InitColorReading = 
int CurrentposX = 0;
int CurrentposY = 0;
int Xposlist[50] = {0};
int Yposlist[50] = {0};
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
int OrienZlist[50] = {0};

double elapsed_time = 0;
const double two_minutes_in_seconds = 120.0;
// Record the start time
int8_t programState = 0; //0 is moving around, no weight detected, 1 is wall is detected, 2 is weight detected, 3 is returning back home

void loop() 
{ 
  elapsed_time = millis()/1000; //Seconds since program has been running
  //Getting current acceleration and therefore position:
  //CurrentOrienX = SENSOR_TYPE_ACCELEROMETER->orientation.x
  //CurrentOrienY = SENSOR_TYPE_ACCELEROMETER->orientation.y
    
  //Have weighted average of IMU and Encoder
    
    
  for (uint16_t element=1; element<49; element++){                 //shifting the window of detected accelerations for X
    if (element == 48) {
      Xposlist[element] = 0;
    } else {
      Xposlist[element+1] = Xposlist[element+1];
    }
  }
  IMUGetPos();
  Xposlist[0] = acc[0];
  for (uint16_t averagingelement = 0; averagingelement < 49; averagingelement++) {  //Averaging the list of detect x accelerations
    AverageAccelerationX += Xposlist[averagingelement];
    AverageAccelerationX = AverageAccelerationX / 50;
  }

  for (uint16_t element = 1; element < 49; element++) {  //shifting the window of detected accelerations for Y
    if (element == 48) {
      Yposlist[element] = 0;
    } else {
      Yposlist[element + 1] = Yposlist[element + 1];
    }
  }
  Yposlist[0] = acc[1];
  for (uint16_t averagingelement = 0; averagingelement < 49; averagingelement++) {  //Averaging the list of detect y accelerations
    AverageAccelerationY += Yposlist[averagingelement];
    AverageAccelerationY = AverageAccelerationY / 50;
  }

  for (uint16_t element = 1; element < 49; element++) {  //shifting the window of detected accelerations for Z orientations
    if (element == 48) {
      OrienZlist[element] = 0;
    } else {
      OrienZlist[element + 1] = OrienZlist[element + 1];
    }
  }
  OrienZlist[0] = ori[2];
  for (uint16_t averagingelement = 0; averagingelement < 49; averagingelement++) {  //Averaging the list of detect Z orientations
    AverageOrienZ += OrienZlist[averagingelement];
    AverageOrienZ = AverageOrienZ / 50;
  }

  int Timedif = millis()/1000 - prevtime;
  Serial.print(Timedif);
  prevtime = millis()/1000;  //Time difference for integration

  CurrentposX += (AverageAccelerationX * pow(Timedif * 50, 2));  //Getting the x position from moving average filter, need pow function in Arduino for powers
  Serial.print("X: ");
  Serial.print(CurrentposX);
  CurrentposY += (AverageAccelerationY * pow(Timedif * 50, 2));  //y
  Serial.print("Y: ");
  Serial.print(CurrentposY);
  CurrentOrienZ = AverageOrienZ;  //orientation in Z
  Serial.print("  OrienZ: ");
  Serial.print(AverageOrienZ);
  Serial.print("\n");

  if (elapsed_time < 100) {
    if (programState == 0) {
      //digitalWrite(State2LEDpin,LOW);
      //digitalWrite(State3LEDpin,LOW);
      //digitalWrite(State01LEDpin,HIGH);
      Serial.print("State 0\n");
      full_forward(timedelay); //can go full forward

      //Looks for walls with TOF
      for (uint8_t i = 0; i < sensorCount; i++)
      {
        Serial.print(sensors[i].read());
        if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
          Serial.print('\t');
        }
      Serial.println();

      uint16_t Acm = sensors[0].read()/10;  //Long range TOF reads
      uint16_t Bcm = sensors[1].read()/10;


      //Looks for walls with ultrasonic sensor, NEED TO MAKE ISR for these
      //A_read();
      //B_read();

      //Acm = microsecondsToCentimeters(durationA);
      //Bcm = microsecondsToCentimeters(durationB);



      //Serial.print(Acm);
      //Serial.print(" ");
      //Serial.print(Bcm);
      //Serial.println();


      //to Avoid Objects:
      //if(Acm > 3300 || Bcm > 3300 ) {
        //full_forward(timedelay);
      if (Acm<20 || Bcm<20) {
        programState = 1;
      }
    } else if (programState == 1) {
      Serial.print("State 0\n");
      if (Acm < 20 && Bcm > 20) {
        full_turn_left(timedelay);
      } else if(Acm > 20 && Bcm < 20) {
        full_turn_right(timedelay);
      } else if (Acm > 20 && Bcm > 20) {
        programState = 0;
      //} else if (Weightdetected) {
        //programState = 2;
      } else {
        stop(timedelay);
        //If TOFs on side both have objects and ones on front have object in front, then it rorates 180 degrees and heads out 



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
      programState = 0;
      //Detecting weights using lower TOF:
      //If an object is found closer than the ultrasonic distance, it is an interrupt with a higher proiority than the turning process, will stop rotating and go forward at full power, then leaves the interruyptr
      } //else if (programState == 2) {
        //going to and picking up weights. ISR may not be needed due to TOF angular range
        //digitalWrite(State01LEDpin,LOW);
        //digitalWrite(State3LEDpin,LOW);
        //digitalWrite(State2LEDpin,HIGH);
      //}
    }
  } else {
    //In the state to find the home base
    programState = 3;
    //digitalWrite(State2LEDpin,LOW);
    //digitalWrite(State01LEDpin,LOW);
    //digitalWrite(State3LEDpin,HIGH);
    Serial.print("Must find home base\n");+
  }
  //PrevPositionX = CurrentposX;
  //PrevOrienZ = CurrentOrienZ;
}




//sensors_event_t orientationData;
//bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//or_x = orientationData->orientation.x
//or_y = orientationData->orientation.y
//or_z = orientationData->orientation.z

