#include <Wire.h> //TOF Setup
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3F;
#define VL53L1X_ADDRESS_START 0x30


// The number of sensors in your system.
const uint8_t sensorCount = 1;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[8] = {0,1,2,3,4,5,6,7};

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

int time = 10; //time in milliseconds
static long durationA,durationB, Acm,Bcm;



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
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


//Color Sensor setup
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


//TOF setup
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3F;
#define VL53L1X_ADDRESS_START 0x30

// The number of sensors in your system.
const uint8_t sensorCount = 1;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[8] = {0,1,2,3,4,5,6,7};

SX1509 io; // Create an SX1509 object to be used throughout
VL53L1X sensors[sensorCount];

 
void setup()
{   
  myservoA.attach(0);  // attaches the servo  to the servo object useing pin 0
  myservoB.attach(1);  // attaches the servo  to the servo object useing pin 1

  pinMode(AtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(AechoPin, INPUT);

  pinMode(BtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(BechoPin, INPUT);
  
  Serial.begin(9600);                  // initialize serial communication:
  
  digitalWrite(AtrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);

}

void full_reverse(int time) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_reverse_speed);

  delay(time);
}

void stop(int time) {
  myservoA.writeMicroseconds(stop_speed);
  myservoB.writeMicroseconds(stop_speed);

  delay(time);
}

void full_forward(int time) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_forward_speed);

  delay(time); 
}

void full_turn_left(int time) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(time);
}

void directioncheck_left(int time) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(time*3); //Will be replaced by IMU code

}

void full_turn_right(int time) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(time);  
}

long A_read(void) {
  digitalWrite(AtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(AtrigPin, LOW);
  durationA = pulseIn(AechoPin, HIGH);

}

long B_read(void) {
  digitalWrite(BtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(BtrigPin, LOW);
  durationB = pulseIn(BechoPin, HIGH);
}

//int colorRead() {

//}

void IMUsetup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void TOFL1setup()
{
  while (!Serial) {}
  Serial.begin(115200);

  io.begin(SX1509_ADDRESS);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

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
}



void IMUprintEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
} 

int IMUGetPos(void) {
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER); //degrees
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE); //rad/s
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL); //m/s^2
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER); 
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER); //Gravity detected
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY); //Gravity preset

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

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

  //return [&orientationData, &angVelocityData, &linearAccelData, &magnetometerData, &accelerometerData, &gravityData]
  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

//InitColorReading = 
IMUSetup();
//TOFL1Setup();
//InitpositionX = SENSOR_TYPE_ACCELEROMETER->orientation.x
int CurrentposX = 0;
int CurrentposY = 0;
int* Xposlist[50] = {0};
int* Yposlist[50] = {0};
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
int* OrienZList[50] = {0};

clock_t start_time, end_time;
double elapsed_time;
const double two_minutes_in_seconds = 120.0;
// Record the start time
start_time = clock();
int8_t programState = 0; //0 is moving around, no weight detected, 1 is wall is detected, 2 is weight detected, 3 is returning back home

void loop() 
{ 
  time_elapsed = clock()-start_time; //Could make an interrupt for the clock
  //Getting current acceleration and therefore position:
  //CurrentOrienX = SENSOR_TYPE_ACCELEROMETER->orientation.x
  //CurrentOrienY = SENSOR_TYPE_ACCELEROMETER->orientation.y
    
  //Have weighted average of IMU and Encoder
    
    
  for (element=1; element<49; element++){                 //shifting the window of detected accelerations for X
    if (element == 48) {
      Xposlist[element] = 0;
    } else {
      Xposlist[element+1] = Xposlist[element+1];
    }
  }
  Xposlist[0]=SENSOR_TYPE_LINEAR_ACCELERATION->acceleration.x;
  for (averagingelement = 0; averagingelement < 49; averagingelement++) {  //Averaging the list of detect x accelerations
    AverageAccelerationX += Xposlist[averagingelement];
    AverageAccelerationX = AverageAccelerationX / 50;
  }

  for (element = 1; element < 49; element++) {  //shifting the window of detected accelerations for Y
    if (element == 48) {
      Yposlist[element] = 0;
    } else {
      Yposlist[element + 1] = Yposlist[element + 1];
    }
  }
  Yposlist[0] = SENSOR_TYPE_LINEAR_ACCELERATION->acceleration.y;
  for (averagingelement = 0; averagingelement < 49; averagingelement++) {  //Averaging the list of detect y accelerations
    AverageAccelerationY += Yposlist[averagingelement];
    AverageAccelerationY = AverageAccelerationY / 50;
  }

  for (element = 1; element < 49; element++) {  //shifting the window of detected accelerations for Z orientations
    if (element == 48) {
      OrienZlist[element] = 0;
    } else {
      OrienZlist[element + 1] = OrienZlist[element + 1];
    }
  }
  OrienZlist[0] = SENSOR_TYPE_ORIENTATION->orientation.z;
  for (averagingelement = 0; averagingelement < 49; averagingelement++) {  //Averaging the list of detect Z orientations
    AverageOrienZ += OrienZlist[averagingelement];
    AverageOrienZ = AverageOrienZ / 50;
  }

  Timedif = clock() - prevtime;
  Serial.print(Timedif);
  prevtime = clock();  //Time difference for integration

  CurrentposX += AverageAccelerationX * (Timedif * 50) * *2;  //Getting the x position from moving average filter
  Serial.print("X: ");
  Serial.print(CurrentposX);
  CurrentposY += AverageAccelerationY * (Timedif * 50) * *2;  //y
  Serial.print("Y: ");
  Serial.print(CurrentposY);
  CurrentOrienZ = AverageOrienZ;  //orientation in Z
  Serial.print("OrienZ: ");
  Serial.print(AverageOrienZ);

  if (time_elapsed < 100) {
    if (programState == 0) {
      full_forward(time); //can go full forward
      //Looks for walls with ultrasonic sensor, NEED TO MAKE ISR for these
      A_read();
      B_read();

      Acm = microsecondsToCentimeters(durationA);
      Bcm = microsecondsToCentimeters(durationB);

      Serial.print(Acm);
      Serial.print(" ");
      Serial.print(Bcm);
      Serial.println();


      //to Avoid Objects:
      //if(Acm > 3300 || Bcm > 3300 ) {
        //full_forward(time);
      if (Acm<20 || Bcm<20) {
        programState = 1;
      }
    }
    

    else if (programState == 1) {
      if (Acm < 20 && Bcm > 20) {
        full_turn_left(time);
      } else if(Acm > 20 && Bcm < 20) {
        full_turn_right(time);
      } else if (Acm > 20 && Bcm > 20) {
        programState = 0;
      //} else if (Weightdetected) {
        //programState = 2;
      } else {
        stop(time);




        //If robot is against a wall where does it go? The following is future code for later if two ultrasonic sensors are used
        //int directionCheckarray[20] = {0}; //An array of distances as the robot rotates when it detects a wall
        //int8_t it_num = 0;
        //int32_t MaxCM = 0;
        //int8_t MaxCMElement = 0;
        //if (Acm>Bcm) {
          //full_turn_left(time);
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
            //full_turn_left(time);
          //for (i=0; i<(it_num-MaxCMElement), i++){
            //full_turn_right(time);
          //}
        //} else {
          //full_turn_right(time);
        //}
        //programState = 0;
      programState = 0;
    } //else if (programState == 2) {

    //}
  }


    //Detecting weights using lower TOF:
    //If an object is found closer than the ultrasonic distance, it is an interrupt with a higher proiority than the turning process, will stop rotating and go forward at full power, then leaves the interruyptr


  } else {
    //In the state to find the home base
    programState = 3;
    Serial.print("Must find home base");
  }
  //PrevPositionX = CurrentposX;
  //PrevOrienZ = CurrentOrienZ;
} 
