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
#include <Induction_Detected.h>
#include <Motor_Functions.h>
#include <Print_Readings.h>
#include <ScoringMode.h>
#include <Sensor_Readings.h>

double elapsed_time = 0;
const double two_minutes_in_seconds = 120.0;
// Record the start time
int8_t programState = 0;  //0 is moving around, no weight detected, 1 is wall is detected, 2 is weight detected, 3 is returning back home


//Setup of LEDs
const int LED1 = 4;  //Green on top set
const int LED2 = 3;  //Yellow
const int LED3 = 2;  //Red
const int LED4 = 5;  //Green on bottom set

int timedelay = 10;  //time in milliseconds, do not comment this out

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

}


//=========================================================================================================================================================================================================================================================
//=========================================================================================================================================================================
//========================================================================================================================================================================================================================================================
//=========================================================================================================================================================================
//========================================================================================================================================================================================================================================================


void loop() {
  
  elapsed_time = millis()/1000;

  //State machine
  if (((MiddleLeft-BottomLeft>50) || (MiddleRight-BottomRight>50) || (InductionDetected == 1)) && (elapsed_time <= 100)) { //Object like weight is found
    programState = 2;
  } else if ((TopLeft < 20 || TopRight < 20 || TopMiddle < 20) && (elapsed_time <= 100)) { //state to turn
    programState = 1;
  } else if ((elapsed_time > 100) || (ElectroMagnet1On && ElectroMagnet2On && ElectroMagnet3On)) { //state to go home
    programState = 3;
  }
  

  // if (digitalRead(CollectPin)==1) { //For collection if weights still have to be in robot
  //   programState=4;
  //   
  // }
  

  if (programState == 1) {
    //Import
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

    //Replace these with Import perhaps

    if (InductionDetected == 1) {
      half_forward(timedelay);
      //Import
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
  }
  Serial.println(); //Next line
}

















 //Seconds since program has been running
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




        //If robot is against a wall where does it go? The following is future code for later if two ultrasonic sensors are used
        //int directionCheckarray[20] = {0}; //An array of distances as the robot rotates when it detects a wall
        //int8_t it_num = 0;
        //int32_t MaxCM = 0;
        //int8_t MaxCMElement = 0;
        //if (TopLeft>TopRight) {
          //full_turn_left(timedelay);
          //Get IMU heading
          //if IMU heading is less than 360 degrees
            //A_read();
            //B_read();
            //directionCheckarray[it_num] = [(TopLeft+TopRight)/2];
            //if ((TopLeft+TopRight)/2 > (MaxCMElement)){
              //MaxCM = (TopLeft+TopRight)/2;
              //MaxCMElement = TopLeft;
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

  //PrevPositionX = CurrentposX;
  //PrevOrienZ = CurrentOrienZ;
//}




//sensors_event_t orientationData;
//bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//or_x = orientationData->orientation.x
//or_y = orientationData->orientation.y
//or_z = orientationData->orientation.z
