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
#include <C:\Users\aos30\OneDrive - University of Canterbury\University\2024\ENMT ENME 301\Robocup\ENMT-ROBOCUP\ourcode\Robocup Code\Combined_Code\Sensors.h>
#include <C:\Users\aos30\OneDrive - University of Canterbury\University\2024\ENMT ENME 301\Robocup\ENMT-ROBOCUP\ourcode\Robocup Code\Combined_Code\Actuators.h>
#include <C:\Users\aos30\OneDrive - University of Canterbury\University\2024\ENMT ENME 301\Robocup\ENMT-ROBOCUP\ourcode\Robocup Code\Combined_Code\Induction_Detected.h>

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


void setup()  //Need one setup function
{

}

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

void Turning(int TopLeft, int TopMiddle, int TopRight) {
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

