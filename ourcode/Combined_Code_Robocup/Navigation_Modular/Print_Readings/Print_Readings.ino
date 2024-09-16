#include <Wire.h>  
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


void setup() {
  // put your setup code here, to run once:

}

void PrintInformation (uint16_t TopLeft, uint16_t TopMiddle, uint16_t TopRight, uint16_t MiddleRight, uint16_t MiddleLeft, uint16_t BottomRight, uint16_t BottomLeft) {
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
  // put your main code here, to run repeatedly:

}
