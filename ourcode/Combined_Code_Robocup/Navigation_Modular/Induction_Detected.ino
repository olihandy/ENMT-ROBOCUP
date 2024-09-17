#include <Wire.h>
#include <stdio.h> 
#include <Induction_Detected.h>

//Stepper Motor Setup
int MAdirpin = 32;
int MAsteppin = 33;
int MBdirpin = 30;
int MBsteppin = 31;
int StepperPosition = 14000;
int StepperState = 0; //0 for moving down, 1 for stopping to make electromsagnets work, 2 for back up then resets to 0 afterwards

const int ElectroMagnet1Pin = 25; //Electromagnet pins, changed from A as the front and rear never turn on
const int ElectroMagnet2Pin = 24;
const int ElectroMagnet3Pin = 14;
bool ElectroMagnet1On = false;
bool ElectroMagnet2On = false;
bool ElectroMagnet3On = false;


void setup() {
  //ElectroMagnet
  pinMode(ElectroMagnet1Pin, OUTPUT);
  pinMode(ElectroMagnet2Pin, OUTPUT);
  pinMode(ElectroMagnet3Pin, OUTPUT);


  //Stepper Motors
  pinMode(MAdirpin,OUTPUT);
  pinMode(MAsteppin,OUTPUT);
  pinMode(MBdirpin,OUTPUT);
  pinMode(MBsteppin,OUTPUT);
}

void Induction_Detected () {
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
}
