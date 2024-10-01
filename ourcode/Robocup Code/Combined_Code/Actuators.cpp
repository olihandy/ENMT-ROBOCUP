#include "Actuators.h"

//Motor setup
Servo myservoA, myservoB;  // create servo object to control a servo, A IS LEFT
int stop_speed = 1500;     // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
int full_reverse_speed = 1950;
int full_forward_speed = 1100;
int half_reverse_speed = 1750;
int half_forward_speed = 1250;
int timedelay = 100;

int FrontElectromagnetPin = 20;
int MiddleElectromagnetPin = 24;
int BackElectromagnetPin = 14;

//STEPPER MOTOR SETUP:
//001000
//000000101

int num_steps = 67000; // Fully down and up

int big_step = 62000;
int little_step = 5000;
int teeny_step = 1000;

int MAdirpin = 32;
int MAsteppin = 33;
int MBdirpin = 30;
int MBsteppin = 31;

void setupActuators() {
  //Motors
  myservoA.attach(0);  // attaches the servo  to the servo object using pin 0
  myservoB.attach(1);  // attaches the servo  to the servo object using pin 1
  Serial.println("Configured DC Motors");

  //Stepper Motors
  pinMode(MAdirpin,OUTPUT);
  pinMode(MAsteppin,OUTPUT);
  pinMode(MBdirpin,OUTPUT);
  pinMode(MBsteppin,OUTPUT);

  pinMode(FrontElectromagnetPin, OUTPUT);
  pinMode(MiddleElectromagnetPin, OUTPUT);
  pinMode(BackElectromagnetPin, OUTPUT);  


}

void full_reverse(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_reverse_speed);

  delay(timedelay);
}

void reverse_left(int timedelay) {
  myservoA.writeMicroseconds(half_reverse_speed);
  myservoB.writeMicroseconds(full_reverse_speed);

  delay(timedelay);
}

void reverse_right(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(half_reverse_speed);

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


void full_turn_left(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(timedelay);
}

void forward_left(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(half_forward_speed);
}


void go_down(void) {
  
  digitalWrite(MAdirpin,LOW);
  digitalWrite(MBdirpin,LOW);
  
  for(int j=0;j<=num_steps;j++)            //Move 1000 steps down
  {
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(20);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(20);
  }
}

void little_step_down(void) {
  
  digitalWrite(MAdirpin,LOW);
  digitalWrite(MBdirpin,LOW);
  
  for(int j=0;j<=little_step;j++)            //Move 1000 steps down
  {
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(20);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(20);
  }
}

void big_step_down(void) {
  
  digitalWrite(MAdirpin,LOW);
  digitalWrite(MBdirpin,LOW);
  
  for(int j=0;j<=big_step;j++)            //Move 1000 steps down
  {
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(20);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(20);
  }
}

void go_up(void) {
  digitalWrite(MAdirpin,HIGH);
  digitalWrite(MBdirpin,HIGH);

  for(int j=0;j<=num_steps;j++)            //Move 1000 steps up
  {
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(20);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(20);
  }
}


void turn_on_electromagnet(int index) {
    switch (index) {
        case 0:
            digitalWrite(BackElectromagnetPin, HIGH); // Activate first electromagnet
            break;
        case 1:
            digitalWrite(MiddleElectromagnetPin, HIGH); // Activate second electromagnet
            break;
        case 2:
            digitalWrite(FrontElectromagnetPin, HIGH); // Activate third electromagnet
            break;
        default:
            Serial.println("Invalid electromagnet index!");
            break;
    }
}

void turn_off_electromagnet(int index) {
    switch (index) {
        case 0:
            digitalWrite(BackElectromagnetPin, LOW); // Activate first electromagnet
            break;
        case 1:
            digitalWrite(MiddleElectromagnetPin, LOW); // Activate second electromagnet
            break;
        case 2:
            digitalWrite(FrontElectromagnetPin, LOW); // Activate third electromagnet
            break;
        default:
            Serial.println("Invalid electromagnet index!");
            break;
    }
}

