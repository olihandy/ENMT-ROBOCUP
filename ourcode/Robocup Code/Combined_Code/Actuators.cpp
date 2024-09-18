#include "Actuators.h"

//Motor setup
Servo myservoA, myservoB;  // create servo object to control a servo
int stop_speed = 1500;     // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
int full_reverse_speed = 1900;
int full_forward_speed = 1100;
int half_reverse_speed = 1750;
int half_forward_speed = 1250;
int timedelay = 100;

//STEPPER MOTOR SETUP:
//001000
//000000101

int num_steps = 30000; // To be optimised
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


void go_down(void) {
  
  digitalWrite(MAdirpin,LOW);
  digitalWrite(MBdirpin,LOW);
  
  for(int j=0;j<=num_steps;j++)            //Move 1000 steps down
  {
    delayMicroseconds(20);
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(100);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(100);
  }
}

void go_up(void) {
  digitalWrite(MAdirpin,HIGH);
  digitalWrite(MBdirpin,HIGH);

    for(int j=0;j<=num_steps;j++)            //Move 1000 steps up
  {
    delayMicroseconds(20);
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(100);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(100);
  }
}

