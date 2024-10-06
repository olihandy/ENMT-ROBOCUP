#include "Actuators.h"

// Motor setup
Servo myservoA, myservoB;  // create servo object to control a servo, A IS LEFT
int stop_speed = 1500;     // 1500 = stop, 1950 = full speed forward, 1100 = full back
int full_reverse_speed = 1950;
int full_forward_speed = 1100;
int half_reverse_speed = 1750;
int half_forward_speed = 1250;
int timedelay = 100;
float kp = 0.5;  // Proportional constant for speed control (tune this)

int FrontElectromagnetPin = 14;
int MiddleElectromagnetPin = 20;
int BackElectromagnetPin = 24;

// Stepper motor setup
int num_steps = 67000;      // Fully down and up
int big_step = 62000;
int little_step = 5000;
int teeny_step = 1000;

int MAdirpin = 32;
int MAsteppin = 33;
int MBdirpin = 30;
int MBsteppin = 31;

void setupActuators() {
  // Motors
  myservoA.attach(0);  // attaches the servo to the servo object using pin 0
  myservoB.attach(1);  // attaches the servo to the servo object using pin 1
  Serial.println("Configured DC Motors");

  // Stepper Motors
  pinMode(MAdirpin, OUTPUT);
  pinMode(MAsteppin, OUTPUT);
  pinMode(MBdirpin, OUTPUT);
  pinMode(MBsteppin, OUTPUT);

  pinMode(FrontElectromagnetPin, OUTPUT);
  pinMode(MiddleElectromagnetPin, OUTPUT);
  pinMode(BackElectromagnetPin, OUTPUT);
}

//--------------------------------------------------------------------------------------------------------//
//--------------------------------------------- Motors ---------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

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


void proportional_forward(int targetDistance, int sensorReading) {
  int error = targetDistance - sensorReading;
  int adjustedSpeed = constrain(full_forward_speed + kp * error, 1100, 1950);  // Adjust speed based on error, within limits

  myservoA.writeMicroseconds(adjustedSpeed);
  myservoB.writeMicroseconds(adjustedSpeed);
}

void proportional_backward(int targetDistance, int sensorReading) {
  int error = targetDistance - sensorReading;
  int adjustedSpeed = constrain(full_reverse_speed - kp * error, 1100, 1950);  // Adjust speed for reverse based on error

  myservoA.writeMicroseconds(adjustedSpeed);
  myservoB.writeMicroseconds(adjustedSpeed);
}

void proportional_forward_left(int targetDistance, int sensorReading) {
  int error = targetDistance - sensorReading;
  int adjustedSpeedA = constrain(half_forward_speed + kp * error, 1100, 1950);  // Adjust speed for left motor
  int adjustedSpeedB = constrain(full_forward_speed + kp * error, 1100, 1950);  // Adjust speed for right motor

  myservoA.writeMicroseconds(adjustedSpeedA);
  myservoB.writeMicroseconds(adjustedSpeedB);
}

void proportional_forward_right(int targetDistance, int sensorReading) {
  int error = targetDistance - sensorReading;
  int adjustedSpeedA = constrain(full_forward_speed + kp * error, 1100, 1950);  // Adjust speed for left motor
  int adjustedSpeedB = constrain(half_forward_speed + kp * error, 1100, 1950);  // Adjust speed for right motor

  myservoA.writeMicroseconds(adjustedSpeedA);
  myservoB.writeMicroseconds(adjustedSpeedB);
}


//--------------------------------------------------------------------------------------------------------//
//----------------------------------------- Stepper Motors ------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

void go_down(int speed) {
  digitalWrite(MAdirpin, LOW);
  digitalWrite(MBdirpin, LOW);
  
  for (int j = 0; j <= num_steps; j++) {
    digitalWrite(MAsteppin, LOW);
    digitalWrite(MBsteppin, LOW);
    delayMicroseconds(speed);
    digitalWrite(MAsteppin, HIGH);
    digitalWrite(MBsteppin, HIGH);
    delayMicroseconds(speed);
  }
}

void little_step_down(int speed) {
  digitalWrite(MAdirpin, LOW);
  digitalWrite(MBdirpin, LOW);
  
  for (int j = 0; j <= little_step; j++) {
    digitalWrite(MAsteppin, LOW);
    digitalWrite(MBsteppin, LOW);
    delayMicroseconds(speed);
    digitalWrite(MAsteppin, HIGH);
    digitalWrite(MBsteppin, HIGH);
    delayMicroseconds(speed);
  }
}

void big_step_down(int speed) {
  digitalWrite(MAdirpin, LOW);
  digitalWrite(MBdirpin, LOW);
  
  for (int j = 0; j <= big_step; j++) {
    digitalWrite(MAsteppin, LOW);
    digitalWrite(MBsteppin, LOW);
    delayMicroseconds(speed);
    digitalWrite(MAsteppin, HIGH);
    digitalWrite(MBsteppin, HIGH);
    delayMicroseconds(speed);
  }
}

void go_up(int speed) {
  digitalWrite(MAdirpin, HIGH);
  digitalWrite(MBdirpin, HIGH);

  for (int j = 0; j <= num_steps; j++) {
    digitalWrite(MAsteppin, LOW);
    digitalWrite(MBsteppin, LOW);
    delayMicroseconds(speed);
    digitalWrite(MAsteppin, HIGH);
    digitalWrite(MBsteppin, HIGH);
    delayMicroseconds(speed);
  }
}

//--------------------------------------------------------------------------------------------------------//
//------------------------------------- Electromagnets ---------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

void turn_on_electromagnet(int electromagnet) {
  switch (electromagnet) {
    case 0:
      digitalWrite(BackElectromagnetPin, HIGH);  // Activate first electromagnet
      break;
    case 1:
      digitalWrite(MiddleElectromagnetPin, HIGH);  // Activate second electromagnet
      break;
    case 2:
      digitalWrite(FrontElectromagnetPin, HIGH);  // Activate third electromagnet
      break;
    default:
      Serial.println("Invalid electromagnet index!");
      break;
  }
}

void turn_off_electromagnet(int electromagnet) {
  switch (electromagnet) {
    case 0:
      digitalWrite(BackElectromagnetPin, LOW);  // Deactivate first electromagnet
      break;
    case 1:
      digitalWrite(MiddleElectromagnetPin, LOW);  // Deactivate second electromagnet
      break;
    case 2:
      digitalWrite(FrontElectromagnetPin, LOW);  // Deactivate third electromagnet
      break;
    default:
      Serial.println("Invalid electromagnet index!");
      break;
  }
}
