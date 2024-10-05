#include "Actuators.h"

// Motor setup
Servo myservoA, myservoB;  // create servo object to control a servo, A IS LEFT
int full_forward_speed = 1100;
int half_forward_speed = 1250;
int quarter_forward_speed = 1380;
int stop_speed = 1500;     // 1500 = stop, 1950 = full speed forward, 1100 = full back
int quarter_reverse_speed = 1670;
int half_reverse_speed = 1750;
int full_reverse_speed = 1950;


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

unsigned long previousMillis = 0; // Stores the last time an action was performed
unsigned long interval;       // Time to wait between steps


void nonBlockingMotorAction(unsigned long interval, int speedA, int speedB) {
  unsigned long currentMillis = millis(); // Get the current time

  if (currentMillis - previousMillis >= interval) { // Check if the interval has passed
    previousMillis = currentMillis; // Update the last action time
    // Perform action based on current motor state
    myservoA.writeMicroseconds(speedA);
    myservoB.writeMicroseconds(speedB);
  }
}

void full_reverse(int timedelay) {
  nonBlockingMotorAction(timedelay, full_reverse_speed, full_reverse_speed);
}

void reverse_left(int timedelay) {
  nonBlockingMotorAction(timedelay, half_reverse_speed, full_reverse_speed);
}

void reverse_right(int timedelay) {
  nonBlockingMotorAction(timedelay, full_reverse_speed, half_reverse_speed);
}

void stop(int timedelay) {
  nonBlockingMotorAction(timedelay, stop_speed, stop_speed);
}

void stop_blocking(int timedelay) {
  myservoA.writeMicroseconds(stop_speed);
  myservoB.writeMicroseconds(stop_speed);
  delay(timedelay);
}

void full_forward(int timedelay) {
  nonBlockingMotorAction(timedelay, full_forward_speed, full_forward_speed);
}

void half_forward(int timedelay) {
  nonBlockingMotorAction(timedelay, half_forward_speed, half_forward_speed);
}

void full_turn_right(int timedelay) {
  nonBlockingMotorAction(timedelay, full_reverse_speed, full_forward_speed);
}

void forward_right(int timedelay) {
  nonBlockingMotorAction(timedelay, half_forward_speed, full_forward_speed);
}

void forward_right_right(int timedelay) {
  nonBlockingMotorAction(timedelay, quarter_forward_speed, full_forward_speed);
}

void full_turn_left(int timedelay) {
  nonBlockingMotorAction(timedelay, full_forward_speed, full_reverse_speed);
}

void forward_left(int timedelay) {
  nonBlockingMotorAction(timedelay, full_forward_speed, half_forward_speed);
}

void forward_left_left(int timedelay) {
  nonBlockingMotorAction(timedelay, full_forward_speed, quarter_forward_speed);
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
