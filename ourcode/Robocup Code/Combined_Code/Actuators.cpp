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
int BackElectromagnetPin = 21;

// Stepper motor setup
int num_steps = 67000;      // Fully down and up
int big_step = 62000;
int little_step = 5000;
int teeny_step = 1000;

int stepper_motor_fast = 20;
int stepper_motor_slow = 40;
int motortime = 10;

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

  // digitalWrite(FrontElectromagnetPin, HIGH);
  // digitalWrite(MiddleElectromagnetPin, HIGH);
  // digitalWrite(BackElectromagnetPin, HIGH);
 
}

//--------------------------------------------------------------------------------------------------------//
//--------------------------------------------- Motors ---------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

unsigned long actionStartTime = 0;
unsigned long actionDuration = 0;
bool actionInProgress = false;
int currentSpeedA = 1500;  // Default neutral position for servos
int currentSpeedB = 1500;

void nonBlockingMotorAction(unsigned long interval, int speedA, int speedB) {
  unsigned long currentMillis = millis(); // Get the current time

  // Start a new action only if no action is currently in progress
  if (!actionInProgress) {
    actionStartTime = currentMillis;    // Record the start time
    actionDuration = interval;          // Set the duration of the action
    currentSpeedA = speedA;             // Set speed for motor A
    currentSpeedB = speedB;             // Set speed for motor B
    actionInProgress = true;            // Mark action as in progress

    myservoA.writeMicroseconds(currentSpeedA); // Start motor A
    myservoB.writeMicroseconds(currentSpeedB); // Start motor B
  }

  // If an action is in progress, check if the interval has passed
  if (actionInProgress && (currentMillis - actionStartTime >= actionDuration)) {
    // Stop the motors after the interval is complete
    actionInProgress = false;          // Mark action as finished
  }
}

unsigned long action1StartTime = 0;
unsigned long action2StartTime = 0;
unsigned long action1Duration = 0;
unsigned long action2Duration = 0;
bool action1InProgress = false;
bool action2InProgress = false;

void sequentialNonBlockingMotorAction(unsigned long interval1, int speedA1, int speedB1, unsigned long interval2, int speedA2, int speedB2) {
  unsigned long currentMillis = millis();

  // Start the first action if it hasn't started
  if (!action1InProgress && !action2InProgress) {
    action1StartTime = currentMillis;
    action1Duration = interval1;
    myservoA.writeMicroseconds(speedA1);
    myservoB.writeMicroseconds(speedB1);
    action1InProgress = true;
  }

  // Check if the first action is completed
  if (action1InProgress && (currentMillis - action1StartTime >= action1Duration)) {
    // Start the second action
    action2StartTime = currentMillis;
    action2Duration = interval2;
    myservoA.writeMicroseconds(speedA2);
    myservoB.writeMicroseconds(speedB2);
    action1InProgress = false; // First action finished
    action2InProgress = true;  // Second action starts
  }

  // Check if the second action is completed
  if (action2InProgress && (currentMillis - action2StartTime >= action2Duration)) {
    action2InProgress = false; // Second action finished
  }
}

void reverseThenTurnLeft(unsigned long interval1, unsigned long interval2) {
  sequentialNonBlockingMotorAction(interval1, full_reverse_speed, full_reverse_speed, 
                                   interval1, full_forward_speed, full_reverse_speed);
}

void reverseThenTurnRight(unsigned long interval1, unsigned long interval2) {
  sequentialNonBlockingMotorAction(interval1, full_reverse_speed, full_reverse_speed, 
                                   interval1, full_reverse_speed, full_forward_speed);  
}

void full_reverse(int timedelay) {
  nonBlockingMotorAction(timedelay, full_reverse_speed, full_reverse_speed);
}

void full_reverse_blocking(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(timedelay);  
}

void reverse_left(int timedelay) {    //Brings back more left
  nonBlockingMotorAction(timedelay, half_reverse_speed, full_reverse_speed);
}

void reverse_left_blocking(int timedelay) {
  myservoA.writeMicroseconds(half_reverse_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(timedelay);   
}
void reverse_right(int timedelay) {
  nonBlockingMotorAction(timedelay, full_reverse_speed, half_reverse_speed);
}

void reverse_right_blocking(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(half_reverse_speed);
  delay(timedelay);  
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

void full_forward_blocking(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay);
}

void half_forward(int timedelay) {
  nonBlockingMotorAction(timedelay, half_forward_speed, half_forward_speed);
}

void half_forward_blocking(int timedelay) {
  myservoA.writeMicroseconds(half_forward_speed);
  myservoB.writeMicroseconds(half_forward_speed);
  delay(timedelay); 
}

void full_turn_right(int timedelay) {
  nonBlockingMotorAction(timedelay, full_reverse_speed, full_forward_speed);
}

void full_turn_right_blocking(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay);   
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

void full_turn_left_blocking(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(timedelay);  
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
    case 1:
      digitalWrite(BackElectromagnetPin, HIGH);  // Activate first electromagnet
      break;
    case 2:
      digitalWrite(MiddleElectromagnetPin, HIGH);  // Activate second electromagnet
      break;
    case 3:
      digitalWrite(FrontElectromagnetPin, HIGH);  // Activate third electromagnet
      break;
    default:
      Serial.println("Invalid electromagnet index!");
      break;
  }
}

void turn_off_electromagnet(int electromagnet) {
  switch (electromagnet) {
    case 1:
      digitalWrite(BackElectromagnetPin, LOW);  // Deactivate first electromagnet
      break;
    case 2:
      digitalWrite(MiddleElectromagnetPin, LOW);  // Deactivate second electromagnet
      break;
    case 3:
      digitalWrite(FrontElectromagnetPin, LOW);  // Deactivate third electromagnet
      break;
    default:
      Serial.println("Invalid electromagnet index!");
      break;
  }
}
