// Actuators.h

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdio.h>
#include <Servo.h>
#include <Wire.h>

extern int stepper_motor_fast;
extern int stepper_motor_slow;
extern int motortime;
extern bool actionInProgress;


// Electromagnet setup
extern int FrontElectromagnetPin;
extern int MiddleElectromagnetPin;
extern int BackElectromagnetPin;
extern int electromagnets_activated;

// Function declarations
void setupActuators(void);
void nonBlockingMotorAction(unsigned long interval, int speedA, int speedB);
void full_reverse(int timedelay);
void full_reverse_blocking(int timedelay);
void reverse_left(int timedelay);
void reverse_right(int timedelay);
void stop(int timedelay);
void stop_blocking(int timedelay);
void full_forward(int timedelay);
void full_forward_blocking(int timedelay);

void half_forward_blocking(int timedelay);
void half_forward(int timedelay);
void full_turn_right(int timedelay);
void forward_right(int timedelay);

void reverse_right_blocking(int timedelay);
void reverse_left_blocking(int timedelay);

void full_turn_right_blocking(int timedelay);
void forward_right_right(int timedelay);

void full_turn_left(int timedelay);
void full_turn_left_blocking(int timedelay);

void forward_left(int timedelay);
void forward_left_left(int timedelay);

void sequentialNonBlockingMotorAction(unsigned long interval1, int speedA1, int speedB1, unsigned long interval2, int speedA2, int speedB2);
void reverseThenTurnLeft(unsigned long interval1, unsigned long interval2);
void reverseThenTurnRight(unsigned long interval1, unsigned long interval2);



// Stepper motor functions now include speed parameter
void go_down(int speed);            // Descend using speed instead of delay
void little_step_down(int speed);   // Small downward steps with adjustable speed
void big_step_down(int speed);      // Larger downward steps with adjustable speed
void go_up(int speed);              // Ascend using speed parameter

// Electromagnet control functions
void turn_on_electromagnet(int electromagnet);   // Turn on specified electromagnet
void turn_off_electromagnet(int electromagnet);          // Turn off specified electromagnet

#endif //ACTUATORS_H
