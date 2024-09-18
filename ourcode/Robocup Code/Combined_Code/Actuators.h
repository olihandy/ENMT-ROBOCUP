// Actuators.h

#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdio.h>
#include <Servo.h>
#include <Wire.h> 

//Motor setup
extern Servo myservoA, myservoB;  // create servo object to control a servo
extern int stop_speed;     // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
extern int full_reverse_speed;
extern int full_forward_speed;
extern int half_reverse_speed;
extern int half_forward_speed;
extern int timedelay;

//STEPPER MOTOR SETUP:
//001000
//000000101

extern int num_steps; // To be optimised
extern int MAdirpin;
extern int MAsteppin;
extern int MBdirpin;
extern int MBsteppin;

// Function declarations
void setupActuators();
void full_reverse(int timedelay);
void stop(int timedelay);
void full_forward(int timedelay);
void half_forward(int timedelay);
void full_turn_right(int timedelay);
void forward_right(int timedelay);
void directioncheck_right(int timedelay);
void full_turn_left(int timedelay);
void forward_left(int timedelay);
void go_down(void);
void go_up(void);

#endif //ACTUATORS_H
