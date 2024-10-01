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

extern int FrontElectromagnetPin;
extern int MiddleElectromagnetPin;
extern int BackElectromagnetPin;
extern int electromagnets_activated;

//STEPPER MOTOR SETUP:
//001000
//000000101

extern int num_steps; // To be optimised
extern int MAdirpin;
extern int MAsteppin;
extern int MBdirpin;
extern int MBsteppin;

// Function declarations
extern void setupActuators();
extern void full_reverse(int timedelay);
extern void reverse_left(int timedelay);
extern void reverse_riht(int timedelay);
extern void stop(int timedelay);
extern void full_forward(int timedelay);
extern void half_forward(int timedelay);
extern void full_turn_right(int timedelay);
extern void forward_right(int timedelay);
extern void full_turn_left(int timedelay);
extern void forward_left(int timedelay);
extern void go_down(void);
extern void little_step_down(void);
extern void big_step_down(void);
extern void go_up(void);
extern void turn_on_electromagnet(int electromagnet);
extern void turn_off_electromagnet(int index);
#endif //ACTUATORS_H
