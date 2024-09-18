// Motor_Functions.h

#ifndef MOTOR_FUNCTIONS_H
#define MOTOR_FUNCTIONS_H

#include <Servo.h>

// Servo motor variables
extern Servo myservoA, myservoB;  // Servo objects to control motors
extern int stop_speed;
extern int full_reverse_speed;
extern int full_forward_speed;
extern int half_reverse_speed;
extern int half_forward_speed;

// Stepper motor variables
extern int num_steps;
extern int MAdirpin;
extern int MAsteppin;
extern int MBdirpin;
extern int MBsteppin;

// Function declarations
void setup_motors();
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

#endif //MOTOR_FUNCTIONS_H
