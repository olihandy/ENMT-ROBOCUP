#include "DropWeights.h"

extern int motortime;
extern int stepper_motor_fast;
extern int stepper_motor_slow;

void drop_weights(void) {
    stop(motortime);
    go_down(stepper_motor_slow);
    turn_off_electromagnet(0);
    turn_off_electromagnet(1);
    turn_off_electromagnet(2);
    stop(motortime);
    go_up(stepper_motor_fast);  
}