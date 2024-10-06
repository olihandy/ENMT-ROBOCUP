#include "WeightCollection.h"

extern int stepper_motor_fast;
extern int stepper_motor_slow;
extern int motortime;
int NumWeightsCollected = 0;

void CollectWeight_1(void) {
  half_forward(motortime);
  if (!digitalRead(BackInductionPin)) {
    half_forward_blocking(3*motortime);
    stop_blocking(motortime);
    go_down(stepper_motor_fast);
    turn_on_electromagnet(1);
    stop_blocking(motortime);
    go_up(stepper_motor_fast);
    NumWeightsCollected++;
  }
}

void CollectWeight_2(void) {
  stop_blocking(motortime);
  big_step_down(stepper_motor_fast);
  half_forward_blocking(12 * motortime);
  stop_blocking(motortime);
  little_step_down(stepper_motor_fast);
  turn_on_electromagnet(2);
  stop_blocking(motortime);
  go_up(stepper_motor_slow);
  NumWeightsCollected++;
}


void CollectWeight_3(void) {
  stop_blocking(motortime);
  big_step_down(stepper_motor_slow);
  half_forward_blocking(7 * motortime);
  stop_blocking(motortime);
  little_step_down(stepper_motor_slow);
  turn_on_electromagnet(3);
  stop_blocking(motortime);
  go_up(stepper_motor_slow);
  NumWeightsCollected++;
}