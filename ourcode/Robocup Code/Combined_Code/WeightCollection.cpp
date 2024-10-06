#include "WeightCollection.h"

extern int stepper_motor_fast;
extern int stepper_motor_slow;
extern int motortime;

volatile bool finished_collecting = false;


void CollectWeight_1(void) {
  finished_collecting = false;
  stop_blocking(motortime);
  go_down(stepper_motor_fast);
  turn_on_electromagnet(1);
  stop_blocking(motortime);
  go_up(stepper_motor_fast);
  finished_collecting = true;
}

void CollectWeight_2(void) {
  finished_collecting = false;
  stop_blocking(motortime);
  big_step_down(stepper_motor_fast);
  half_forward_blocking(10 * motortime);
  stop_blocking(motortime);
  little_step_down(stepper_motor_fast);
  turn_on_electromagnet(2);
  stop_blocking(motortime);
  go_up(stepper_motor_fast);
  finished_collecting = true;

}


void CollectWeight_3(void) {
  finished_collecting = false;
  stop_blocking(motortime);
  big_step_down(stepper_motor_fast);
  half_forward_blocking(5 * motortime);
  stop_blocking(motortime);
  little_step_down(stepper_motor_fast);
  turn_on_electromagnet(3);
  stop_blocking(motortime);
  go_up(stepper_motor_fast);
  finished_collecting = true;
}