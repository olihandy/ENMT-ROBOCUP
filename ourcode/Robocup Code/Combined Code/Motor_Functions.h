#include <stdio.h>
#include <Servo.h>
#include <Wire.h>

void setup();

void full_reverse(int timedelay);

void stop(int timedelay);

void full_forward(int timedelay);

void half_forward(int timedelay);

void full_turn_right(int timedelay);

void forward_right(int timedelay);

void directioncheck_right(int timedelay);

void full_turn_left(int timedelay);

void forward_left(int timedelay);

void Turning(int TopLeft, int TopMiddle, int TopRight);

#endif