#include <Servo.h>

Servo myservoA,myservoB;     // create servo object to control a servo
int stop_speed = 1500;        // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
int full_forward_speed = 1900;
int full_reverse_speed = 1100;
void setup()
{   
  myservoA.attach(2);  // attaches the servo  to the servo object useing pin 0
  myservoB.attach(2);  // attaches the servo  to the servo object useing pin 1

}

void full_reverse(int time) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_reverse_speed);

  delay(time);
}

void stop(int time) {
  myservoA.writeMicroseconds(stop_speed);
  myservoB.writeMicroseconds(stop_speed);

  delay(time);
}

void full_forward(int time) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_forward_speed);

  delay(time); 
}

void full_turn_left(int time) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(time);
}

void full_turn_right(int time) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(time);  
}

void loop() 
{ 
int time = 10;
full_forward(time);
full_reverse(time);
full_turn_left(2* time);
full_turn_right(time);
} 
