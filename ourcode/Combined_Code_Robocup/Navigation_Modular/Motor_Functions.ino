#include <stdio.h>
#include <Servo.h>
#include <Wire.h> 
#include <Motor_Functions.h>

//Motor setup
Servo myservoA, myservoB;  // create servo object to control a servo
int stop_speed = 1500;     // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
int full_reverse_speed = 1900;
int full_forward_speed = 1100;
int half_reverse_speed = 1750;
int half_forward_speed = 1250;



void setup() {
  //Motors
  myservoA.attach(7);  // attaches the servo  to the servo object using pin 0
  myservoB.attach(8);  // attaches the servo  to the servo object using pin 1

  Serial.println("Configured DC Motors");
}

void full_reverse(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_reverse_speed);

  delay(timedelay);
}

void stop(int timedelay) {
  myservoA.writeMicroseconds(stop_speed);
  myservoB.writeMicroseconds(stop_speed);

  delay(timedelay);
}

void full_forward(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_forward_speed);

  delay(timedelay);
}

void half_forward(int timedelay) {
  myservoA.writeMicroseconds(half_forward_speed);
  myservoB.writeMicroseconds(half_forward_speed);

  delay(timedelay);
}

void full_turn_right(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay);
}

void forward_right(int timedelay) {
  myservoA.writeMicroseconds(half_forward_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay);
}

void directioncheck_right(int timedelay) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(timedelay * 3);  //Will be replaced by IMU code
}

void full_turn_left(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(timedelay);
}


void forward_left(int timedelay) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(half_forward_speed);
}

void Turning(int TopLeft, int TopMiddle, int TopRight) {
  digitalWrite(LED1,LOW);
  if (TopLeft < 20 && TopMiddle > 20 && TopRight > 20) { //&& PrevturnTime-millis()>2000
    full_turn_right(timedelay);
    digitalWrite(LED3,LOW);
    digitalWrite(LED2,HIGH); //Yellow
    Serial.print(" Right");
  } else if (TopLeft > 20 && TopMiddle > 20 && TopRight < 20) { //&& PrevturnTime-millis()>2000
    full_turn_left(timedelay);
    digitalWrite(LED3,LOW);
    digitalWrite(LED2,HIGH); //YELLOW
    Serial.print(" Left");
  } else if (TopLeft > 20 && TopMiddle > 20 && TopRight > 20) { //open space
    digitalWrite(LED3,LOW); //off
    digitalWrite(LED2,LOW); //off
    programState = 0;
  } else if (TopLeft > 20 && TopMiddle < 20 && TopRight > 20) { //A thin slab of wall in front of the robot
    digitalWrite(LED3,LOW); //off
    digitalWrite(LED2,HIGH); //off
    if (TopLeft<TopRight) { 
      full_turn_right(timedelay);
      Serial.print("Slab Turn Right");
    } else {
      full_turn_left(timedelay);
      Serial.print("Slab Turn Left");
    }
  } else if (TopLeft < 20 && TopMiddle < 20 && TopRight < 20) { //Solid Wall ahead, IMU will do this
    if (TopLeft<TopRight) {
      full_turn_right(timedelay);
      Serial.print("Solid Wall Turn Right");
    } else {
      full_turn_left(timedelay);
      Serial.print("Solid Wall Turn Left");
    }
    Serial.print("180 degree turn");
  } else if (TopLeft < 20 && TopMiddle > 20 && TopRight < 20) {
    full_forward(timedelay);
    Serial.print("Space in Between");
  } else { //A large wall in front of just more than half of the robot
    digitalWrite(LED2,LOW);
    digitalWrite(LED3,HIGH); //Red stopped
    if (TopLeft<TopRight) {
      full_turn_right(timedelay);
      Serial.print("Wall Turn Right");
    } else {
      full_turn_left(timedelay);
      Serial.print("Wall Turn Left");
    }
  }
}
