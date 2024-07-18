#include <Servo.h>

Servo myservoA,myservoB;     // create servo object to control a servo
int stop_speed = 1500;        // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
int full_forward_speed = 1900;
int full_reverse_speed = 1100;

const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;


int time = 10; //time in milliseconds
static long durationA,durationB, Acm,Bcm;


 
void setup()
{   
  myservoA.attach(0);  // attaches the servo  to the servo object useing pin 0
  myservoB.attach(1);  // attaches the servo  to the servo object useing pin 1

  pinMode(AtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(AechoPin, INPUT);

  pinMode(BtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(BechoPin, INPUT);
  
  Serial.begin(9600);                  // initialize serial communication:
  
  digitalWrite(AtrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);

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

void directioncheck_left(int time) {
  myservoA.writeMicroseconds(full_reverse_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  delay(time*3); //Will be replaced by IMU code

}

void full_turn_right(int time) {
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_reverse_speed);
  delay(time);  
}

void A_read(void) {
  digitalWrite(AtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(AtrigPin, LOW);
  durationA = pulseIn(AechoPin, HIGH);

}

void B_read(void) {
  digitalWrite(BtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(BtrigPin, LOW);
  durationB = pulseIn(BechoPin, HIGH);
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
} 

void loop() 
{ 

 
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:


 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.

  A_read();
  B_read();

  
 
  // convert the time into a distance
  Acm = microsecondsToCentimeters(durationA);
  Bcm = microsecondsToCentimeters(durationB);

  Serial.print(Acm);
  Serial.print(" ");
  Serial.print(Bcm);
  Serial.println();
  if(Acm > 3300 || Bcm > 3300 ) {
    full_forward(time);
  } else if(Acm < 20 && Bcm > 20) {
    full_turn_left(time);
  } else if(Acm > 20 && Bcm < 20) {
    full_turn_right(time);
  } else if (Acm > 20 && Bcm > 20) {
    full_forward(time);
  } else {
    stop();
    //If robot is against a wall where does it go? The following is future code for later
    //int directionCheckarray[20] = {0}; //An array of distances as the robot rotates when it detects a wall
    //int8_t it_num = 0;
    //int32_t MaxCM = 0;
    //int8_t MaxCMElement = 0;
    //if (Acm>Bcm) {
      //full_turn_left(time);
      //Get IMU heading
      //if IMU heading is less than 360 degrees
        //A_read();
        //B_read();
        //directionCheckarray[it_num] = [(Acm+Bcm)/2];
        //if ((Acm+Bcm)/2 > (MaxCMElement)){
          //MaxCM = (Acm+Bcm)/2;
          //MaxCMElement = Acm;
        //}
        //it_num++;
        //full_turn_left(time);
      //for (i=0; i<(it_num-MaxCMElement), i++){
        //full_turn_right(time);
      //}
    //} else {
      //full_turn_right(time);
    //}
  }
} 
