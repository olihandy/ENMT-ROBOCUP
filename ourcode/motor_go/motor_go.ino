#include <Servo.h>

Servo myservoA,myservoB;     // create servo object to control a servo

void setup()
{   
  myservoA.attach(0);  // attaches the servo  to the servo object useing pin 0
  myservoB.attach(1);  // attaches the servo  to the servo object useing pin 1

}

void loop() 
{ 
  myservoA.writeMicroseconds(1100);      // sets the servo position full speed backward
  myservoB.writeMicroseconds(1100);      // sets the servo position full speed backward

  delay(1500);                           // waits for the servo to get there 
  
  // myservoA.writeMicroseconds(1500);      // sets the servo stop
  // myservoB.writeMicroseconds(1500);      // sets the servo stop
  // delay(1500);                           // waits for the servo to get there 


  //myservoA.writeMicroseconds(1900);      // sets the servo position full speed forward
  //myservoB.writeMicroseconds(1900);      // sets the servo position full speed forward

  delay(1500);                           // waits for the servo to get there 

  // myservoA.writeMicroseconds(1500);      // sets the servo stop
  // myservoB.writeMicroseconds(1500);      // sets the servo stop
  // delay(1500);                           // waits for the servo to get there 
} 
