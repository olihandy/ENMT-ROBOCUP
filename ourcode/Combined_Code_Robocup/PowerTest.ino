#include <Servo.h>

Servo myservoA,myservoB;     // create servo object to control a servo
const int InductiveProximitySensorPin = A12;
int full_forward_speed = 1900;

const int ElectroMagnetPin = A13;

static long durationA,durationB, Acm,Bcm;

const int AtrigPin = 3;
const int AechoPin = 2;

const int BtrigPin = 5;
const int BechoPin = 4;

int MAdirpin = 7;
int MAsteppin = 8;
int MBdirpin = 30;
int MBsteppin = 31;



void setup()
{   
  myservoA.attach(0);  // attaches the servo  to the servo object useing pin 0
  myservoB.attach(1);  // attaches the servo  to the servo object useing pin 1
  Serial.begin(9600);
  pinMode(InductiveProximitySensorPin, INPUT);
  pinMode(ElectroMagnetPin, OUTPUT);

  pinMode(AtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(AechoPin, INPUT);

  pinMode(BtrigPin, OUTPUT);            //Setup ultrasound pins
  pinMode(BechoPin, INPUT);

  digitalWrite(AtrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);

  pinMode(MAdirpin,OUTPUT);
  pinMode(MAsteppin,OUTPUT);
  pinMode(MBdirpin,OUTPUT);
  pinMode(MBsteppin,OUTPUT);
  

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
  myservoA.writeMicroseconds(full_forward_speed);
  myservoB.writeMicroseconds(full_forward_speed);
  
  int sensorValue = digitalRead(InductiveProximitySensorPin);

  A_read();
  B_read();

  Acm = microsecondsToCentimeters(durationA);
  Bcm = microsecondsToCentimeters(durationB);

  digitalWrite(ElectroMagnetPin,HIGH);

  digitalWrite(MAdirpin,LOW);
  digitalWrite(MBdirpin,LOW);
  
  int j = 0;
  for(j=0;j<=100000;j++)            //Move 1000 steps
  {

    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(50);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(50);
  }
  digitalWrite(MAdirpin,HIGH);
  digitalWrite(MBdirpin,HIGH);
}
