#include <Servo.h>
#include <Wire.h> // Include the I2C library (required)
#include <SparkFunSX1509.h> // Include SX1509 library

Servo myservoA,myservoB;     // create servo object to control a servo
int stop_speed = 1500;        // Variable to change direction of movement, 1500 = stop, 1900 = full speed foward, 1100 = full back
int full_forward_speed = 1900;
int full_reverse_speed = 1100;

// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout


int MAdirpin = 30;
int MAsteppin = 31;
int MBdirpin = 32;
int MBsteppin = 33;
int MElectro = 26;
int MElectro1 = 24;

uint32_t num_steps = 5000;
const byte SX1509_AIO6 = 6; 
const byte SX1509_AIO5 = 5; 
const byte SX1509_AIO4 = 4; 

void setup()
{   

  myservoA.attach(0);  // attaches the servo  to the servo object useing pin 0
  myservoB.attach(1);  // attaches the servo  to the servo object useing pin 1  
  // Serial is used in this example to display the input value
  // of the SX1509_INPUT_PIN input:
  Serial.begin(9600);
  // Call io.begin(<address>) to initialize the SX1509. If it
  // successfully communicates, it'll return 1.
  if (!io.begin(SX1509_ADDRESS))
  {
    Serial.println("Failed to communicate.");
    while (1) ;
  }
  pinMode(MAdirpin,OUTPUT);
  pinMode(MAsteppin,OUTPUT);
  pinMode(MBdirpin,OUTPUT);
  pinMode(MBsteppin,OUTPUT);
  pinMode(MElectro,OUTPUT);
  pinMode(MElectro1,OUTPUT);

  io.pinMode(SX1509_AIO6, INPUT);
  io.pinMode(SX1509_AIO5, INPUT);
  io.pinMode(SX1509_AIO4, INPUT);

  delay(2000);
}

void go_down(uint32_t steps)
{
  digitalWrite(MAdirpin,HIGH);
  digitalWrite(MBdirpin,HIGH);
  
  for(uint32_t j = 0;j < steps; j++)
  {
    delayMicroseconds(20);
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(20);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(20);
  }
}

void go_up(uint32_t steps)
{
  digitalWrite(MAdirpin,LOW);
  digitalWrite(MBdirpin,LOW);
  
  for(uint32_t j = 0;j < steps; j++)
  {
    delayMicroseconds(20);
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);

    delayMicroseconds(20);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(20);
  }
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

void loop()
{
  bool bAIO4,bAIO5,bAIO6;

  char  sOut[100];
  
  bAIO4=io.digitalRead(SX1509_AIO4);
  bAIO5=io.digitalRead(SX1509_AIO5);
  bAIO6=io.digitalRead(SX1509_AIO6);
  
  full_forward(100);
  go_down(20000);
  full_reverse(100);
  go_up(20000);

  if(!bAIO5) {
    digitalWrite(MElectro1,LOW);
    digitalWrite(MElectro,LOW);
  } else {
    digitalWrite(MElectro1,HIGH);
    digitalWrite(MElectro,HIGH);
  }
  
  sprintf(sOut,"UP: %d ElectroMagnet: %d Down: %d",!bAIO4,!bAIO5,!bAIO6);
  
  Serial.println(sOut);




}