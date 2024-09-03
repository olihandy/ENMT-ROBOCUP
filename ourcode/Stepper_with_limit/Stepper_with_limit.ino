#include <Wire.h> // Include the I2C library (required)
#include <SparkFunSX1509.h> // Include SX1509 library

// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout

//STEPPER MOTOR SETUP:
//001000
//000000101

int MAdirpin = 30;
int MAsteppin = 31;
int MBdirpin = 32;
int MBsteppin = 33;
const byte SX1509_AIO6 = 6; 
int MElectro1 = 24;
int MElectro2 = 25;
int MElectro3 = 26;

void setup()
{   
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

  pinMode(MElectro1,OUTPUT);
  pinMode(MElectro2,OUTPUT);
  pinMode(MElectro3,OUTPUT);

  io.pinMode(SX1509_AIO6, INPUT);
  
  digitalWrite(MElectro1,HIGH);
  digitalWrite(MElectro2,HIGH);
  digitalWrite(MElectro3,HIGH);

}

void go_down(void) {
  
  digitalWrite(MAdirpin,LOW);
  digitalWrite(MBdirpin,LOW);
  
  for(int j=0;j<=1000;j++)            //Move 1000 steps down
  {
    delayMicroseconds(20);
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(100);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(100);
  }
}

void go_up(void) {
  digitalWrite(MAdirpin,HIGH);
  digitalWrite(MBdirpin,HIGH);

    for(int j=0;j<=1000;j++)            //Move 1000 steps up
  {
    delayMicroseconds(20);
    digitalWrite(MAsteppin,LOW);
    digitalWrite(MBsteppin,LOW);
    delayMicroseconds(100);
    digitalWrite(MAsteppin,HIGH);
    digitalWrite(MBsteppin,HIGH);
    delayMicroseconds(100);
  }
}

void loop()
{
  
  bool bAIO6=io.digitalRead(SX1509_AIO6);

  if(bAIO6) {
    go_down();
  } else {
    go_up();
  }




}
