#include <Wire.h> // Include the I2C library (required)
#include <SparkFunSX1509.h> // Include SX1509 library

// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
const byte SX1509_ADDRESS = 0x3E;  // SX1509 I2C address
SX1509 io; // Create an SX1509 object to be used throughout


int MAdirpin = 30;
int MAsteppin = 31;
int MBdirpin = 32;
int MBsteppin = 33;

uint32_t num_steps = 5000;
const byte SX1509_AIO6 = 6; 


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
  io.pinMode(SX1509_AIO6, INPUT);

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
  Serial.println("Ended move");

}
void loop()
{
  bool bAIO6;
  char  sOut[100];
  bAIO6=io.digitalRead(SX1509_AIO6);
  
  if(!bAIO6) {
      go_down(10000);
  }
  //Set direction for all channels



}