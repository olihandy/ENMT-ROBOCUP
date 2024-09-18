/*
Test out the induction sensor
*/
//This means the pins, not entire ports which the cables connect to
const int AechoPin = 26;
 
void setup() 
{  
  pinMode(AechoPin, INPUT);
  Serial.begin(9600);                  // initialize serial communication:
}
 
void loop()
{
  int Acm = digitalRead(AechoPin);
  Serial.print(Acm);
  Serial.print("\n");
  delay(100);
}