const int sensorPin = A12;

void setup() {
  serial.begin(9600);
  pinMode(sensorPin, INPUT);
}

void loop() {
  int sensorValue = digitalReal(sensorPin);

  Serial.Print("Metal detected: ");
  Serial.println(sensorValue);

  delay(500);
}