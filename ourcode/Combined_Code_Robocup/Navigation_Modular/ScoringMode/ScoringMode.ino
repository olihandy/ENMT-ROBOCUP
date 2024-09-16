const int ElectroMagnet1Pin = 25; //Electromagnet pins, changed from A as the front and rear never turn on
const int ElectroMagnet2Pin = 24;
const int ElectroMagnet3Pin = 14;

void setup() {
  // put your setup code here, to run once:
  pinMode(ElectroMagnet1Pin, OUTPUT);
  pinMode(ElectroMagnet2Pin, OUTPUT);
  pinMode(ElectroMagnet3Pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(ElectroMagnet1Pin, 60);
  analogWrite(ElectroMagnet2Pin, 60);
  analogWrite(ElectroMagnet3Pin, 60);
  ElectroMagnet1On = true;
  ElectroMagnet2On = true;
  ElectroMagnet3On = true;
  Serial.print("4 Activated  ");
  while(1){
    stop(timedelay);
  }
}
