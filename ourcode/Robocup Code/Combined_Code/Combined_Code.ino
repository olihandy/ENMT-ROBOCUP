
#include "Combined_Code.h"
#include "IMU.h"


void setup() {
  setupSensors();
  setupActuators();
  delay(2000);
  setup_IMU();
}

void loop() {
  //_forward(timedelay);
  UpdateAll();
}
