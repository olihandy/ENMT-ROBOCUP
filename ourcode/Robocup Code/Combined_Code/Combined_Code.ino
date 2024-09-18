#include "Sensors.h"
#include "Actuators.h"
#include "Combined_Code.h"

void setup() {
  setupActuators();
  setupSensors();
}

void loop() {
  PrintInformation();
  full_forward(100);
}
