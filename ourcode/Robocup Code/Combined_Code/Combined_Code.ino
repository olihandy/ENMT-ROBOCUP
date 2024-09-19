#include "Sensors.h"
#include "Actuators.h"
#include "Combined_Code.h"


void setup() {
  setupActuators();
  setupSensors();
}

void loop() {
  PrintInformation();
  full_forward(10);
  // go_down();
  // go_up();
}
