#include "Sensors.h"
#include "Actuators.h"
#include "Navigation.h"
#include "Combined_Code.h"


void setup() {
  setupSensors();
  setupActuators();
}

void loop() {
  Navigation();
}
