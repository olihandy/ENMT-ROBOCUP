#include "Sensors.h"
#include "Actuators.h"
#include "Navigation.h"
#include "Combined_Code.h"


void setup() {
  setupActuators();
  setupSensors();
  
}

void loop() {
  Navigation();
}
