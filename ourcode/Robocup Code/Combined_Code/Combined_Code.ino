
#include "Combined_Code.h"


void setup() {
  setupSensors();
  setupActuators();
  delay(1000);
}

void loop() {

  UpdateAll();
}
