
#include "Combined_Code.h"


void setup() {
  setupSensors();
  setupActuators();
  delay(2000);
}

void loop() {

  UpdateAll();
}
