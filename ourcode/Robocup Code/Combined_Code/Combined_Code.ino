
#include "Combined_Code.h"
#include "IMU.h"

void setup() {
  setupSensors();
  setupActuators();
  setup_IMU();
  delay(2000);

}

void loop() {
  UpdateAll();
}
