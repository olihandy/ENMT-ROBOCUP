#include "Sensors.h"
#include "Actuators.h"

void setup() {
  setupActuators();
  setupSensors();
}

void loop() {
  // Get the readings from the sensors
  uint16_t TOFreadings[numReadings];
  bool electromagnetStates[numElectroMagnets];
  bool inductionSensorStates[numInductiveSensors];
  // Get sensor readings and store them in the array
  GetTOF(TOFreadings);
  GetElectroMagnet(electromagnetStates);
  GetInduction(inductionSensorStates);

  // Example program state, you can update this according to your logic
  int programState = 0;

  // Call PrintInformation with the array of readings and program state
  PrintInformation(TOFreadings, electromagnetStates, inductionSensorStates, programState);

  delay(10); // Adjust delay as needed
  go_down();
  stop(100);
  full_turn_left(1000);
  go_up();
  stop(100);
  full_turn_right(1000);
}
