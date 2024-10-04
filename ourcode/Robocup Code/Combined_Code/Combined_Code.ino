
#include "Combined_Code.h"
#include "IMU.h"

void setup() {
  setupSensors();
  setupActuators();
  setup_IMU();
  // Wait for the Go button to be pressed
  Serial.println("Waiting for Go button to be pressed...");
  while (digitalRead(GoButtonPin) == LOW) {
    // Optional: Blink an LED or print a message while waiting
    delay(100);  // Small delay to avoid spamming the loop
  }
  
  Serial.println("Go button pressed! Starting...");
}

void loop() {
  UpdateAll();
}
