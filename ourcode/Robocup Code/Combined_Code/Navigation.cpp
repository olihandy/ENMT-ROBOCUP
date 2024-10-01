#include "Navigation.h"
#include "Sensors.h"

// Define shared variables here
bool ReadyToDrive = false;
bool WeightDetected = false;
int NumWeightsCollected = 0;
bool TimeToGo = false;
bool homeReached = false;

int NOCHANGETHRESHOLD = 5;
int timeWeightDetected;

int motortime = 100;

WallDetectionState wallState = NO_WALL;
RobotState currentState = STARTING;
WeightDetectionState weightState = WEIGHT_NOT_DETECTED;

// Add these global variables to store previous sensor states
uint16_t prevTOFReadings[7];
bool prevInductionSensorStates[2];

bool CheckForSensorUpdates() {
    bool sensorChanged = false;

    // Check for changes in TOF readings
    for (int i = 0; i < numReadings; i++) {
        if (abs(TOFreadings[i] - prevTOFReadings[i]) > NOCHANGETHRESHOLD) {  // Only consider changes greater than 5
            sensorChanged = true; // Mark as changed
            prevTOFReadings[i] = TOFreadings[i]; // Update previous readings
        }
    }

    // Check for changes in induction sensor states
    bool* inductionSensorState = GetInduction(); // Get current induction sensor states
    for (int i = 0; i < numInductiveSensors; i++) {
        if (inductionSensorState[i] != prevInductionSensorStates[i]) {
            sensorChanged = true; // Mark as changed
            prevInductionSensorStates[i] = inductionSensorState[i]; // Update previous state
        }
    }

    return sensorChanged;
}

void PrintStates(void) {
    // Print out all the states for debugging

    Serial.print("ReadyToDrive: ");
    Serial.println(ReadyToDrive ? "True" : "False");

    Serial.print("WeightDetected: ");
    Serial.println(WeightDetected ? "True" : "False");

    Serial.print("WeightsCollected: ");
    Serial.println(NumWeightsCollected);

    Serial.print("TimeToGo: ");
    Serial.println(TimeToGo ? "True" : "False");

    Serial.print("HomeReached: ");
    Serial.println(homeReached ? "True" : "False");

    Serial.print("WallState: ");
    switch (wallState) {
        case NO_WALL:
            Serial.println("NO_WALL");
            break;
        case WALL_AHEAD:
            Serial.println("WALL_AHEAD");
            break;
        case SLIT_DETECTED:
            Serial.println("SLIT_DETECTED");
            break;
        case LEFT_WALL_DETECTED:
            Serial.println("LEFT_WALL_DETECTED");
            break;
        case SLAB_WALL_DETECTED:
            Serial.println("SLAB_WALL_DETECTED");
            break;
        case RIGHT_WALL_DETECTED:
            Serial.println("RIGHT_WALL_DETECTED");
            break;
    }

    Serial.print("CurrentState: ");
    switch (currentState) {
        case STARTING:
            Serial.println("STARTING");
            break;
        case DRIVING:
            Serial.println("DRIVING");
            break;
        case COLLECTING_WEIGHT:
            Serial.println("COLLECTING_WEIGHT");
            break;
        case RETURNING_HOME:
            Serial.println("RETURNING_HOME");
            break;
    }

    Serial.print("WeightState: ");
    switch (weightState) {
        case WEIGHT_NOT_DETECTED:
            Serial.println("WEIGHT_NOT_DETECTED");
            break;
        case WEIGHT_DETECTED:
            Serial.println("WEIGHT_DETECTED");
            break;
        case WEIGHT_CONFIRMED:
            Serial.println("WEIGHT_CONFIRMED");
            break;
    }

    Serial.println("----------------------");
}

void UpdateWallState(uint32_t TopLeft, uint32_t TopMiddle, uint32_t TopRight) {
  if (TopLeft < 30) {
    if (TopRight < 30) {
      if (TopMiddle < 30) {
        wallState = WALL_AHEAD;
      } else {
        wallState = SLIT_DETECTED;
      }
    } else {
      wallState = LEFT_WALL_DETECTED;
    }
  } else if (TopMiddle < 30) {
    wallState = SLAB_WALL_DETECTED;
  } else if (TopRight < 30) {
    wallState = RIGHT_WALL_DETECTED;
  } else {
    wallState = NO_WALL;
  }
}

void UpdateWeightState(uint32_t MiddleRight, uint32_t BottomRight, uint32_t MiddleLeft, uint32_t BottomLeft, bool FrontInduction) {
    // Confirm weight based on FrontInduction
    if (FrontInduction) {
        // Set to WEIGHT_CONFIRMED only if it's not already confirmed
        if (weightState != WEIGHT_CONFIRMED) {
            weightState = WEIGHT_CONFIRMED;
        }
    } 
    // Only update if weight is not confirmed
    else if (weightState != WEIGHT_CONFIRMED) {
        if ((MiddleRight > (BottomRight)) || (MiddleLeft > (BottomLeft))) {
            timeWeightDetected = millis();
            weightState = WEIGHT_DETECTED;
        } else {
            weightState = WEIGHT_NOT_DETECTED;
        }
    }
}


void Navigation(uint32_t TopMiddle, uint32_t TopLeft, uint32_t TopRight, uint32_t MiddleLeft, uint32_t MiddleRight, uint32_t BottomLeft, uint32_t BottomRight, bool BackInduction) {

    switch (currentState) {
      case STARTING:
        // Take start readings, then set readyToDrive
        if (TopLeft > (TopRight + 20)) {
          forward_left(motortime);
        } else {
          forward_right(motortime);
        }
        ReadyToDrive = true;

        if (ReadyToDrive) {
          currentState = DRIVING;
        }
        break;

      case DRIVING:
        switch (weightState) {
          case WEIGHT_NOT_DETECTED:
            switch (wallState) {
              case WALL_AHEAD:
                full_reverse(motortime);
                break;
              case SLIT_DETECTED:
                full_forward(motortime);
                break;
              case LEFT_WALL_DETECTED:
                full_turn_right(motortime);
                break;
              case SLAB_WALL_DETECTED:
                full_reverse(10*motortime);
                if (TopLeft > (TopRight)) {
                  full_turn_left(5*motortime);
                } else {
                  full_turn_right(5*motortime);
                }
                break;
              case RIGHT_WALL_DETECTED:
                full_turn_left(motortime);
                break;
             case NO_WALL:
             default:
               if ((TopLeft > TopRight + 20) && (TopLeft > 30)) {
                  forward_left(motortime);
                } else {
                  forward_right(motortime);
                }
                break;
                }
              break;

          case WEIGHT_DETECTED:
            if (BottomLeft > (BottomRight + 2)) {
              forward_right(motortime);
            } else if (BottomRight > (BottomLeft + 2)) {
              forward_left(motortime);
            } else {
              half_forward(motortime);
            }
            break;

          case WEIGHT_CONFIRMED:
            if((millis() - timeWeightDetected) > timeoutDuration) {
              weightState = WEIGHT_NOT_DETECTED;
            }
            if(NumWeightsCollected == 0) {
              half_forward(motortime);
            }else if(NumWeightsCollected > 0) {
              stop(motortime);
              go_down();
              half_forward(motortime * (15 - NumWeightsCollected));
              stop(motortime);
              currentState = COLLECTING_WEIGHT;
            }
            if (BackInduction) {
              stop(5*motortime);
              go_down();
              currentState = COLLECTING_WEIGHT;
              }
            break;
        }
        break;

    case COLLECTING_WEIGHT:
        if (NumWeightsCollected == 0) {
            turn_on_electromagnet(0);
            Serial.println("Electromagnet 0 activated");

            NumWeightsCollected++;
            go_up();
            weightState = WEIGHT_NOT_DETECTED;
            currentState = DRIVING;

        } else if (NumWeightsCollected == 1) {
            turn_on_electromagnet(1);
            Serial.println("Electromagnet 1 activated");
           
            NumWeightsCollected++;
            go_up();
            weightState = WEIGHT_NOT_DETECTED;            
            currentState = DRIVING;
        } else if (NumWeightsCollected == 2) {
            turn_on_electromagnet(2);
            Serial.println("Electromagnet 2 activated");
           
            go_up();
            currentState = DRIVING;
            weightState = WEIGHT_NOT_DETECTED;
            NumWeightsCollected++;
            TimeToGo = true;
        }
        
        weightState = WEIGHT_NOT_DETECTED; // This will allow you to transition out of CONFIRMED
        if (TimeToGo) {
            currentState = RETURNING_HOME;
        } else {
            currentState = DRIVING;
        }
        break;

      case RETURNING_HOME:
        stop(motortime);
        homeReached = 1;
        if (homeReached) {
          stop(motortime);
          go_down();
          turn_off_electromagnet(1);
          turn_off_electromagnet(2);
          turn_off_electromagnet(3);
          delay(10);
          go_up();
        }
        break;
    }
  }

void UpdateAll(void) {
    // Update all sensor readings
    UpdateTOFReadings(); 
    bool* electromagnetState = GetElectroMagnet();
    bool* inductionSensorState = GetInduction();

    // Update states
    UpdateWallState(GetAverageTOFReading(1), GetAverageTOFReading(0), GetAverageTOFReading(2));
    UpdateWeightState(GetAverageTOFReading(4), GetAverageTOFReading(6), GetAverageTOFReading(3), GetAverageTOFReading(5), inductionSensorState[0]);
    
    // Check for sensor changes and update lastChangeTime if necessary
    if (CheckForSensorUpdates()) {
        lastChangeTime = millis(); // Reset timer on sensor change
    }

    // If no change detected within the timeout duration, execute reverse navigation
    if (millis() - lastChangeTime > timeoutDuration) {
        Serial.println("No sensor change detected for 5 seconds. Executing reverse navigation.");
        full_reverse(10 * motortime);
        full_turn_right(10 * motortime);
    } else {
        // Normal navigation logic based on updated sensor data
        Navigation(GetAverageTOFReading(0), GetAverageTOFReading(1), GetAverageTOFReading(2), 
                   GetAverageTOFReading(3), GetAverageTOFReading(4), GetAverageTOFReading(5), 
                   GetAverageTOFReading(6), inductionSensorState[1]);
    }

    // Print debugging information
    PrintInformation();
    PrintStates();
}
