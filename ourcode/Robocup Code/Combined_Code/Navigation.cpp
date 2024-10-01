#include "Navigation.h"
#include "Sensors.h"

// LENGTH OF ROBOT ~ 35
// Define shared variables here
bool ReadyToDrive = false;
bool WeightDetected = false;
int NumWeightsCollected = 0;
bool TimeToGo = false;
// int time = millis();
bool homeReached = false;

int LengthOfRobot = 35;

int five_seconds = 5000;
int NOCHANGETHRESHOLD = 5;
int timeWeightDetected;

int motortime = 100;

WallDetectionState wallState = NO_WALL;
RobotState currentState = STARTING;
WeightDetectionState weightState = WEIGHT_NOT_DETECTED;
WeightPositionState WeightPosition = CLEAR;

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
        case FINISHED:
            Serial.println("DONE WOOHOO!");
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

    Serial.print("WeightPosition: ");
    switch (WeightPosition) {
        case CLEAR:
            Serial.println("CLEAR");
            break;
        case AGAINST_WALL:
            Serial.println("AGAINST_WALL");
            break;
        case LEFT_CLOSER_WALL:
            Serial.println("LEFT_CLOSER_WALL");
            break;
        case RIGHT_CLOSER_WALL:
            Serial.println("RIGHT_CLOSER_WALL");
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

int weightDetectionDuration = 5000; // Stay in WEIGHT_DETECTED for 5 seconds before reverting

void UpdateWeightState(uint32_t MiddleRight, uint32_t BottomRight, uint32_t MiddleLeft, uint32_t BottomLeft, bool FrontInduction) {
    // If weight is confirmed by induction sensor, set to WEIGHT_CONFIRMED
    if (FrontInduction) {
        weightState = WEIGHT_CONFIRMED;
        return;
    }

    // Only proceed with weight detection if it's not already confirmed
    if (weightState != WEIGHT_CONFIRMED) {
        // Check if the TOF sensor readings suggest a weight is detected
        if ((MiddleRight > (BottomRight + 10)) || (MiddleLeft > (BottomLeft + 10))) {
            if (weightState != WEIGHT_DETECTED) {
                // First time detecting weight, record the time
                timeWeightDetected = millis();
                weightState = WEIGHT_DETECTED;
            }
        } else {
            // If no weight is detected by sensors and sufficient time has passed, reset to WEIGHT_NOT_DETECTED
            if (weightState == WEIGHT_DETECTED && (millis() - timeWeightDetected) > five_seconds) {
                weightState = WEIGHT_NOT_DETECTED;
            }
        }
    }
}


void UpdateWeightPositionState(uint32_t MiddleRight, uint32_t BottomRight, uint32_t MiddleLeft, uint32_t BottomLeft, uint32_t TopMiddle, uint32_t TopLeft, uint32_t TopRight) {

    // If the robot is in the CLEAR state, do not update the state until a weight is collected
    if (WeightPosition == CLEAR && weightState != WEIGHT_CONFIRMED) {
        return;  // Remain in CLEAR state until a weight is detected and confirmed
    }

    // First, check TopMiddle
    if (TopMiddle < LengthOfRobot) {
        // Check which side is closer by comparing TopLeft and TopRight
        if (TopLeft > TopRight) {
            WeightPosition = LEFT_CLOSER_WALL;
        } else if (TopRight > TopLeft) {
            WeightPosition = RIGHT_CLOSER_WALL;
        } else {
            WeightPosition = AGAINST_WALL;  // If they are about the same, assume it's against the wall
        }
    }
    // If TopMiddle is not close enough, check side proximity
    else if ((MiddleRight < LengthOfRobot) && (MiddleLeft < LengthOfRobot)) {
        if (MiddleLeft < MiddleRight + 5) {
            WeightPosition = LEFT_CLOSER_WALL;
        } else if (MiddleRight < MiddleLeft + 5) {
            WeightPosition = RIGHT_CLOSER_WALL;
        } else {
            WeightPosition = AGAINST_WALL;  // Both sides are almost equal, consider it against the wall
        }
    } else {
        WeightPosition = CLEAR;  // If neither side is close, it's clear
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
                forward_right(motortime);
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
                forward_left(motortime);
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
            UpdateWeightPositionState(MiddleRight, BottomRight, MiddleLeft, BottomLeft, TopMiddle, TopLeft, TopRight);
            switch (WeightPosition) {
              case CLEAR:
                if(BottomLeft > 20 && BottomRight > 20) {
                  if (BottomLeft > (BottomRight + 5)) {
                    forward_right(motortime);
                  } else if (BottomRight > (BottomLeft + 5)) {
                    forward_left(motortime);
                  } else {
                    half_forward(10*motortime);
                  }                  
                } else {
                  if (BottomLeft > (BottomRight + 5)) {
                    full_turn_right(motortime);
                  } else if (BottomRight > (BottomLeft + 5)) {
                      full_turn_right(motortime);
                  } else {
                      half_forward(10*motortime);
                  }
                }
                break;
              case AGAINST_WALL:
                full_reverse(5*motortime);
                full_turn_left(5*motortime);
                forward_right(10*motortime);
                if(TopLeft < 20) {
                  full_forward(motortime);
                } else {
                  forward_left(motortime);
                }
                break;
              case LEFT_CLOSER_WALL:
                reverse_left(5*motortime);
                full_turn_left(5*motortime);
                half_forward(motortime);
                if(TopLeft < 20) {
                  full_forward(motortime);
                } else {
                  forward_left(motortime);
                }
                break;
              case RIGHT_CLOSER_WALL:
                reverse_right(5*motortime);
                full_turn_right(5*motortime);
                half_forward(motortime);
                if(TopRight < 20) {
                  full_forward(motortime);
                } else {
                  forward_left(motortime);
                }
                break;
              default:
                full_forward(motortime);
                break;
            }
            break;

          case WEIGHT_CONFIRMED:
            if((millis() - timeWeightDetected) > timeoutDuration) {
              weightState = WEIGHT_NOT_DETECTED;
            }
            
            if(NumWeightsCollected == 0) {
              half_forward(motortime);
                if (BackInduction) {
                stop(motortime);
                go_down();
                currentState = COLLECTING_WEIGHT;
                }
            }else if(NumWeightsCollected > 0) {
              stop(motortime);
              big_step_down();
              half_forward(10 * motortime);
              stop(motortime);
              little_step_down();
              currentState = COLLECTING_WEIGHT;
            }
            break;
        }
        break;

    case COLLECTING_WEIGHT:
        if (NumWeightsCollected == 0) {
            turn_on_electromagnet(0);
            Serial.println("Electromagnet 0 activated");
            stop(motortime);
            NumWeightsCollected++;
            go_up();
            weightState = WEIGHT_NOT_DETECTED;
            currentState = DRIVING;

        } else if (NumWeightsCollected == 1) {
            turn_on_electromagnet(1);
            stop(motortime);

            Serial.println("Electromagnet 1 activated");
           
            NumWeightsCollected++;
            go_up();
            weightState = WEIGHT_NOT_DETECTED;            
            currentState = DRIVING;
        } else if (NumWeightsCollected == 2) {
            turn_on_electromagnet(2);
            Serial.println("Electromagnet 2 activated");
            stop(motortime);           
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
          turn_off_electromagnet(0);
          turn_off_electromagnet(1);
          turn_off_electromagnet(2);
          stop(motortime);
          go_up();
          NumWeightsCollected = 0;
          currentState = FINISHED;
        }
        break;

      case FINISHED:
        stop(motortime);
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

