#include "Navigation.h"
#include "Sensors.h"

//--------------------------------------------------------------------------------------------------------//
//--------------------------------------- Shared Variables ------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

bool ReadyToDrive = false;
bool WeightDetected = false;
int NumWeightsCollected = 0;
bool TimeToGo = false;
bool homeReached = false;

int LengthOfRobot = 35;
int five_seconds = 5000;
int NOCHANGETHRESHOLD = 5;
int timeWeightDetected;
unsigned long lastOrientationChangeTime = 0;
float lastOrientation = 0;

int stepper_motor_fast = 20;
int stepper_motor_slow = 40;
int motortime = 100;

WallDetectionState wallState = NO_WALL;
RobotState currentState = STARTING;
WeightDetectionState weightState = WEIGHT_NOT_DETECTED;

int numTOFs =7;

extern bool electromagnetStates[3];
extern bool inductionSensorStates[2];
extern uint16_t averagedTOFreadings[7];

const extern int numReadings;
extern uint16_t averagedTOFreadings[];

uint16_t TopMiddle = averagedTOFreadings[0];
uint16_t TopLeft = averagedTOFreadings[1];
uint16_t TopRight = averagedTOFreadings[2];
uint16_t MiddleLeft = averagedTOFreadings[3];
uint16_t MiddleRight = averagedTOFreadings[4];
uint16_t BottomLeft = averagedTOFreadings[5];
uint16_t BottomRight = averagedTOFreadings[6];
bool BackInduction = !digitalRead(BackInductionPin);
bool FrontInduction = !digitalRead(FrontInductionPin);

uint16_t prevTOFReadings[7];
bool prevInductionSensorStates[2];

//--------------------------------------------------------------------------------------------------------//
//----------------------------------- Sensor Update and State Printing -----------------------------------//
//--------------------------------------------------------------------------------------------------------//

void CheckForSensorUpdates() {
  bool sensorChanged = false;
  for (int i = 0; i < numTOFs; i++) {
    if (abs(averagedTOFreadings[i] - prevTOFReadings[i]) > NOCHANGETHRESHOLD) {
      sensorChanged = true; // Sensor value has changed significantly
      prevTOFReadings[i] = averagedTOFreadings[i];
    }
  }
  // Track time for sensor change timeout
  static unsigned long lastChangeTime = millis();
  unsigned long currentTime = millis();
  // Update last change time if any sensor changes are detected
  if (sensorChanged) {
    lastChangeTime = currentTime;
  }
  // If no sensor changes detected for the timeout duration, perform reverse navigation
  if (currentTime - lastChangeTime > timeoutDuration) {
    Serial.println("No sensor change detected for timeout duration. Executing reverse navigation.");
    full_reverse(10 * motortime);    // Perform reverse
    full_turn_right(10 * motortime); // Perform turn
  }
}



void checkOrientation(void) {
  float currentOrientation = ori[0];
  // Check if the orientation has changed significantly (beyond a small tolerance)
  if (abs(currentOrientation - lastOrientation) > 5.0) {
    // Reset the timer if orientation has changed
    lastOrientationChangeTime = millis();
    lastOrientation = currentOrientation;
  }

  // If the robot has been facing the same direction for too long
  if (millis() - lastOrientationChangeTime > five_seconds) {
    // Time to turn around
    full_turn_left(10*motortime);
    Serial.println("No direction change detected, turning");
    
    // After turning, reset the timer and the orientation
    lastOrientationChangeTime = millis();
    lastOrientation = ori[0]; // getOrientation() retrieves the updated orientation after turn
  }
}

void PrintStates(void) {
  // Print out all states for debugging
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
    case NO_WALL: Serial.println("NO_WALL"); break;
    case WALL_AHEAD: Serial.println("WALL_AHEAD"); break;
    case SLIT_DETECTED: Serial.println("SLIT_DETECTED"); break;
    case LEFT_WALL_DETECTED: Serial.println("LEFT_WALL_DETECTED"); break;
    case SLAB_WALL_DETECTED: Serial.println("SLAB_WALL_DETECTED"); break;
    case RIGHT_WALL_DETECTED: Serial.println("RIGHT_WALL_DETECTED"); break;
  }

  Serial.print("CurrentState: ");
  switch (currentState) {
    case STARTING: Serial.println("STARTING"); break;
    case DRIVING: Serial.println("DRIVING"); break;
    case COLLECTING_WEIGHT: Serial.println("COLLECTING_WEIGHT"); break;
    case RETURNING_HOME: Serial.println("RETURNING_HOME"); break;
    case FINISHED: Serial.println("FINISHED"); break;
  }

  Serial.print("WeightState: ");
  switch (weightState) {
    case WEIGHT_NOT_DETECTED: Serial.println("WEIGHT_NOT_DETECTED"); break;
    case WEIGHT_DETECTED: Serial.println("WEIGHT_DETECTED"); break;
    case WEIGHT_CONFIRMED: Serial.println("WEIGHT_CONFIRMED"); break;
  }

  Serial.println("----------------------");
}

//--------------------------------------------------------------------------------------------------------//
//----------------------------------- Wall and Weight State Updates --------------------------------------//
//--------------------------------------------------------------------------------------------------------//

void UpdateWallState(void) {
  TopMiddle = averagedTOFreadings[0];
  TopLeft = averagedTOFreadings[1];
  TopRight = averagedTOFreadings[2];
  if (TopLeft < 300) {
    if (TopRight < 300) {
      if (TopMiddle < 300) {
        wallState = WALL_AHEAD;
      } else {
        wallState = SLIT_DETECTED;
      }
    } else {
      wallState = LEFT_WALL_DETECTED;
    }
  } else if (TopMiddle < 300) {
    wallState = SLAB_WALL_DETECTED;
  } else if (TopRight < 300) {
    wallState = RIGHT_WALL_DETECTED;
  } else {
    wallState = NO_WALL;
  }
}

void UpdateWeightState(void) {
  TopMiddle = averagedTOFreadings[0];
  TopLeft = averagedTOFreadings[1];
  TopRight = averagedTOFreadings[2];
  MiddleLeft = averagedTOFreadings[3];
  MiddleRight = averagedTOFreadings[4];
  BottomLeft = averagedTOFreadings[5];
  BottomRight = averagedTOFreadings[6];
  FrontInduction = !digitalRead(FrontInductionPin);
  if (FrontInduction) {
    if (weightState != WEIGHT_CONFIRMED) {
      weightState = WEIGHT_CONFIRMED;
    }
  } else if (weightState != WEIGHT_CONFIRMED) {
    if (((MiddleRight > (BottomRight + 100)) && (TopRight > (MiddleRight + 150))) || ((MiddleLeft > (BottomLeft + 100)) && (TopLeft > (MiddleLeft + 150)))) {
      timeWeightDetected = millis();
      weightState = WEIGHT_DETECTED;
    } else {
      weightState = WEIGHT_NOT_DETECTED;
    }
  }
}

//--------------------------------------------------------------------------------------------------------//
//----------------------------------- Navigation Logic ---------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//

void Navigation(void) {
  TopMiddle = averagedTOFreadings[0];
  TopLeft = averagedTOFreadings[1];
  TopRight = averagedTOFreadings[2];
  MiddleLeft = averagedTOFreadings[3];
  MiddleRight = averagedTOFreadings[4];
  BottomLeft = averagedTOFreadings[5];
  BottomRight = averagedTOFreadings[6];
  BackInduction = !digitalRead(BackInductionPin);  
    switch (currentState) {
      case STARTING:
        // Take start readings, then set readyToDrive
        if (TopLeft > (TopRight + 200)) {
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
        if(ori[2] < -5) {
          full_reverse(10*motortime);
        }
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
              full_forward(motortime);
              break;
                }
            break;

          case WEIGHT_DETECTED:
            if(TopMiddle < 200) {
              full_reverse(5*motortime);
              if(TopLeft > TopRight){
                full_turn_left(5*motortime);
              } else {
                full_turn_right(5*motortime);
              }
            } else if(TopLeft < 100) {
              full_reverse(5*motortime);
              full_turn_left(3*motortime);

            } else if(TopRight < 100) {
              full_reverse(5*motortime);
              full_turn_right(3*motortime);
            } else {
              if(BottomLeft < 400 || BottomRight <400) {
                if (BottomLeft > (BottomRight + 50)) {
                  forward_right_right(3*motortime);
                } else if (BottomRight > (BottomLeft + 50)) {
                  forward_left_left(3*motortime);
                } else {
                  half_forward(motortime);
                } 
              } else {
              if (BottomLeft > (BottomRight + 50)) {
                forward_right(3*motortime);
              } else if (BottomRight > (BottomLeft + 50)) {
                forward_left(3*motortime);
              } else {
                half_forward(motortime);                
              }
            }
            }
            break;
          case WEIGHT_CONFIRMED:
            if((millis() - timeWeightDetected) > timeoutDuration) {
              weightState = WEIGHT_NOT_DETECTED;
            }
            
            if(NumWeightsCollected == 0) {
              half_forward(motortime);
              if (BackInduction) {
                stop_blocking(motortime);
                go_down(stepper_motor_fast);
                currentState = COLLECTING_WEIGHT;
                }
            }else if(NumWeightsCollected > 0) {
              stop_blocking(motortime);
              big_step_down(stepper_motor_fast);
              half_forward(10 * motortime);
              stop_blocking(motortime);
              little_step_down(stepper_motor_fast);
              currentState = COLLECTING_WEIGHT;
            }
            break;
        }
        break;

    case COLLECTING_WEIGHT:
        if (NumWeightsCollected == 0) {
            turn_on_electromagnet(0);
            Serial.println("Electromagnet 0 activated");
            stop_blocking(motortime);
            NumWeightsCollected++;
            go_up(stepper_motor_fast);
            weightState = WEIGHT_NOT_DETECTED;
            currentState = DRIVING;

        } else if (NumWeightsCollected == 1) {
            turn_on_electromagnet(1);
            stop_blocking(motortime);
            Serial.println("Electromagnet 1 activated");
            NumWeightsCollected++;
            go_up(stepper_motor_fast);
            weightState = WEIGHT_NOT_DETECTED;            
            currentState = DRIVING;

        } else if (NumWeightsCollected == 2) {
            turn_on_electromagnet(2);
            Serial.println("Electromagnet 2 activated");
            stop_blocking(motortime);
            go_up(stepper_motor_fast);
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
          go_down(stepper_motor_fast);
          turn_off_electromagnet(0);
          turn_off_electromagnet(1);
          turn_off_electromagnet(2);
          stop(motortime);
          go_up(stepper_motor_fast);
          NumWeightsCollected = 0;
          currentState = FINISHED;
        }
        break;

      case FINISHED:
        stop(motortime);
        delay(100);
        homeReached = 0;
        currentState = STARTING;
    }
  }


