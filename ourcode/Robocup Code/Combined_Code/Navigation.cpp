#include "Navigation.h"
#include "Sensors.h"

// Define shared variables here
bool ReadyToDrive = false;
bool WeightDetected = false;
int NumWeightsCollected = 0;
bool TimeToGo = false;
bool homeReached = false;

int motortime = 10;

WallDetectionState wallState = NO_WALL;
RobotState currentState = STARTING;
WeightDetectionState weightState = WEIGHT_NOT_DETECTED;

void PrintStates(void) {
    // Print out all the states for debugging

    Serial.print("ReadyToDrive: ");
    Serial.println(ReadyToDrive ? "True" : "False");

    Serial.print("WeightDetected: ");
    Serial.println(WeightDetected ? "True" : "False");

    Serial.print("ThreeWeightsCollected: ");
    Serial.println(ThreeWeightsCollected ? "True" : "False");

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

void UpdateWallState(uint16_t TopLeft, uint16_t TopMiddle, uint16_t TopRight) {
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

void UpdateWeightState(uint16_t MiddleRight, uint16_t BottomRight, uint16_t MiddleLeft, uint16_t BottomLeft, bool FrontInduction) {
    if (FrontInduction) {
        weightState = WEIGHT_CONFIRMED;
    } else if ((MiddleRight > (BottomRight + 10)) || (MiddleLeft > (BottomLeft + 10))) {
        weightState = WEIGHT_DETECTED;
    } else {
        weightState = WEIGHT_NOT_DETECTED;
    }
}

void Navigation(void) {
  
    UpdateTOFReadings(); // Update the circular buffers with new TOF data
    bool* electromagnetState = GetElectroMagnet();
    bool* inductionSensorState = GetInduction();

    // Use averaged readings instead of raw readings
    uint16_t TopMiddle = GetAverageTOFReading(0);
    uint16_t TopLeft = GetAverageTOFReading(1);
    uint16_t TopRight = GetAverageTOFReading(2);
    uint16_t MiddleLeft = GetAverageTOFReading(3);
    uint16_t MiddleRight = GetAverageTOFReading(4);
    uint16_t BottomLeft = GetAverageTOFReading(5);
    uint16_t BottomRight = GetAverageTOFReading(6);

    // Update Front and Back Induction states
    bool FrontInduction = inductionSensorState[0];
    bool BackInduction = inductionSensorState[1];

    bool FrontELectromagnet = electromagnetState[0];
    bool MiddleELectromagnet = electromagnetState[1];
    bool BackELectromagnet = electromagnetState[2];

    // Update wall state
    UpdateWallState(TopLeft, TopMiddle, TopRight); // TopLeft, TopMiddle, TopRight
    UpdateWeightState(MiddleRight, BottomRight, MiddleLeft, BottomLeft, FrontInduction);
    
    PrintInformation();
    PrintStates();

    switch (currentState) {
      case STARTING:
        // Take start readings, then set readyToDrive
        if (TopLeft > (TopRight + 20)) {
          forward_left(3*motortime);
        } else {
          forward_right(3*motortime);
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
                full_turn_right(3*motortime);
                break;
              case SLAB_WALL_DETECTED:
                full_reverse(5*motortime);
                if (TopLeft > (TopRight + 20)) {
                  full_turn_left(3*motortime);
                } else {
                  full_turn_right(3*motortime);
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
            if (BackInduction) {
              if(NumWeightsCollected = 0) {
                turn_on_electromagnet(0);
                NumWeightsCollected ++;
                go_up();
                weightState = COLLECTING_WEIGHT;
              } else if(NumWeightsCollected = 1) {
                turn_on_electromagnet(1);
                go_up();
                weightState = COLLECTING_WEIGHT;
              } else if(NumWeightsCollected = 2) {
                turn_on_electromagnet(2);
                go_up();
                weightState = COLLECTING_WEIGHT;
                NumWeightsCollected ++;
                TimeToGo = true;
              }
            }
            break;
        }
        break;

      case COLLECTING_WEIGHT:
      
        if (TimeToGo) {
          currentState = RETURNING_HOME;
        } else {
          WeightState = WEIGHT_NOT_DETECTED;
          currentState = DRIVING;
        }
        break;

      case RETURNING_HOME:
        // Logic for the Returning Home state
        if (homeReached) {
          // Drop weights
        }
        break;
    }
}
