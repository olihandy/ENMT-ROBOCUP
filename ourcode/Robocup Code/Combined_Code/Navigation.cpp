#include "Navigation.h"
#include "Actuators.h"
#include "Sensors.h"

bool ReadyToDrive = 0;
bool WeightDetected = 0;
bool ThreeWeightsCollected = 0;
bool TimeToGo = 0;
bool homeReached = 0;

int motortime = 10;

int programState = 1;
int elapsed_time = 0;

//ELECTROMAGNET
const int ElectroMagnet1Pin = 25;
const int ElectroMagnet2Pin = 24;
const int ElectroMagnet3Pin = 14;
bool ElectroMagnet1On = false;
bool ElectroMagnet2On = false;
bool ElectroMagnet3On = false;
const int numElectroMagnets = 3;


//INDUCTION
const int FrontInductionPin = 27; 
const int BackInductionPin = 26;
const int numInductiveSensors = 2;


//TOF
const byte SX1509_ADDRESS = 0x3F;
const int VL53L0X_ADDRESS_START = 0x30;
const int VL53L1X_ADDRESS_START = 0x35;

//number of sensors in system.
const uint8_t sensorCountL1 = 3;
const uint8_t sensorCountL0 = 4; 

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL1[sensorCountL1] = {0, 1, 2};
const uint8_t xshutPinsL0[sensorCountL0] = {3, 4, 5, 6};

 SX1509 io;  // Create an SX1509 object to be used throughout
 VL53L1X sensorsL1[sensorCountL1];
 VL53L0X sensorsL0[sensorCountL0];

uint16_t TopRight, TopLeft, TopMiddle, MiddleLeft, MiddleRight, BottomLeft, BottomRight;
const int numReadings = 7;

uint16_t TOFreadings[numReadings];
bool electromagnetStates[numElectroMagnets];
bool inductionSensorStates[numInductiveSensors];

WallDetectionState wallState = NO_WALL;
RobotState currentState = STARTING;
WeightDetectionState weightState = WEIGHT_NOT_DETECTED;

void Navigation(void) {
  GetTOF(TOFreadings);
  GetElectroMagnet(electromagnetStates);
  GetInduction(inductionSensorStates);

  TOFreadings[0] = TopMiddle;
  TOFreadings[1] = TopLeft;
  TOFreadings[2] = TopRight;
  TOFreadings[3] = MiddleLeft;
  TOFreadings[4] = MiddleRight;
  TOFreadings[5] = BottomLeft;
  TOFreadings[6] = BottomRight;

  if (TopLeft < 20) {
    if (TopRight < 20) {
      if (TopMiddle < 20) {
        wallState = WALL_AHEAD;
      } else {
        wallState = SLIT_DETECTED;
      }
    } else {
      wallState = LEFT_WALL_DETECTED;
    }
  } else if (TopMiddle < 20) {
    wallState = SLAB_WALL_DETECTED;
  } else if (TopRight < 20) {
    wallState = RIGHT_WALL_DETECTED;
  } else {
    wallState = NO_WALL; // If no walls are detected, default to NO_WALL
  }

  if((MiddleRight > (BottomRight + 5)) ||(MiddleLeft > (BottomLeft + 5))) {
  	weightState = WEIGHT_DETECTED;
  } else if(!inductionSensorStates[0]) {
    weightState = WEIGHT_CONFIRMED;
  } else {
    weightState = WEIGHT_NOT_DETECTED;
  }




  switch (currentState) {
    case STARTING:
      //Take start readings, then set readyToDrive
      if(TopLeft > TopRight) {
        forward_left(motortime);
      } else {
        forward_right(motortime);
      }
      ReadyToDrive = 1;          
        
      if (ReadyToDrive) {
        currentState = DRIVING;
      }
      break;

      case DRIVING:

        switch (weightState) {
          case WEIGHT_NOT_DETECTED:
              
            switch (wallState) {
              case WALL_AHEAD:
                full_turn_right(motortime);
                break;
              case SLIT_DETECTED:
                full_forward(motortime);
                break;
              case LEFT_WALL_DETECTED:
                forward_right(motortime);
                break;
              case SLAB_WALL_DETECTED:
                full_reverse(motortime);
                full_turn_left(motortime);
                break;
              case RIGHT_WALL_DETECTED:
                forward_left(motortime);
                break;
              case NO_WALL:
              default:
                if(TopLeft > TopRight) {
                  forward_left(motortime);
                } else {
                  forward_right(motortime);
                }
                break;
                }
          break;
                
          case WEIGHT_DETECTED:
            if(BottomLeft > BottomRight + 2) {
              forward_right(motortime);
            } else if(BottomRight > BottomLeft + 2) {
              forward_left(motortime);
            } else {
              half_forward(motortime);
            }
            break;

          case WEIGHT_CONFIRMED:
            go_down();
            if(!inductionSensorStates[1]) {
              turn_on_electromagnet();
            }
            break;
        }
        break;

      case COLLECTING_WEIGHT:
        if (ThreeWeightsCollected || TimeToGo) {
          currentState = RETURNING_HOME;
        }
        break;

      case RETURNING_HOME:
        // Logic for the Returning Home state
        // Example condition: if the robot has returned home
        if (homeReached) {
          // Drop weights
        }
        break;
        }
  }


