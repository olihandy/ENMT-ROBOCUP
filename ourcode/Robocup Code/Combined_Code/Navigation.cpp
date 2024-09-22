#include "Navigation.h"


// Define shared variables here
// bool ReadyToDrive = false;
// bool WeightDetected = false;
// bool ThreeWeightsCollected = false;
// bool TimeToGo = false;
// bool homeReached = false;

// int motortime = 10;

// int FrontInduction;
// int BackInduction;

// WallDetectionState wallState = NO_WALL;
// RobotState currentState = STARTING;
// WeightDetectionState weightState = WEIGHT_NOT_DETECTED;

void Navigation(void) {
  // GetTOF(TOFreadings);
  // GetElectroMagnet(electromagnetStates);
  // GetInduction(inductionSensorStates);

  // Implement your navigation logic here
  // Ensure it interacts correctly with actuators and sensors
  Serial.print("1");
}


  // int TopMiddle = 1;
  // int TopLeft = 1;
  // int TopRight = 1;
  // int MiddleLeft = 1;
  // int MiddleRight = 1;
  // int BottomLeft = 1;
  // int BottomRight = 1;

  // FrontInduction = 1;
  // BackInduction = 1;

  //  FrontInduction = !inductionSensorStates[0];
  //  BackInduction = !inductionSensorStates[1];

  // TOFreadings[0] = TopMiddle;
  // TOFreadings[1] = TopLeft;
  // TOFreadings[2] = TopRight;
  // TOFreadings[3] = MiddleLeft;
  // TOFreadings[4] = MiddleRight;
  // TOFreadings[5] = BottomLeft;
  // TOFreadings[6] = BottomRight;

  // if (TopLeft < 20) {
  //   if (TopRight < 20) {
  //     if (TopMiddle < 20) {
  //       wallState = WALL_AHEAD;
  //     } else {
  //       wallState = SLIT_DETECTED;
  //     }
  //   } else {
  //     wallState = LEFT_WALL_DETECTED;
  //   }
  // } else if (TopMiddle < 20) {
  //   wallState = SLAB_WALL_DETECTED;
  // } else if (TopRight < 20) {
  //   wallState = RIGHT_WALL_DETECTED;
  // } else {
  //   wallState = NO_WALL; // If no walls are detected, default to NO_WALL
  // }

  // if((MiddleRight > (BottomRight + 5)) ||(MiddleLeft > (BottomLeft + 5))) {
  // 	weightState = WEIGHT_DETECTED;
  // } else if(FrontInduction) {
  //   weightState = WEIGHT_CONFIRMED;
  // } else {
  //   weightState = WEIGHT_NOT_DETECTED;
  // }

  // switch (currentState) {
  //   case STARTING:
  //     //Take start readings, then set readyToDrive
  //     if(TopLeft > TopRight) {
  //       forward_left(motortime);
  //     } else {
  //       forward_right(motortime);
  //     }
  //     ReadyToDrive = 1;          
        
  //     if (ReadyToDrive) {
  //       currentState = DRIVING;
  //     }
  //     break;

  //   case DRIVING:

  //     switch (weightState) {
  //       case WEIGHT_NOT_DETECTED:
              
  //         switch (wallState) {
  //           case WALL_AHEAD:
  //             full_turn_right(motortime);
  //             break;
  //           case SLIT_DETECTED:
  //             full_forward(motortime);
  //             break;
  //           case LEFT_WALL_DETECTED:
  //             forward_right(motortime);
  //             break;
  //           case SLAB_WALL_DETECTED:
  //             full_reverse(motortime);
  //             full_turn_left(motortime);
  //             break;
  //           case RIGHT_WALL_DETECTED:
  //             forward_left(motortime);
  //             break;
  //           case NO_WALL:
  //           default:
  //             if(TopLeft > TopRight) {
  //               forward_left(motortime);
  //             } else {
  //               forward_right(motortime);
  //             }
  //             break;
  //             }
  //       break;
                
  //       case WEIGHT_DETECTED:
  //         if(BottomLeft > BottomRight + 2) {
  //           forward_right(motortime);
  //         } else if(BottomRight > BottomLeft + 2) {
  //           forward_left(motortime);
  //         } else {
  //           half_forward(motortime);
  //         }
  //         break;

  //       case WEIGHT_CONFIRMED:
  //         go_down();
  //         if(BackInduction) {
  //           turn_on_electromagnet();
  //         }
  //         break;
  //       }
  //       break;

  //   case COLLECTING_WEIGHT:
  //     if (ThreeWeightsCollected || TimeToGo) {
  //       currentState = RETURNING_HOME;
  //     }
  //     break;

  //   case RETURNING_HOME:
  //     // Logic for the Returning Home state
  //     // Example condition: if the robot has returned home
  //     if (homeReached) {
  //       // Drop weights
  //     }
  //     break;
  //     }
  // }


