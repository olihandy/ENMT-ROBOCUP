#include "ReturningHome.h"

// State definitions for wall detection and robot actions
enum WallState {
  TURN_TO_ORIENTATION,  // 0 - Correct orientation towards home
  DRIVE_FORWARD,        // 1 - Drive forward until a wall is found
  FOLLOW_WALL,          // 2 - Follow the wall (left, right, or none)
  NO_WALL_REORIENT      // 3 - No wall, reorient and move forward
};

WallState currentWallState = TURN_TO_ORIENTATION;
int FoundWall = 0;           
// int homeAngle = 225;        
int homeAngle = 135;
extern bool homeReached;
extern int motortime;


uint16_t Middle = averagedTOFreadings[0];
uint16_t Left = averagedTOFreadings[1];
uint16_t Right = averagedTOFreadings[2];


void return_home(void) {c:\Users\aos30\OneDrive - University of Canterbury\University\2024\ENMT ENME 301\Robocup\ENMT-ROBOCUP\ourcode\Robocup Code\Combined_Code\DropWeights.cpp
  Serial.print(currentWallState);
  Middle = averagedTOFreadings[0];
  Left = averagedTOFreadings[1];
  Right = averagedTOFreadings[2];


  if (!homeReached) {
    switch (currentWallState) {
      case TURN_TO_ORIENTATION: {
        // Correct the orientation of the robot towards home angle
        int robot_orientation = ori[0];
        if (robot_orientation > (homeAngle + 5)) {
          full_turn_left(motortime);
        } else if (robot_orientation < (homeAngle - 5)) {
          full_turn_right(motortime);
        } else {
          currentWallState = DRIVE_FORWARD;
        }
        break;
      }

      case DRIVE_FORWARD: {
        full_forward(motortime);
        if ((Right < 300)) {
          FoundWall = 1;  // Detected right wall
          currentWallState = FOLLOW_WALL;  // Transition to follow right wall
        } else if ((Left < 300)) {
          FoundWall = 2;  // Detected left wall
          currentWallState = FOLLOW_WALL;  // Transition to follow left wall
        } else {
          FoundWall = 0;  // No wall detected, just keep driving forward
        }
        break;
      }

      case FOLLOW_WALL: {
        if (FoundWall == 1) {  // Right wall
          if (Right > 350) {  // Too far from the right wall
            forward_right(motortime);  // Turn slightly towards the wall
          } else if (Right < 200) {  // Too close to the right wall
            forward_left(motortime);  // Turn slightly away from the wall
          } else {
            full_forward(motortime);  // Move forward
          }

          // Obstacle detected in front
          if (Middle < 300) {
            full_turn_left_blocking(10 * motortime);  // Turn left 90 degrees to avoid obstacle
          }

        } else if (FoundWall == 2) {  // Left wall
          // Stay close to the left wall
          if (Left > 350) {  // Too far from the left wall
            forward_left(motortime);  // Turn slightly towards the wall
          } else if (Left < 200) {  // Too close to the left wall
            forward_right(motortime);  // Turn slightly away from the wall
          } else {
            full_forward(motortime);  // Move forward
          }

          // Obstacle detected in front
          if (Middle < 300) {
            full_turn_right_blocking(10 * motortime);  // Turn right 90 degrees to avoid obstacle
          }

        } else if (FoundWall == 3) {  // No wall detected
          currentWallState = NO_WALL_REORIENT;  // Transition to reorientation state
        }

        break;
      }

      case NO_WALL_REORIENT: {
        int robot_orientation = ori[0];
        if (robot_orientation > (homeAngle + 5)) {
          full_turn_left(motortime);
        } else if (robot_orientation < (homeAngle - 5)) {
          full_turn_right(motortime);
        } else {
          full_forward(motortime);
          currentWallState = DRIVE_FORWARD;
        }
        break;
      }
    }

    if (ColorCompareHome()) {
      homeReached = true;
    }
  }
}
