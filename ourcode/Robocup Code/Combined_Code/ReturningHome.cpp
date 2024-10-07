#include "ReturningHome.h"

int FoundWall = 0;           // State to detect if the robot found a wall
int StartingAngle = 0;       // Starting angle (robot's orientation when it starts)
extern bool homeReached;     // Flag to check if the home base is reached
uint16_t Middle = averagedTOFreadings[0];
uint16_t Left = averagedTOFreadings[1];
uint16_t Right = averagedTOFreadings[2];

void return_home(void) {
  // Update TOF sensor readings
  Middle = averagedTOFreadings[0];
  Left = averagedTOFreadings[1];
  Right = averagedTOFreadings[2];

  if (!homeReached) {
    // Homing logic when wall is not found
    if (FoundWall == 0) {
      if (Right > 200 && Left > 200) { // If no walls are detected
        int robot_orientation = ori[0]; // Use IMU orientation data (ori[0])

        int homing_angle = 360 - robot_orientation; // Homing angle calculation
        if (homing_angle < 0) homing_angle += 360;  // Keep homing angle within bounds
        else if (homing_angle >= 360) homing_angle -= 360;

        // Adjust robot heading towards home
        if (homing_angle > 10) {
          full_turn_left(motortime);  // Turn left if homing angle is large
        } else if (homing_angle < -10) {
          full_turn_right(motortime); // Turn right if homing angle is large
        } else {
          full_forward(motortime);    // Move forward if close to the home angle
        }
      } else if (Right < 200 && Middle < 500) { // Detected wall ahead
        full_turn_left(motortime);  // Turn left when close to a right wall
      } else if (Left < 200 && Middle < 500) { // Detected wall ahead
        full_turn_right(motortime); // Turn right when close to a left wall
      }

      // If a wall is found, update FoundWall state
      if ((Right < 300) && (Middle > 500)) {
        FoundWall = 1;  // Detected right wall
      } else if ((Left < 300) && (Middle > 500)) {
        FoundWall = 2;  // Detected left wall
      }

    } else if (FoundWall == 1 || FoundWall == 2) {
      // Wall-following logic after a wall is detected
      full_forward(motortime);

      if (FoundWall == 1) { // Following the right wall
        if (Right > 30) {
          full_turn_right(motortime); // Keep a safe distance from the right wall
        } else if (Right < 5) {
          full_turn_left(motortime);  // Turn away if too close to the wall
        }
      } else if (FoundWall == 2) { // Following the left wall
        if (Left > 30) {
          full_turn_left(motortime); // Keep a safe distance from the left wall
        } else if (Left < 5) {
          full_turn_right(motortime); // Turn away if too close to the wall
        }
      }
    }

    // Check if the robot has reached home based on color detection
    if (ColorCompareHome()) {
      homeReached = true; // Set flag when the robot reaches home
    }
  }
}
