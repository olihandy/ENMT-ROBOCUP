#include "Sensors.h"
#include "Actuators.h"

enum WallDetectionState {
    NO_WALL,            // Default state when no wall is detected
    WALL_AHEAD,         // Wall detected ahead
    SLIT_DETECTED,      // Slit detected
    LEFT_WALL_DETECTED, // Wall detected on the left
    SLAB_WALL_DETECTED, // Slab wall detected
    RIGHT_WALL_DETECTED // Wall detected on the right
};
// Declare the current state
WallDetectionState wallState = NO_WALL;

enum RobotState {
    STARTING,          // 0 = Starting
    DRIVING,           // 1 = Driving
    COLLECTING_WEIGHT, // 2 = Collecting Weight
    RETURNING_HOME     // 3 = Returning Home
};
// Declare the initial state
RobotState currentState = STARTING;



void Navigation(void) {
  GetTOF(TOFreadings);
  GetElectroMagnet(electromagnetStates);
  GetInduction(inductionSensorStates);


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



  switch (currentState) {
      case STARTING:
          //Take start readings
          if (readyToDrive) {
              currentState = DRIVING;
          }
          break;

      case DRIVING:
          // Logic for the Driving state
          // Example condition: if the robot detects a weight to collect
          if (weightDetected) {
              currentState = COLLECTING_WEIGHT;
          }
          break;

      case COLLECTING_WEIGHT:
          // Logic for the Collecting Weight state
          // Example condition: if the weight has been collected
          if (ThreeWeightsCollected || TimeToGo) {
              currentState = RETURNING_HOME;
          }
          break;

      case RETURNING_HOME:
          // Logic for the Returning Home state
          // Example condition: if the robot has returned home
          if (homeReached) {
              currentState = STARTING;  // Reset to starting state, or stop
          }
          break;
  }


