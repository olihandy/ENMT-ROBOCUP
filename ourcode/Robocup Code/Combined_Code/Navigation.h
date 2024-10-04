#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Sensors.h" 
#include "Actuators.h"
#include "IMU.h"

extern float ori[3]; // Declare it as extern so it can be accessed in other files

// Wall detection state
enum WallDetectionState {
  NO_WALL,            // Default state when no wall is detected
  WALL_AHEAD,         // Wall detected ahead
  SLIT_DETECTED,      // Slit detected
  LEFT_WALL_DETECTED, // Wall detected on the left
  SLAB_WALL_DETECTED, // Slab wall detected
  RIGHT_WALL_DETECTED // Wall detected on the right
};

// Declare the current wall detection state
extern WallDetectionState wallState;

// Robot state enumeration
enum RobotState {
  STARTING,          // 0 = Starting
  DRIVING,           // 1 = Driving
  COLLECTING_WEIGHT, // 2 = Collecting Weight
  RETURNING_HOME,    // 3 = Returning Home
  FINISHED           // 4 = Finished
};

// Declare the current robot state
extern RobotState currentState;

// Weight detection state enumeration
enum WeightDetectionState {
  WEIGHT_NOT_DETECTED, // 0 = Weight not detected
  WEIGHT_DETECTED,     // 1 = Weight detected
  WEIGHT_CONFIRMED     // 2 = Weight confirmed
};

// Declare the current weight detection state
extern WeightDetectionState weightState;


// Function prototypes
extern void PrintStates(void);
extern void UpdateWallState(uint32_t TopLeft, uint32_t TopMiddle, uint32_t TopRight);
extern void UpdateWeightState(uint32_t MiddleRight, uint32_t BottomRight, uint32_t MiddleLeft, uint32_t BottomLeft, bool FrontInduction);
extern void UpdateAll(void);
extern void Navigation(uint32_t TopMiddle, uint32_t TopLeft, uint32_t TopRight, uint32_t MiddleLeft, uint32_t MiddleRight, uint32_t BottomLeft, uint32_t BottomRight, bool BackInduction);


#endif //NAVIGATION_H
