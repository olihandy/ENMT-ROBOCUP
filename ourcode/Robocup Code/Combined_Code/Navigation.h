#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Sensors.h" 
#include "Actuators.h"

// Boolean flags for robot state
extern bool ReadyToDrive;
extern bool WeightDetected;
extern bool ThreeWeightsCollected;
extern bool TimeToGo;
extern bool homeReached;

// Program state and motor timing
extern int motortime;

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
  RETURNING_HOME     // 3 = Returning Home
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
extern void Navigation(void);
extern void PrintStates(void);
extern void UpdateWeightState(void);
extern void UpdateWallState(int TopLeft, int TopMiddle, int TopRight);

#endif //NAVIGATION_H
