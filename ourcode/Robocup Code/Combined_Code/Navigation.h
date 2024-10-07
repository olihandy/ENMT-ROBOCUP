#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Sensors.h" 
#include "Actuators.h"
#include "IMU.h"


extern bool ReadyToDrive;
extern bool WeightDetected;
extern int NumWeightsCollected;
extern bool TimeToGo;
extern bool homeReached;
extern bool collect_weight;

extern float ori[2]; // Declare it as extern so it can be accessed in other files

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
extern void UpdateWallState(void);
extern void UpdateWeightState(void);
extern void UpdateAll(void);
extern void Navigation(void);
extern void CheckForSensorUpdates(void); // Added function for checking sensor updates
extern void checkOrientation(void); // Added function for checking orientation

#endif //NAVIGATION_H
