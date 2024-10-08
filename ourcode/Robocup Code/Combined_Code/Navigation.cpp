#include "Navigation.h"
#include "Sensors.h"
#include "WeightCollection.h"

//--------------------------------------------------------------------------------------------------------//
//--------------------------------------- Shared Variables ------------------------------------------------//
//--------------------------------------------------------------------------------------------------------//


int reverseCount = 0;                  // Counter for reversals
const int reverseThreshold = 5;        // Number of times robot can reverse before it turns around
unsigned long lastReverseTime = 0;     // Time when the last reverse happened
const unsigned long reverseTimeout = 10000; // 10 seconds timeout to reset the reverse count

extern unsigned long elapsed_time;
bool ReadyToDrive = false;
bool WeightDetected = false;
bool TimeToGo = false;
bool homeReached = false;
bool collect_weight = false;
extern int NumWeightsCollected;

extern bool finished_collecting;
int LengthOfRobot = 350;
int five_seconds = 5000;
int ten_seconds = 10000;
int NOCHANGETHRESHOLD = 5;
int timeWeightDetected;
unsigned long lastOrientationChangeTime = 0;
float lastOrientation = 0;

extern int stepper_motor_fast;
extern int stepper_motor_slow;
extern int motortime;

WallDetectionState wallState = NO_WALL;
WeightDetectionState weightState = WEIGHT_NOT_DETECTED;

int numTOFs =7;


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

extern float ori[2];

float pitch = ori[1];
float yaw = ori[0];

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
  // Update last change time if any sensor changes are detected
  if (sensorChanged) {
    lastChangeTime = elapsed_time;
  }
  // If no sensor changes detected for the timeout duration, perform reverse navigation
  if ((elapsed_time - lastChangeTime) > timeoutDuration) {
    Serial.println("No sensor change detected for timeout duration. Executing reverse navigation.");
    full_reverse_blocking(10 * motortime);    // Perform reverse
    full_turn_left_blocking(100 * motortime); // Perform turn
  }
}



void checkOrientation(void) {
  yaw = ori[0];
  pitch = ori[1];

  if(abs(pitch) > 5) {
    full_reverse_blocking(5*motortime);
    Serial.print("REVERSING");
  }

  // Check if the orientation has changed significantly (beyond a small tolerance)
  if (abs(yaw - lastOrientation) > 10.0) {
    // Reset the timer if orientation has changed
    lastOrientationChangeTime = millis();
    lastOrientation = yaw;
  }

  // If the robot has been facing the same direction for too long
  if ((millis() - lastOrientationChangeTime) > ten_seconds) {
    // Time to turn around
    full_turn_left_blocking(100*motortime);
    Serial.println("No direction change detected, turning");
    
    // After turning, reset the timer and the orientation
    lastOrientationChangeTime = millis();
    lastOrientation = ori[0]; // getOrientation() retrieves the updated orientation after turn
  }
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
  // if((millis() - timeWeightDetected) > timeoutDuration) {
  //   weightState = WEIGHT_NOT_DETECTED;
  // }
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


  // Check if enough time has passed since the last reverse to reset the counter
  if (elapsed_time - lastReverseTime > reverseTimeout) {
    reverseCount = 0;  // Reset reverse count after timeout
  }

  switch (weightState) {
    case WEIGHT_NOT_DETECTED:
      collect_weight = false;
      switch (wallState) {
        case WALL_AHEAD:
          full_reverse(motortime);
          reverseCount++;  // Increment reverse counter
          lastReverseTime = elapsed_time;  // Update the last reverse time

          // If the robot has reversed too many times, perform a 180-degree turn
          if (reverseCount >= reverseThreshold) {
            full_turn_right_blocking(15*motortime);  // 180-degree turn
            reverseCount = 0;  // Reset reverse counter
            Serial.println("Stuck in a loop. Performing 180-degree turn.");
          }
          break;

        case SLIT_DETECTED:
          full_forward(motortime);
          break;

        case LEFT_WALL_DETECTED:
          if (TopLeft < 100)
          {
            reverse_right_blocking(5*motortime);
            full_turn_right_blocking(5* motortime);
          } else {
            forward_right(motortime);
          }

        case SLAB_WALL_DETECTED:
          full_reverse_blocking(10 * motortime);
          reverseCount++;
          lastReverseTime = elapsed_time;
          if (reverseCount >= reverseThreshold) {
            full_turn_right_blocking(200 * motortime);
            reverseCount = 0;
            Serial.println("Stuck in a loop. Performing 180-degree turn.");
          }
          if (TopLeft > TopRight) {
            full_turn_left_blocking(5 * motortime);
          } else {
            full_turn_right_blocking(5 * motortime);
          }
          break;

        case RIGHT_WALL_DETECTED:
          if (TopRight < 100)
          {
            reverse_left_blocking(5*motortime);
            full_turn_left_blocking(5* motortime);
          } else {
            forward_left(motortime);
          }
          

          break;

        case NO_WALL:
        default:
          full_forward(motortime);
          break;
      }
      break;

    case WEIGHT_DETECTED:
      if (TopMiddle < 200) {
        full_reverse(5 * motortime);
        reverseCount++;
        lastReverseTime = elapsed_time;
        if (reverseCount >= reverseThreshold) {
          full_turn_right_blocking(200 * motortime);
          reverseCount = 0;
          Serial.println("Stuck in a loop. Performing 180-degree turn.");
        }

        if (TopLeft > TopRight) {
          full_turn_left(5 * motortime);
        } else {
          full_turn_right(5 * motortime);
        }
      } else if (TopLeft < 100) {
        full_reverse_blocking(5 * motortime);
        full_turn_left_blocking(3 * motortime);
      } else if (TopRight < 100) {
        full_reverse_blocking(5 * motortime);
        full_turn_right_blocking(3 * motortime);
      } else {
        if (BottomLeft < 400 || BottomRight < 400) {
          if (BottomLeft > (BottomRight + 50)) {
            full_turn_right_blocking(5 * motortime);
          } else if (BottomRight > (BottomLeft + 50)) {
            full_turn_left_blocking(5 * motortime);
          } else {
            half_forward(motortime);
          }
        } else {
          if (BottomLeft > (BottomRight + 50)) {
            forward_right(3 * motortime);
          } else if (BottomRight > (BottomLeft + 50)) {
            forward_left(3 * motortime);
          } else {
            half_forward(motortime);
          }
        }
      }
      break;

    case WEIGHT_CONFIRMED:
      if (TopMiddle < 300) {
        if (TopLeft > TopRight) {
          full_turn_left_blocking(5 * motortime);
        } else {
          full_turn_right_blocking(5 * motortime);
        }
      } else {
        collect_weight = true;
        weightState = WEIGHT_NOT_DETECTED;
      }
      break;
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

  Serial.print("WeightState: ");
  switch (weightState) {
    case WEIGHT_NOT_DETECTED: Serial.println("WEIGHT_NOT_DETECTED"); break;
    case WEIGHT_DETECTED: Serial.println("WEIGHT_DETECTED"); break;
    case WEIGHT_CONFIRMED: Serial.println("WEIGHT_CONFIRMED"); break;
  }

  Serial.println("----------------------");
}