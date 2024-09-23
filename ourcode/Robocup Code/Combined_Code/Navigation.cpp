#include "Navigation.h"
#include "Sensors.h"

// Define shared variables here
bool ReadyToDrive = false;
bool WeightDetected = false;
bool ThreeWeightsCollected = false;
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
            wallState = (TopMiddle < 20) ? WALL_AHEAD : SLIT_DETECTED;
        } else {
            wallState = LEFT_WALL_DETECTED;
        }
    } else if (TopMiddle < 20) {
        wallState = SLAB_WALL_DETECTED;
    } else if (TopRight < 30) {
        wallState = RIGHT_WALL_DETECTED;
    } else {
        wallState = NO_WALL; // If no walls are detected, default to NO_WALL
    }
}

void UpdateWeightState(uint16_t* TOFreadings, bool FrontInduction) {
    if (FrontInduction) {
        weightState = WEIGHT_CONFIRMED;
    } else if ((TOFreadings[4] > (TOFreadings[6] + 10)) || (TOFreadings[3] > (TOFreadings[5] + 10))) {
        weightState = WEIGHT_DETECTED;
    } else {
        weightState = WEIGHT_NOT_DETECTED;
    }
}

void Navigation(void) {
    uint16_t* TOFreading = GetTOF();
    bool* electromagnetState = GetElectroMagnet();
    bool* inductionSensorState = GetInduction();

    // Update Front and Back Induction states
    int FrontInduction = inductionSensorState[0];
    int BackInduction = inductionSensorState[1];

    // Update wall state
    UpdateWallState(TOFreading[1], TOFreading[0], TOFreading[2]); // TopLeft, TopMiddle, TopRight
    UpdateWeightState(TOFreading, FrontInduction);
    
    PrintInformation();
    PrintStates();

    switch (currentState) {
        case STARTING:
            // Take start readings, then set readyToDrive
            if (TOFreading[1] > TOFreading[2] + 20) {
                forward_left(motortime);
            } else {
                forward_right(motortime);
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
                            full_turn_right(motortime);
                            break;
                        case SLAB_WALL_DETECTED:
                            full_reverse(5*motortime);
                            if (TOFreading[1] > TOFreading[2] + 20) {
                                full_turn_left(motortime);
                            } else {
                                full_turn_right(motortime);
                            }
                            break;
                            break;
                        case RIGHT_WALL_DETECTED:
                            full_turn_left(motortime);
                            break;
                        case NO_WALL:
                        default:
                            if (TOFreading[1] > TOFreading[2] + 20) {
                                forward_left(motortime);
                            } else {
                                forward_right(motortime);
                            }
                            break;
                    }
                    break;

                case WEIGHT_DETECTED:
                    if (TOFreading[5] > TOFreading[6] + 2) {
                        forward_right(motortime);
                    } else if (TOFreading[6] > TOFreading[5] + 2) {
                        forward_left(motortime);
                    } else {
                        half_forward(motortime);
                    }
                    break;

                case WEIGHT_CONFIRMED:
                    if (BackInduction) {
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
            if (homeReached) {
                // Drop weights
            }
            break;
    }
}
