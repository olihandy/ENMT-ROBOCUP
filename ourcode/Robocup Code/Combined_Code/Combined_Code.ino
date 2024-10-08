/********************************************************************************
 *                               ROBOCUP TEMPLATE                              
 *        
 *  
 *  This is a template program design with modules for 
 *  different components of the robot, and a task scheduler
 *  for controlling how frequently tasks sholud run
 *  
 *  
 *  
 ******************************************************************************/

#include <Servo.h>                  //control the DC motors
#include <Adafruit_TCS34725.h>      //colour sensor
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler 
#include <stdio.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "SensorBuffering.h"

#include "Actuators.h"
#include "IMU.h"
#include "Navigation.h"
#include "Sensors.h"
#include "SensorBuffering.h"
#include "WeightCollection.h"
#include "ReturningHome.h"
#include "DropWeights.h"

extern bool actionInProgress;
unsigned long start_time;
unsigned long currentTime =0;
extern int motortime;
bool runProgram = true;
extern bool ReadyToDrive;
extern bool WeightDetected;
extern int NumWeightsCollected;
extern bool TimeToGo;
extern float GoTime;
extern bool homeReached;
extern bool collect_weight;
extern bool electromagnetStates[3];         // State of electromagnets

// Robot state enumeration
enum RobotState {
  STARTING,          // 0 = Starting
  DRIVING,           // 1 = Driving
  COLLECTING_WEIGHT, // 2 = Collecting Weight
  RETURNING_HOME,    // 3 = Returning Home
  FINISHED           // 4 = Finished
};
// Declare the current robot state
RobotState currentState = STARTING;

enum WeightsCollectedState {
  ZERO,
  ONE,
  TWO,
  THREE
};
WeightsCollectedState collectionState = ZERO;



//**********************************************************************************
// Local Definitions
//**********************************************************************************




// Task period Definitions
// ALL OF THESE VALUES WILL NEED TO BE SET TO SOMETHING USEFUL !!!!!!!!!!!!!!!!!!!!
#define TOF_READ_TASK_PERIOD                70     //Takes 45 ms, 70 ms max
#define READ_ELECTROMAGNET_TASK_PERIOD      2     // Takes 0 ms
#define READ_INDUCTIVE_TASK_PERIOD          50     // Takes 0 ms
#define IMU_UPDATE_TASK_PERIOD              50       //Takes 2 ms
#define WALL_UPDATE_TASK_PERIOD             50     //Takes 0ms
#define WEIGHT_UPDATE_TASK_PERIOD           50     //Takes 0ms?
#define CHECK_ORIENTATION_TASK_PERIOD       50     //Takes 0ms?
#define CHECK_SENSOR_UPDATES_TASK_PERIOD    50     //Takes 0ms?
#define NAVIGATION_TASK_PERIOD              50    //Takes 100 ms?
#define PRINT_IMU_TASK_PERIOD               500       //Takes 0ms?
#define PRINT_INFORMATION_TASK_PERIOD       500     //Takes 130 ms
#define PRINT_STATES_TASK_PERIOD            500     //Takes 0 ms? 
#define COLLECT_WEIGHT_1_TASK_PERIOD        500
#define COLLECT_WEIGHT_2_TASK_PERIOD        500
#define COLLECT_WEIGHT_3_TASK_PERIOD        500
#define RETURN_HOME_TASK_PERIOD             500
#define COLOUR_COMPARE_TASK_PERIOD          50
#define COLOUR_START_TASK_PERIOD            50
#define DROP_WEIGHTS_TASK_PERIOD            1000

// Task execution amount definitions
// -1 means indefinitely
#define TOF_READ_TASK_NUM_EXECUTE              -1
#define READ_ELECTROMAGNET_NUM_EXECUTE         -1
#define READ_INDUCTIVE_NUM_EXECUTE             -1
#define IMU_UPDATE_TASK_NUM_EXECUTE            -1
#define WALL_UPDATE_TASK_NUM_EXECUTE           -1
#define WEIGHT_UPDATE_TASK_NUM_EXECUTE         -1
#define CHECK_ORIENTATION_TASK_NUM_EXECUTE     -1
#define CHECK_SENSOR_UPDATES_TASK_NUM_EXECUTE  -1
#define NAVIGATION_TASK_NUM_EXECUTE            -1
#define PRINT_IMU_NUM_EXECUTE                  -1
#define PRINT_INFORMATION_TASK_NUM_EXECUTE     -1
#define PRINT_STATES_TASK_NUM_EXECUTE          -1
#define COLLECT_WEIGHT_1_NUM_EXECUTE           -1
#define COLLECT_WEIGHT_2_NUM_EXECUTE           -1
#define COLLECT_WEIGHT_3_NUM_EXECUTE           -1
#define RETURN_HOME_NUM_EXECUTE                -1
#define COLOUR_COMPARE_NUM_EXECTUTE            -1
#define COLOUR_START_NUM_EXECUTE               -1
#define DROP_WEIGHTS_NUM_EXECUTE               -1



// Pin deffinitions
#define IO_POWER  49
#define STARTBUT 23

// Serial deffinitions
#define BAUD_RATE 9600

Servo right_motor;
Servo left_motor;


//**********************************************************************************
// Task Scheduler and Tasks
//**********************************************************************************

/* The first value is the period, second is how many times it executes
   (-1 means indefinitely), third one is the callback function */

// Tasks for reading sensors and navigation
Task tRead_TOF(TOF_READ_TASK_PERIOD,                              TOF_READ_TASK_NUM_EXECUTE,                &GetTOF);
Task tRead_electromagnet(READ_ELECTROMAGNET_TASK_PERIOD,          READ_ELECTROMAGNET_NUM_EXECUTE,           &GetElectroMagnet);
Task tRead_inductive(READ_INDUCTIVE_TASK_PERIOD,                  READ_INDUCTIVE_NUM_EXECUTE,               &GetInduction);
Task tIMU_update(IMU_UPDATE_TASK_PERIOD,                          IMU_UPDATE_TASK_NUM_EXECUTE,              &IMU);
Task tUpdate_wall_state(WALL_UPDATE_TASK_PERIOD,                  WALL_UPDATE_TASK_NUM_EXECUTE,             &UpdateWallState);
Task tUpdate_weight_state(WEIGHT_UPDATE_TASK_PERIOD,              WEIGHT_UPDATE_TASK_NUM_EXECUTE,           &UpdateWeightState);
Task tCheck_orientation(CHECK_ORIENTATION_TASK_PERIOD,            CHECK_ORIENTATION_TASK_NUM_EXECUTE,       &checkOrientation);
Task tCheck_sensor_updates(CHECK_SENSOR_UPDATES_TASK_PERIOD,      CHECK_SENSOR_UPDATES_TASK_NUM_EXECUTE,    &CheckForSensorUpdates);
Task tNavigation(NAVIGATION_TASK_PERIOD,                          NAVIGATION_TASK_NUM_EXECUTE,              &Navigation);
Task tIMU_print(PRINT_IMU_TASK_PERIOD,                            PRINT_IMU_NUM_EXECUTE,                    &IMU_print);
Task tPrint_information(PRINT_INFORMATION_TASK_PERIOD,            PRINT_INFORMATION_TASK_NUM_EXECUTE,       &PrintInformation);
Task tPrint_states(PRINT_STATES_TASK_PERIOD,                      PRINT_STATES_TASK_NUM_EXECUTE,            &PrintStates);
Task tCollect_weight_1(COLLECT_WEIGHT_1_TASK_PERIOD,              COLLECT_WEIGHT_1_NUM_EXECUTE,             &CollectWeight_1);
Task tCollect_weight_2(COLLECT_WEIGHT_2_TASK_PERIOD,              COLLECT_WEIGHT_2_NUM_EXECUTE,             &CollectWeight_2);
Task tCollect_weight_3(COLLECT_WEIGHT_3_TASK_PERIOD,              COLLECT_WEIGHT_3_NUM_EXECUTE,             &CollectWeight_3);
Task tReturn_home(RETURN_HOME_TASK_PERIOD,                        RETURN_HOME_NUM_EXECUTE,                  &return_home);
Task tColour_compare(COLOUR_COMPARE_TASK_PERIOD,                  COLOUR_COMPARE_NUM_EXECTUTE,              &ColorCompareHome);
Task tColour_start(COLOUR_START_TASK_PERIOD,                      COLOUR_START_NUM_EXECUTE,                 &colorStart);
Task tDrop_Weights(DROP_WEIGHTS_TASK_PERIOD,                      DROP_WEIGHTS_NUM_EXECUTE,                 &drop_weights);


Scheduler taskManager;

//**********************************************************************************
// Function Definitions
//**********************************************************************************

void pin_init();
void task_init();
void stopAllActions();
void startingActions();
void competition_init();
void drivingActions();
void returningHomeActions();

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  setupActuators();
  setupSensors();
  setup_IMU();
  Serial.println("System Initialised");
  while(!digitalRead(STARTBUT)) {
    start_time = millis() / 1000;
    delay(1);
  }
  task_init();
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work) 
// Set as high or low
//**********************************************************************************
void pin_init(){
    pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
    digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
    pinMode(STARTBUT, INPUT_PULLUP);
    Serial.println("Pins have been initialised \n"); 

}


void stopAllActions() {
  tRead_TOF.disable();
  tRead_electromagnet.disable();
  tRead_inductive.disable();
  tIMU_update.disable();
  tUpdate_wall_state.disable();
  tUpdate_weight_state.disable();
  tCheck_orientation.disable();
  tCheck_sensor_updates.disable();
  tNavigation.disable();
  tIMU_print.disable();
  tPrint_information.disable();
  tPrint_states.disable();
  tCollect_weight_1.disable();
  tCollect_weight_2.disable();
  tCollect_weight_3.disable();
  tReturn_home.disable();
  tColour_compare.disable();  
  tColour_start.disable();
  tDrop_Weights.disable();
}

void startingActions() {
  tRead_TOF.enable();
  tRead_electromagnet.enable();
  tRead_inductive.enable();
  tIMU_update.enable();
  tUpdate_wall_state.enable();
  tUpdate_weight_state.enable();
  tCheck_orientation.enable();
  tCheck_sensor_updates.enable();
  tNavigation.disable();
  tIMU_print.enable();
  tPrint_information.enable();
  tPrint_states.enable();
  tCollect_weight_1.disable();
  tCollect_weight_2.disable();
  tCollect_weight_3.disable();
  tReturn_home.disable();
  tColour_compare.enable();
  tColour_start.enable();
  tDrop_Weights.disable();
  

}

void drivingActions() {
  tNavigation.enable();
  tColour_start.disable();
}


void returningHomeActions() {
  stop_blocking(10);
  tNavigation.disable();
  tUpdate_wall_state.disable();
  tUpdate_weight_state.disable();
  tReturn_home.enable();
  tColour_compare.enable();
  tColour_start.disable();
  tIMU_update.enable();
  tIMU_print.enable();
  tPrint_information.disable();
  tRead_TOF.enable();
  tCheck_orientation.disable();
  tDrop_Weights.disable();
  tPrint_states.disable();
}
//**********************************************************************************
// Initialise the tasks for the scheduler
//**********************************************************************************
void task_init() {  
  // This is a class/library function. Initialise the task scheduler
  taskManager.init();     
 
  // Add tasks to the scheduler
  taskManager.addTask(tRead_TOF);   
  taskManager.addTask(tRead_electromagnet);   
  taskManager.addTask(tRead_inductive);   
  taskManager.addTask(tIMU_update);
  taskManager.addTask(tUpdate_wall_state);
  taskManager.addTask(tUpdate_weight_state);
  taskManager.addTask(tCheck_orientation);
  taskManager.addTask(tCheck_sensor_updates);
  taskManager.addTask(tNavigation); 
  taskManager.addTask(tIMU_print);
  taskManager.addTask(tPrint_information);
  taskManager.addTask(tPrint_states);
  taskManager.addTask(tCollect_weight_1);
  taskManager.addTask(tCollect_weight_2);
  taskManager.addTask(tCollect_weight_3);
  taskManager.addTask(tReturn_home);
  taskManager.addTask(tColour_compare);
  taskManager.addTask(tColour_start);
  taskManager.addTask(tDrop_Weights);


  // Enable the tasks
  tRead_TOF.enable();
  tRead_electromagnet.enable();
  tRead_inductive.enable();
  tIMU_update.enable();
  tUpdate_wall_state.enable();
  tUpdate_weight_state.enable();
  tCheck_orientation.enable();
  tCheck_sensor_updates.enable();
  tNavigation.enable();
  tIMU_print.enable();
  tPrint_information.enable();
  tPrint_states.enable();
  tCollect_weight_1.enable();
  tCollect_weight_2.enable();
  tCollect_weight_3.enable();
  tReturn_home.enable();
  tColour_compare.enable();
  tColour_start.enable();
  tDrop_Weights.enable();


  Serial.println("Tasks have been initialised \n");
}


void competition_init() {
  tPrint_information.disable();
  tPrint_states.disable();
  }


//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
  
  static unsigned long collectionStartTime = 0;
  const unsigned long collectionDelay = 100;
  static bool firstCollection = true;

  int currentTime = millis() / 1000 - start_time ;

  // int start_time2 = millis();
  // GetInduction();
  // int end_time = millis();
  // Serial.println(end_time-start_time2);

  if(actionInProgress) {
    tNavigation.disable();
  } else {
    tNavigation.enable();
  }

  if (runProgram) {
    // Serial.println(currentTime);
    if (currentTime > 100) {
      TimeToGo = true;
    }

    switch (currentState) {
      case STARTING:
        startingActions();
        ReadyToDrive = true;
        if (ReadyToDrive) {
          currentState = DRIVING;
        }
        break;

      case DRIVING:
      drivingActions();
        if (TimeToGo || NumWeightsCollected == 3) {
          currentState = RETURNING_HOME;
        }
        if (collect_weight) {
          currentState = COLLECTING_WEIGHT;
        }
        break;

      case COLLECTING_WEIGHT:

        // Check if enough time has passed since the last collection or for the first collection
        if (firstCollection || (millis() - collectionStartTime >= collectionDelay)) {
          firstCollection = false;  // Reset the first collection flag
          switch (collectionState) {
            case ZERO:
              if(NumWeightsCollected == 0) {
                tCollect_weight_1.enable(); // Enable task to collect the second weight
              } else {
                tCollect_weight_1.disable();
                break;
              }
              break;
            case ONE:
              if(NumWeightsCollected == 1) {
                tCollect_weight_2.enable(); // Enable task to collect the second weight
              } else {
                tCollect_weight_2.disable();
                break;
              }
              break;
            case TWO:
              if(NumWeightsCollected == 2) {
                tCollect_weight_3.enable(); // Enable task to collect the second weight
              } else {
                tCollect_weight_3.disable();
                currentState = RETURNING_HOME; // Move to returning home after collecting all weight
                break;
              }
              break;
            case THREE:
              currentState = RETURNING_HOME; // Move to returning home after collecting all weights
              break;
          }

          // After each weight collection, check if a weight was collected
          if (NumWeightsCollected > static_cast<int>(collectionState)) {
            if (collectionState < THREE) {
              collectionState = static_cast<WeightsCollectedState>(collectionState + 1);
              collectionStartTime = millis(); // Reset the timer after collecting a weight
            }

            // Reset flags and return to driving state after weight is collected
            collect_weight = false;
            currentState = DRIVING;
          }
        }
        break;

      case RETURNING_HOME:
        returningHomeActions();
        if (homeReached) {
          full_forward_blocking(5*motortime);
          stop_blocking(motortime);
          tDrop_Weights.enable();
          currentState = FINISHED;
        }
        break;

      case FINISHED:
      if(currentTime < 110) {
        tDrop_Weights.disable();
        NumWeightsCollected = 0;
        full_reverse_blocking(15*motortime);
        full_turn_left_blocking(15*motortime);
        firstCollection = true;
        collectionStartTime = 0;
        currentState = DRIVING;
        break;
      } else {
        stopAllActions();
        break;
      }
    }
    taskManager.execute();
  } else {
    stop(10);
    NumWeightsCollected = 0;
    currentState = STARTING;
    stopAllActions();
  }
}