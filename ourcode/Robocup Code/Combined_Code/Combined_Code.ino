/********************************************************************************
 *                               ROBOCUP TEMPLATE                              
 *        
 *  
 *  This is a template program design with modules for 
 *  different components of the robot, and a task scheduler
 *  for controlling how frequently tasks sholud run
 *  
 *  
 *  written by: Logan Chatfield, Ben Fortune, Lachlan McKenzie, Jake Campbell
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

bool runProgram = false;
extern bool ReadyToDrive;
extern bool WeightDetected;
extern int NumWeightsCollected;
extern bool TimeToGo;
extern bool homeReached;
extern bool collect_weight;
extern volatile bool finished_collecting;

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
#define READ_ELECTROMAGNET_TASK_PERIOD      50     // Takes 0 ms
#define READ_INDUCTIVE_TASK_PERIOD          50     // Takes 0 ms
#define IMU_UPDATE_TASK_PERIOD              5       //Takes 2 ms
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
#define RETURN_HOME_TASK_PERIOD             1000

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



Scheduler taskManager;

//**********************************************************************************
// Function Definitions
//**********************************************************************************

void pin_init();
void task_init();
void stopAllActions();

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  setupActuators();
  setup_IMU();
  setupSensors();
  int_init();
  task_init();
  Wire.begin();
  Serial.println("System Initialised");
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


void toggle_prog_ISR() {
  runProgram = !runProgram;
}

void int_init() {
  attachInterrupt(digitalPinToInterrupt(STARTBUT), toggle_prog_ISR, FALLING);
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

  Serial.println("Tasks have been initialised \n");
}



//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
  Serial.print(finished_collecting);
  if (runProgram) {
    int currentTime = millis() / 1000;
    if (currentTime > 100) {
      TimeToGo = true;
    }

    switch (currentState) {
      case STARTING:
        // Perform initial navigation actions
        tReturn_home.disable();
        tCollect_weight_1.disable();
        tCollect_weight_2.disable();
        tCollect_weight_3.disable();
        
        tNavigation.enable();
        

        Serial.println("Starting Navigation...");
        ReadyToDrive = true;
        if (ReadyToDrive) {
          currentState = DRIVING;
        }
        break;

      case DRIVING:
        tNavigation.enable();
        tRead_TOF.enable();
        tUpdate_wall_state.enable();
        tUpdate_weight_state.enable();
        tCheck_sensor_updates.enable();

        // Call navigation-related actions
        if (TimeToGo) {
          currentState = RETURNING_HOME;
        }
        if (collect_weight) {
          currentState = COLLECTING_WEIGHT;
        }
        break;

      case COLLECTING_WEIGHT:
        tNavigation.disable();
        tUpdate_wall_state.disable();
        tUpdate_weight_state.disable();

        // Enable the appropriate collection task based on the collectionState
        switch (collectionState) {
          case ZERO:
            tCollect_weight_1.enable();
            break;
          case ONE:
            tCollect_weight_2.enable();
            break;
          case TWO:
            tCollect_weight_3.enable();
            break;
          case THREE:
            // All weights collected, prepare to return home
            currentState = RETURNING_HOME; 
            break;
        }

        // Check if the collection is finished for the current weight
        if (finished_collecting) {
          // Disable the current collection task
          switch (collectionState) {
            case ZERO:
              tCollect_weight_1.disable();
              break;
            case ONE:
              tCollect_weight_2.disable();
              break;
            case TWO:
              tCollect_weight_3.disable();
              break;
            default:
              break;
          }

          // Move to the next collection state
          if (collectionState < THREE) {
            collectionState = static_cast<WeightsCollectedState>(collectionState + 1);
          }

          // Reset finished_collecting for the next weight
          finished_collecting = false;
        }
        break;

      case RETURNING_HOME:
        tNavigation.disable();
        tUpdate_wall_state.disable();
        tUpdate_weight_state.disable();
        tReturn_home.enable();

        if (homeReached) {
          currentState = FINISHED;
        }
        break;

      case FINISHED:
        stopAllActions();
        break;
    }
    taskManager.execute();
  } else {
    // Disable tasks when the program is not running
    stop(10);
    NumWeightsCollected = 0;
    tNavigation.disable();
    tCheck_orientation.disable();
    tCheck_sensor_updates.disable();
  }
}