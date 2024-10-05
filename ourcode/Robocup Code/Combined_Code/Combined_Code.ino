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


#include "Actuators.h"
#include "IMU.h"
#include "Navigation.h"
#include "Sensors.h"
#include "SensorBuffering.h"


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
#define PRINT_IMU_TASK_PERIOD               200       //Takes 0ms?
#define PRINT_INFORMATION_TASK_PERIOD       200     //Takes 130 ms
#define PRINT_STATES_TASK_PERIOD            200     //Takes 0 ms? 

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


// Pin deffinitions
#define IO_POWER  49

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
Task tUpdate_wall_state(WALL_UPDATE_TASK_PERIOD,                  WALL_UPDATE_TASK_NUM_EXECUTE,             &UpdateWallState); // Added missing period and execution parameters
Task tUpdate_weight_state(WEIGHT_UPDATE_TASK_PERIOD,              WEIGHT_UPDATE_TASK_NUM_EXECUTE,           &UpdateWeightState); // Added missing period and execution parameters
Task tCheck_orientation(CHECK_ORIENTATION_TASK_PERIOD,            CHECK_ORIENTATION_TASK_NUM_EXECUTE,       &checkOrientation);
Task tCheck_sensor_updates(CHECK_SENSOR_UPDATES_TASK_PERIOD,      CHECK_SENSOR_UPDATES_TASK_NUM_EXECUTE,    &CheckForSensorUpdates);
Task tNavigation(NAVIGATION_TASK_PERIOD,                          NAVIGATION_TASK_NUM_EXECUTE,              &Navigation);
Task tIMU_print(PRINT_IMU_TASK_PERIOD,                            PRINT_IMU_NUM_EXECUTE,                    &IMU_print);
Task tPrint_information(PRINT_INFORMATION_TASK_PERIOD,            PRINT_INFORMATION_TASK_NUM_EXECUTE,       &PrintInformation);
Task tPrint_states(PRINT_STATES_TASK_PERIOD,                      PRINT_STATES_TASK_NUM_EXECUTE,            &PrintStates);


Scheduler taskManager;

//**********************************************************************************
// Function Definitions
//**********************************************************************************

void pin_init();
void robot_init();
void task_init();

//**********************************************************************************
// put your setup code here, to run once:
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  setupActuators();
  setup_IMU();
  setupSensors();
  pin_init();
  task_init();
  Wire.begin();
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work) 
// Set as high or low
//**********************************************************************************
void pin_init(){
    
    Serial.println("Pins have been initialised \n"); 

    pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
    digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board
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
  // taskManager.addTask(tPrint_information);
  // taskManager.addTask(tPrint_states);

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

  Serial.println("Tasks have been initialised \n");
}



//**********************************************************************************
// put your main code here, to run repeatedly
//**********************************************************************************
void loop() {
  // unsigned long start_time = millis();
  // Navigation();
  // unsigned long end_time = millis();
  // Serial.println(end_time-start_time);

  taskManager.execute();    //execute the scheduler
}
