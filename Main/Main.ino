#include <Zumo32U4.h>
//#include <Estimation.h>
#include <robot_control.h>
#include <TurnSensor.h>
Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
L3G gyro;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

//int goal[2] = {3,4};

int cell_counter = 0;
//int next_cell[2] = 0;

double* x_hat = 0;
double* y_hat = 0;
double* theta_hat =0;

int intial[2] = {3,4}; //initial cell location

void setup() {

  turnSensorSetup(); // intialize Gyro
  delay(500);
  turnSensorReset(); // Reset Orientation to zero

  // after being placed fetch best estimate of location (uses kevins IR function)
  //runs once

  // determine array of cells to visit on our way to finish
  
  ir_init();
}

void loop() {


  //update next_cell
  

  //initiate error from next_cell's center
  

  // fetch applicable sensor data and weight as necessary for measurement
  
  //

  cell_counter += 1;
}
