
#include <Zumo32U4.h>
#include <robot_control.h>
#include <TurnSensor.h>

Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
L3G gyro;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Encoders encoders;

int end_pos[2] = {2,2};
int start_pos[2] = {0,0};
int current_pos[2] = {0,0};
uint16_t objective = 200;
int lastDisplayTime=0;


//int values = {0};


void setup() {

  turnSensorSetup(); // intialize Gyro
  delay(500);

  // after being placed fetch best estimate of location (uses kevins IR function)
  //runs once

  // determine array of cells to visit on our way to finish
  line_sense_init();
  motors.setSpeeds(0,0);
  delay(2000);

  turnSensorReset(); // Reset Orientation to zero
 

  Serial.begin(9600);
  

  align_frames(start_pos);
}

void loop() {
  
  int cells_to_visit[20][2];
  int (*cells_to_visit_add)[20][2] = &cells_to_visit;
  getCellsToVist(cells_to_visit_add, start_pos, end_pos);


  char dir;
 // int cells_to_visit[20][2] = {{1,0},{2,0},{2,1},{2,2}, {2,3}, {2,4}};


  // forward

  int i = 0;
  while (!(cells_to_visit[i][0]==end_pos[0] && cells_to_visit[i][1]==end_pos[1]))
  {

    int x_move = cells_to_visit[i][0] - current_pos[0];
    int y_move = cells_to_visit[i][1] - current_pos[1];

    // Figure out whether we need to turn
    int theta_desired;
    switch (x_move){
    case -1:
      theta_desired = 180;
      dir = 'L';
      break;
    case 1:
      theta_desired = 0;
      dir = 'R';
      break;
    case 0:
    //Serial.println("case 0");
      
      switch(y_move){
        case -1:
        theta_desired = 270;
        dir = 'R';
        break;
        case 1:
        theta_desired = +90;
        dir ='L';

        break;
        case 0:
        theta_desired = 69;
        break;
      }
      break;
    }
      
    turn(theta_desired, dir, 1);
    delay(1000);
    forward(objective, theta_desired);
    delay(1000);
      
    current_pos[0] = cells_to_visit[i][0];
    current_pos[1] = cells_to_visit[i][1];
    i++;
  }

  find_dot();


  while (1){}
}

