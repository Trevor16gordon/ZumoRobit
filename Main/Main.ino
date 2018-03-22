
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

//int values = {0};


void setup() {

  turnSensorSetup(); // intialize Gyro
  delay(500);
  turnSensorReset(); // Reset Orientation to zero

  // after being placed fetch best estimate of location (uses kevins IR function)
  //runs once

  // determine array of cells to visit on our way to finish
  
  ir_init();

  Serial.begin(9600);

  delay(1000);

  turn(90,'L',1);
  delay(1000);
  forward(200,90);
  

  //align_frames(start_pos);

}

void loop() {

  motors.setSpeeds(0,0);


    while (1){}
}

//void printReadingsToSerial()
//{
//  static char buffer[80];
//  sprintf(buffer, "%d %d %d\n",
//  values[0],
//  values[1],
//  values[2]
//  );
//  Serial.print(buffer);
//
//}

