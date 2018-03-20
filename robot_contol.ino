#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"
#include<robot_control.h>

char report[80];

double objective = 1000; // travel 1000mm
int i = 0;
int t = 0;
int dir;
float v = 0;
uint32_t maxSpeed = 200;
int32_t countsLeft = 0;
int32_t countsRight = 0;
int32_t counts = 0;
uint32_t t1 = 0;
uint32_t t2 = 0;
uint32_t d = 0;
int motorSpeed = 0;
uint32_t distance;
uint32_t d1;
uint32_t error = objective;

Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
L3G gyro;


void setup()
{
  turnSensorSetup();
  delay(500);
  turnSensorReset();
}

void loop()
{
  for (i=0; i<4; i++)
  {

    error = objective;
    dir = 0;
    countsLeft = encoders.getCountsAndResetLeft();
    countsRight = encoders.getCountsAndResetRight();
    counts = 0;
    
    turnSensorReset();

    // Move Forward while until objective has been reached
    while (!distance_reached(counts, objective, &error, &distance))
    {
      d = distance - d1;
  
      countsLeft = encoders.getCountsLeft();
      countsRight = encoders.getCountsRight();
      counts = (countsRight + countsLeft)/2;
      
      turnSensorUpdate();

      // Calculate Velocity
      if ((millis()-100)>(t1)){
        t2 = millis();
        v = vel(t1, t2, d);
        t1 = millis();
      }

      lcd.clear();
      lcd.print(error);
      lcd.gotoXY(0, 1);


      // Robot Dynamics function
      move_robot(&error, dir, vel, &maxSpeed);
      
      d1 = distance;
    }

    // Stop at objective and wait to initiate 90 deg turn
    motors.setSpeeds(0,0);
    turnSensorReset();
    dir = 1;

    delay(1000);

    // Turn 90 deg
    while(abs(turnAngle+(dir*turnAngle90)) > (2*turnAngle1))
    {
      turnSensorUpdate();
      move_robot(&error, dir, vel, &maxSpeed);
    }
    
  }
  
  maxSpeed = 100;
  motors.setSpeeds(0,0);
  while(1){}
}



