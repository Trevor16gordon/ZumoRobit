
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


int end_pos[2] = {4,3};
int start_pos[2] = {0,0};
int current_pos[2] = {0,0};
uint16_t objective = 200;
int theta_desired;

int maxTurnSpeed = 200;
int time1 = 0;
int time2 = 0;
int time3 = 0;
double prevDistance = 0;
int lastDisplayTime = 0;
int motorSpeed = 100;
int16_t velocity;
int maxSpeed = 200;
int error = 10000;



//int values = {0};


void setup() {

  turnSensorSetup(); // intialize Gyro
  delay(500);

  // after being placed fetch best estimate of location (uses kevins IR function)
  //runs once

  // determine array of cells to visit on our way to finish

  ir_init();
  //line_sense_init();
  motors.setSpeeds(0,0);
  delay(200);

  turnSensorReset(); // Reset Orientation to zero

    while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    lcd.gotoXY(0, 0);
    lcd.print(String("Wait4Button"));
    lcd.print(F("   "));
  }


//  while (true)
//  {
//  //check_for_obstacle();
//  getNextCellBug(0, 0);
//  delay(100);
//  }

  Serial.begin(9600);
  

  //align_frames(start_pos);
}

void loop() {
  

  int cells_to_visit[20][2];
  int (*cells_to_visit_add)[20][2] = &cells_to_visit;
  getCellsToVist(cells_to_visit_add, start_pos, end_pos);


  char dir;
 // int cells_to_visit[20][2] = {{1,0},{2,0},{2,1},{2,2}, {2,3}, {2,4}};


  // forward

  int i = 0;
  int cell_path_indexer = 0;
  bool off_main_path = false;
  
  while (!(current_pos[0]==end_pos[0] && current_pos[1]==end_pos[1]))
  {
    int x_move = 0;
    int y_move = 0;
    bool obstacle = check_for_obstacle();
    
    if (obstacle)
    {
      lcd.gotoXY(0, 0);
      lcd.print(String("**Wall**"));
      lcd.print(F("   "));
      delay(1000);
      getNextCellBug(x_move, y_move);

        lcd.clear();
        lcd.gotoXY(0, 0);
        lcd.print(String(x_move));
        lcd.print(F("   "));
        lcd.gotoXY(0, 1);
        lcd.print(String(y_move));
        lcd.print(F("   "));
        delay(10000);
      
      current_pos[0] = current_pos[0]+x_move;
      current_pos[1] = current_pos[1]+y_move;
      delay(200);
    }
    else
    {
    x_move = cells_to_visit[i][0] - current_pos[0];
    y_move = cells_to_visit[i][1] - current_pos[1];

    current_pos[0] = cells_to_visit[i][0];
    current_pos[1] = cells_to_visit[i][1];
    i++;

    
      lcd.gotoXY(0, 0);
      lcd.print(String("Clear :)"));
      lcd.print(F("   "));
      
      delay(200);
    }
    
    // Figure out whether we need to turn
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
        while (true)
        {
          lcd.gotoXY(0, 0);
          lcd.print(String("Idiot"));
          lcd.print(F("   "));
        }
        break;
      }
      break;
    }

    turn(theta_desired, dir, 1);
    delay(100);
    drive(objective, theta_desired);
    delay(100);
      

  }

  turn(theta_desired, dir, 1);


  //find_dot();

  while (1){}
}


void drive(uint16_t objective, int16_t angle)
{
  while ( error > 2) {

    int leftMotor;
    int rightMotor;

    int32_t countsLeft = encoders.getCountsLeft();
    int32_t countsRight = encoders.getCountsRight();
    int32_t counts = (countsRight + countsLeft) / 2;
    const int radius = 19; //radius in millimeters
    const float countsToDistance = 2 * 3.14159 * radius / 1200; // convert between counts and millimeters
    double distance = counts * countsToDistance;
    error = objective - distance;
    int speedGain = 10;


    if ((time2 < (millis() - 100))) {
      velocity = (distance - prevDistance) * 1000 / (millis() - time2);
      if (velocity < 200) {
        maxSpeed = maxSpeed + 10;
      }
      else if (velocity > 400) {
        maxSpeed = maxSpeed - 10;
      }
      time2 = millis();
      prevDistance = distance;
    }
    // Read the gyro to update turnAngle
    turnSensorUpdate();

    // Calculate the motor turn speed using proportional and
    // derivative PID terms. add 90 degrees when objective is reached
    //  int32_t turnSpeed = -((int32_t)(turnAngle)) / (turnAngle1 / 45)- turnRate / 20;
    int32_t turnSpeed = (45 * (int32_t)((((angle * turnAngle1) - (int32_t)turnAngle)) / ((int32_t)turnAngle1)) - turnRate / 15);

    turnSpeed = constrain(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
    motorSpeed = error * speedGain;
    motorSpeed = constrain(motorSpeed, -maxSpeed, maxSpeed);
    leftMotor = motorSpeed - turnSpeed;
    rightMotor = motorSpeed + turnSpeed;
    motors.setSpeeds(leftMotor, rightMotor);

    if ( error <= 2) {
      motors.setSpeeds(0, 0);
      delay(100);
      countsLeft = encoders.getCountsAndResetLeft();
      countsRight = encoders.getCountsAndResetRight();
      prevDistance = 0;
      time1 = millis();
    }

    if ((uint8_t)(millis() - lastDisplayTime) >= 100) {
      lastDisplayTime = millis();
      lcd.clear();
      lcd.print(error);
      lcd.gotoXY(0, 1);
      lcd.print(velocity);
    }
  }
  error = 10000;
}


bool check_for_obstacle()
{
  uint16_t values[3] = {0};

    ir_sense(values);
    lcd.gotoXY(0, 0);
    lcd.print(String(values[1]));
    lcd.print(F("   "));

  if (values[1] > 10)
  {
    return true;
  }
  else
  {
    return false;
  }
//  
//  while (true)
//  {
//    ir_sense(values);
//    turnSensorUpdate();
//    lcd.gotoXY(0, 0);
//    lcd.print(String(values[1]));
//    lcd.print(F("   "));
//  }

}

void getNextCellBug(int &xdiff, int &ydiff)
{


  uint32_t current_theta_any;
  uint32_t current_theta_rounded=23;

  turnSensorUpdate();
  current_theta_any = turnAngle/turnAngle1;

  switch (current_theta_any)
    {
     case 0 ... 45:
     current_theta_rounded = 0;
     break;
     case 315 ... 359:
     current_theta_rounded = 0;
     break;
     case 46 ... 135:
     current_theta_rounded = 90;
     break;
     case 136 ... 225:
     current_theta_rounded = 180;
     break;
     case 226 ... 314:
     current_theta_rounded = 270;
     break;
     default:
     current_theta_rounded = 69;
     break;
   }

  bool front = false;
  bool right = false;
  bool left = false;

  uint16_t values[3] = {0};
  ir_sense(values);
  front = (values[1] > 10);
  right = (values[2] > 10);
  left = (values[3] > 10);
  lcd.clear();

  for (int i=0; i<4; i++)
  {
   ir_sense(values);
    
  front = front||(values[1] > 12);
  right = right||(values[2] > 5);
  left = left||(values[3] > 5);
  delay(200);
  }

//  while (true)
//  {
//  ir_sense(values);
//  front = (values[1] > 10);
//  right = (values[2] > 10);
//  left = (values[3] > 10);
//
  lcd.gotoXY(0, 0);
  lcd.print(front);
//  lcd.print(F("   "));
  lcd.gotoXY(0, 1);
  lcd.print(left);
//  lcd.print(F("   "));
  delay(1000);
//  }

  // Case, wall in front and no walls beside
  // Next cell will be to the left
  if ( (front)&(!left) )
  {
    lcd.gotoXY(0, 0);
    lcd.print(String("WL_FRNT"));
    delay(1000);
    switch (current_theta_rounded)
    {
     case 0:
     ydiff = 1;
     break;
     case 180:
     ydiff = -1;
     break;
     case 90:
     xdiff = -1;
     break;
     case 270:
     xdiff = 1;
     break;
   }
 }

  // Case, we don’t sense any walls anymore
  // Next cell will be to the right
  else if ( (!right)&(!front) )
  {
    lcd.gotoXY(0, 0);
    lcd.print(String("WL_NONE"));
    delay(1000);
    switch (current_theta_rounded)
    {
     case 0:
     ydiff = -1;
     break;
     case 180:
     ydiff = 1;
     break;
     case 90:
     xdiff = 1;
     break;
     case 270:
     xdiff = -1;
     break;
   }
 }

  // Case, we see the right wall still but no front
  // Next cell will be forward
  else if ( (right)&(!front) )
    {
    lcd.gotoXY(0, 0);
    lcd.print(String("WL_RGHT"));
    delay(1000);
    switch (current_theta_rounded)
    {
     case 0:
     xdiff = 1;
     break;
     case 180:
     xdiff = -1;
     break;
     case 90:
     ydiff = 1;
     break;
     case 270:
     ydiff = -1;
     break;
   }
 }
//  lcd.clear();
//  lcd.gotoXY(0, 0);
//  lcd.print(String(xdiff));
//  lcd.print(F("   "));
//  lcd.gotoXY(0, 1);
//  lcd.print(String(ydiff));
//  lcd.print(F("   "));
//  delay(10000);

  
}

