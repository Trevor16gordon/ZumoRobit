
#include <Zumo32U4.h>
#include <robot_control.h>
#include <TurnSensor.h>

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

Zumo32U4LCD lcd;
L3G gyro;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;



int start_pos[2] = {0};
int current_pos[2] = {0};
int end_pos[2] = {0};



uint16_t objective = 210;
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


void setup() {

  enter_start(start_pos);

  delay(500);
  lcd.clear();

  enter_start(end_pos);

  delay(500);
  lcd.clear();
  
  for(int i=0; i<2; i++)
  {
    current_pos[i] = start_pos[i];
  }

  turnSensorSetup(); // intialize Gyro
  delay(500);

  // after being placed fetch best estimate of location (uses kevins IR function)
  //runs once


  ir_init();
  //line_sense_init();
  delay(200);

  turnSensorReset(); // Reset Orientation to zero

  align2(start_pos);

  // Start playing note A in octave 4 at maximum volume
  // volume (15) for 2000 milliseconds.
  buzzer.playNote(NOTE_A(4), 2000, 15);

  // Wait for 200 ms and stop playing note.
  delay(200);
  buzzer.stopPlaying();
  
  Serial.begin(9600);
}

void loop() {
  
  //int cells_to_visit[20][2];
  //int (*cells_to_visit_add)[20][2] = &cells_to_visit;
  //getCellsToVist(cells_to_visit_add, start_pos, end_pos);


  char dir;
  int cells_to_visit[20][2] = {{10,5},{10,4},{10,3},{10,2},{10,1},{9,1},{8,1},{7,1},{6,1},{5,1},{4,1},{4,2},{4,3},{4,4},{5,4},{5,5}};

  int i = 0;
  int cell_path_indexer = 0;
  bool off_main_path = false;
  String bug_dir = "undecided";
  
  while (!(current_pos[0]==end_pos[0] && current_pos[1]==end_pos[1]))
  {
    int x_move = 0;
    int y_move = 0;
    
    if (off_main_path)
    {
      
      lcd.gotoXY(0, 0);
      lcd.print(String("*OffPath*"));
      lcd.print(F("   "));
      delay(500);
      getNextCellBug(x_move, y_move, bug_dir);

//        lcd.clear();
//        lcd.gotoXY(0, 0);
//        lcd.print(String(x_move));
//        lcd.print(F("   "));
//        lcd.gotoXY(0, 1);
//        lcd.print(String(y_move));
//        lcd.print(F("   "));
//        delay(1000);
      
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
    switch (x_move)
    {
    case -1:
      theta_desired = 180;
      break;
    case 1:
      theta_desired = 0;
      break;
    case 0: 
      switch(y_move)
      {
        case -1:
        theta_desired = 270;
        break;
        case 1:
        theta_desired = +90;
        break;
        default:
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
      default:
      while (true)
        {
          lcd.gotoXY(0, 0);
          lcd.print(String("default"));
          lcd.print(F("   "));
          lcd.gotoXY(0, 1);
          lcd.print(String(x_move));
          lcd.print(F("   "));
        }
      break;
    }

    turn(theta_desired, dir, 1);
    delay(100);




    bool obstacle = check_for_obstacle();

    if (!obstacle)
    {
    turn(theta_desired, dir, 1);
    delay(100);
    drive(objective, theta_desired);
    }
    else
    {
      lcd.gotoXY(0, 0);
      lcd.print(String("*Wall*"));
      lcd.print(F("   "));
      delay(500);
      current_pos[0] = current_pos[0]-x_move;
      current_pos[1] = current_pos[1]-y_move;
      off_main_path = true;
      continue;
    }
    delay(100);


    // After moving if we were off track, lets check if were back on track
    if (off_main_path)
    {
    // Loop through initial path and see if we are back on track
       for (int jj=i; jj<20; jj++)
       {
        // Uncomment to print cell checking
//        lcd.clear();
//        lcd.gotoXY(0, 0);
//        char bufferval[8];
//        snprintf(bufferval,sizeof(bufferval), "? (%i,%i)", cells_to_visit[jj][0], cells_to_visit[jj][1]);
//        lcd.print(bufferval);
//        lcd.print(F("   "));
//        lcd.gotoXY(0, 1);
//        char bufferval2[8];
//        snprintf(bufferval2,sizeof(bufferval2), "@ (%i,%i)", current_pos[0], current_pos[1]);
//        lcd.print(bufferval2);
//        lcd.print(F("   "));
//        delay(100);
        
        if (current_pos[0]==cells_to_visit[jj][0] && current_pos[1]==cells_to_visit[jj][1])
        {
              i = jj +1;
              lcd.clear();
              lcd.gotoXY(0, 0);
              lcd.print(String("HOLY"));
              lcd.print(F("   "));
              lcd.gotoXY(0, 1);
              lcd.print(String("SHIT"));
              lcd.print(F("   "));
              delay(500);
              lcd.clear();
              off_main_path = false;
              bug_dir == "undecided";
              break;
              
        }
        else
        {
          off_main_path = true;
        }
       }
    }

  }

  turn(theta_desired, dir, 1);



  //find_dot();

  while (1){
              lcd.clear();
              lcd.gotoXY(0, 0);
              lcd.print(String("GOAL"));
              lcd.print(F("   "));
              lcd.gotoXY(0, 1);
              lcd.print(String(":)"));
              lcd.print(F("   "));
              delay(500);
    }
}


void drive(uint16_t objective, int16_t angle)
{

//    while(abs(angle - turnAngle/turnAngle1) > 10 && abs(angle - turnAngle/turnAngle1) < 355)
//    {
//        lcd.clear();
//        lcd.gotoXY(0, 0);
//        lcd.print(String("SELF"));
//        lcd.print(F("   "));
//        lcd.gotoXY(0, 1);
//        lcd.print(String("DSTRCT"));
//        lcd.print(F("   "));
//        delay(500);
//        turn(angle, "L", 1);
//        turnSensorUpdate();
//    }
    

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

  if (values[1] > 29)
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

void getNextCellBug(int &xdiff, int &ydiff, String &bug_dir)
{


  uint32_t current_theta_any;
  uint32_t current_theta_rounded=23;

  turnSensorUpdate();
  current_theta_any = turnAngle/turnAngle1;

  switch (current_theta_any)
    {
     case 0 ... 45:
     current_theta_rounded = 0;
     turnSensorReset();
     delay(200);
     break;
     case 315 ... 359:
     current_theta_rounded = 0;
     turnSensorReset();
     delay(200);
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
  lcd.clear();

  for (int i=0; i<10; i++)
  {
   ir_sense(values);
    
  front = front||(values[1] > 29);
  right = right||(values[2] > 29);
  left = left||(values[0] > 29);
  delay(50);
  }

//  lcd.gotoXY(0, 0);
//  lcd.print(front);
//  lcd.gotoXY(0, 1);
//  lcd.print(left);
//  delay(1000);


  // Case, wall in front and no walls beside
  // Next cell will be to the left
  if ( (front&&(right||(bug_dir == "undecided"))) )
  {
    bug_dir = "left";
    lcd.gotoXY(0, 0);
    lcd.print(String("WL_FRNT"));
    delay(200);

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

  // Case, we don’t sense any walls to right
  // Next cell will be to the right
  else if ( (!right)&&(bug_dir == "left") )
  {
    lcd.gotoXY(0, 0);
    lcd.print(String("RGHT_OPEN"));
    delay(200);
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
  else if ( (right)&(!front)&(bug_dir=="left"))
    {
    lcd.gotoXY(0, 0);
    lcd.print(String("WL_RGHT"));
    delay(200);
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

 else
 {

  while (true)
  {
      lcd.clear();
      lcd.gotoXY(0, 0);
      lcd.print(String("Unkown"));
      lcd.print(F("   "));
      lcd.gotoXY(0, 1);
      lcd.print(String("Wall"));
      lcd.print(F("   "));
      delay(5000);
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

