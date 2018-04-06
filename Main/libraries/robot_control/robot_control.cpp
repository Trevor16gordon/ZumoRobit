#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"
#include "robot_control.h"

//function declarations

bool distance_reached(uint16_t counts, uint16_t objective, uint32_t* error, uint32_t* distance)
{
	// function takes encoder count to quantify distance travelled
	// 
	//	inputs:
	//			count - encoder counts 
	//			objective - distance in mm 
	//			error pointer
	//			distance pointer
	//	output:
	//			state - t/f on if distance has been reached
	bool state;
	
	*distance = counts*COUNTS_FACTOR;
	
	*error = objective - *distance;
	
	if ((counts*COUNTS_FACTOR) < (objective-THRESHOLD))
	{
		state = 0;
		
	}
	else
	{
		state =1;
		*distance = 0;

	}
	
	return state;
}


int32_t speed_control(int32_t error)
{
	// function takes error to calculate speed
	// 
	//	inputs:
	//			error pointer
	//	output:
	//			speed value
	
	
	int32_t motorSpeed;
	int speedGain = 15;
	
	motorSpeed = (error) * speedGain;
	
	//Serial.println(motorSpeed);
	
	return motorSpeed;
	
}


int32_t turn_control(int angle)
{
	// function for PD control of robot turning
	// 
	//	inputs:
	//			angle - target angle (pass 0 for tracking forward) 
	//	output:
	//			turnSpeed - value for motors from PD controller
	
	int32_t turnSpeed = -((int32_t)(turnAngle + angle*turnAngle1)) / (turnAngle1 / 45);// - turnRate / 30;
	//Serial.print(turnSpeed);
	//turnSpeed = constrain(turnSpeed, (int16_t) -300, (int16_t) 300);

	
	return turnSpeed;
}

int32_t turn_control_moving(int angle)
{
	// function for PD control of robot turning
	// 
	//	inputs:
	//			angle - target angle (pass 0 for tracking forward) 
	//	output:
	//			turnSpeed - value for motors from PD controller
	
    int32_t error = (45*(int32_t)((((angle*turnAngle1)-(int32_t)turnAngle))/((int32_t)turnAngle1))-turnRate/15);
	//Serial.println((((int32_t)turnAngle) - angle*turnAngle1)/turnAngle1);
	
	return error;
}


int vel(uint32_t t1, uint32_t t2, uint32_t deltad)
{
	int v;
	v = 1000*deltad/(t2-t1);
	
	return v;
}


void move_robot(int32_t* error, int angle_desired, int vel, int16_t* max_speed)
{
	
	int leftMotor;
    int rightMotor;
	
	int32_t e = *error;
	
	//Serial.println(e);
	
	// lcd.gotoXY(0, 0);
    // lcd.print(vel);
    // lcd.print(F("   "));
    // delay(10);


	
	
	if (*error < THRESHOLD)
	{
		
		int32_t t_speed = turn_control(angle_desired);
		motors.setSpeeds(-t_speed, t_speed);


	}
	else
	{	

		int32_t m_speed = speed_control(e);
		int32_t t_speed = turn_control_moving(angle_desired);
		
		
		if(vel < 200){
		  *max_speed = *max_speed + 10; 
		}
		else if(vel > 400){
		  *max_speed = *max_speed - 10;
		}
		
		
		//m_speed = constrain(m_speed, -(*max_speed), *max_speed);
		
		//Serial.println(m_speed);
		
		t_speed = constrain(t_speed, -200, 200);
		m_speed = constrain(m_speed, -200, 200);

		
		leftMotor =  m_speed + t_speed; 
		rightMotor = m_speed - t_speed;
		
		//Serial.println(leftMotor);
	
		motors.setSpeeds(leftMotor, rightMotor);
	}
}	


void ir_sense(uint16_t* values)
{
	// function for using robots IR sensors
	// 
	//	inputs:
	//			values - pointer to array of length 3 to write [L,R,F] IR data back to
	//	output:
	//			NONE
	
	static uint16_t lastSampleTime = 0;
	
	uint16_t left;
	uint16_t front;
	uint16_t right;
	
	if ((uint16_t)(millis() - lastSampleTime) >= 100)
	{
		lastSampleTime = millis();

		// Send IR pulses and read the proximity sensors.
		proxSensors.read();
	

		left = proxSensors.countsLeftWithLeftLeds();
		//	proxSensors.countsLeftWithRightLeds();
		front = (proxSensors.countsFrontWithLeftLeds() + proxSensors.countsFrontWithRightLeds())/2;
		//	proxSensors.countsRightWithLeftLeds();
		right = proxSensors.countsRightWithRightLeds();
		
		values[0] = left;
		values[1] = front;
		values[2] = right;
	}
}


void ir_init()
{
	// function for initializing IR sensors
	// 
	//	inputs:
	//			NONE
	//	output:
	//			NONE
	
	proxSensors.initThreeSensors();
	
	uint16_t levels[16];
	for(int i = 1; i < 16; i ++)
	{
		levels[i-1] = i*2;
	}
	
	proxSensors.setBrightnessLevels(levels, sizeof(levels)/2);
}


void align_frames(int *initial)
{
	// function for aligning world & coordinate frames after
	//  initial placement with unknown orientation
	// 
	//	inputs:
	//			initial - initial coordinate placements in [x,y]
	//	output:
	//			NONE
	
	int turn_count = 0;
	int max_index = 0;
	int max =0;
	uint16_t values[3] = {0};
	uint16_t lengths[4] = {0};
	uint16_t avg = 0;
	uint32_t m_speed = 100;
	
	// determine # of 90degree turns to rotate clockwise after max detect
	if ((initial[0] < 5)&&(initial[1] > 4))
	{
		turn_count = 1;
	}
	else if ((initial[0] > 4)&&(initial[1] > 4))
	{
		turn_count = 0;
	}
	else if ((initial[0] < 5)&&(initial[1] < 5))
	{
		turn_count = 2;
	}
	else
	{
		turn_count = 3;
	}
	
	turnSensorReset();
	
	Serial.println(turn_count);
	Serial.println(max_index);
	
	//rotate in 90 degree increments recording distances
	for (int i=0; i<4; i++)
	{	
		
		for (int k=0;k<4; k++)
		{
			ir_sense(values);
			delay(100);
			avg = avg + values[0];
		}
		
		ir_sense(values);
		lengths[i] = values[0]+values[1];
		
		lcd.gotoXY(0, 0);
		lcd.print(values[1]);
		lcd.print(F("   "));
		lcd.gotoXY(0, 1);
		lcd.print(values[0]);
		lcd.print(F("   "));
		delay(100);
		
		delay(100);
		
		motors.setSpeeds(0,0);
		
		turn(89, 'R', 0);
		
		motors.setSpeeds(0,0);
		delay(1000);
		turnSensorReset();
		
	}
	
	// determine minimum distance orientation to walls
	for (int j=0; j<4; j++)
	{
		if (lengths[j] > max)
		{
			max_index = j;
			max = lengths[j];
		}
	}
	
	delay(1000);
	
	int alignment = (max_index + turn_count);
	
	if (alignment > 4)
	{
		alignment = alignment-4;
	}		
	
	//align coordinate frame & world frame
	for (int k=0;k<alignment;k++)
	{
		turn(89, 'R', 0);
		delay(1000);
		turnSensorReset();
	}
}
	

void turn(int angle, char direction, bool moving)
{
	// function for turning arbritrary angle
	// 
	//	inputs:
	//			angle - target angle (w.r.t gyro 0 degree angle)
	//			direction - char 'L' or 'R' 		
	//	output:
	//			NONE
	
	int32_t tspeed; 
	int dir;

	
	switch(direction)
	{
		case 'L':
			dir = -1;
			break;
		case 'R':
			dir = 1;
			break;
	}
	
	uint16_t t1 = millis();
	
	while(millis()-t1 < 1000)
	{
		  //Serial.println(((dir*((int32_t)(turnAngle))+angle*turnAngle1))/turnAngle1);
		  //Serial.println(dir);
		  turnSensorUpdate();
		  if (moving)
		  {
			  tspeed = turn_control_moving(angle);
		  }
		  else
		  {
				  tspeed = turn_control(angle);
		  }
		  tspeed = constrain(tspeed, -250, 250);
		  motors.setSpeeds(-tspeed, tspeed);
		  delay(1);
		  
		//lcd.gotoXY(0, 0);
		//lcd.print((((int32_t)turnAngle >> 16) * 360) >> 16);
	}
	
	motors.setSpeeds(0,0);
}


void forward(uint32_t objective, int theta)
{
	uint32_t distance=0;
	uint32_t d1=0;
	uint32_t d ;
	int32_t error = objective;
    uint32_t countsLeft = encoders.getCountsAndResetLeft();
    uint32_t countsRight = encoders.getCountsAndResetRight();
    uint32_t counts = 0;
	int16_t maxSpeed = 200;
	
	uint32_t t1 = 0;
	int v;

     while (!distance_reached(counts, objective, &error, &distance))
    {
		  d = distance - d1;
		  
		  
		 if (t1 < (millis()-100))
		{
			v = 1000*(d)/(millis()-t1);
			t1 = millis();
			
		}
		
		lcd.gotoXY(0, 0);
		lcd.print(v);
		lcd.print(F("   "));
		delay(10);
		  
	  
		  countsLeft = encoders.getCountsLeft();
		  countsRight = encoders.getCountsRight();
		  counts = (countsRight + countsLeft)/2;
		  
		  turnSensorUpdate();
		  



		  // Robot Dynamics function
		  move_robot(&error, theta, v, &maxSpeed);
		  
		  
		  d1 = distance;
    }
	
	//turn(90,'L', 1);
	
	motors.setSpeeds(0,0);
}
	

void getCellsToVist(int (*cells_to_visit)[20][2], int* start_position, int* end_position)

{
	// int start_pos[2] = {4,3};
	// int end_pos[2] = {0,1};

	// int end_pos[2] = {4,3};
	// int start_pos[2] = {0,1};

	// Get x_dir and y_dir
	int x_dir;
	int y_dir;

	int start_pos[2] = {4,3};
	int end_pos[2] = {0,1};
	int move_x = end_position[0] - start_position[0];
	int move_y = end_position[1] - start_position[1];
	int steps = abs(move_x) + abs(move_y);


	x_dir = move_x/abs(move_x);
	y_dir = move_y/abs(move_y);


	// Loop to generate cells_to_visit to cover x dir
	for (int i=0; i<x_dir*move_x; i++)
	{
		(*cells_to_visit)[i][0] = start_position[0] + (i+1)*x_dir;
		(*cells_to_visit)[i][1] = start_position[1];
	}

	// Loop to generate cells_to_visit to cover y dir
	for (int j=abs(move_x); j<abs(steps); j++)
	{
		(*cells_to_visit)[j][1] = start_position[1] + (j+1)*y_dir - y_dir*abs(move_x);
		(*cells_to_visit)[j][0] = end_position[0];
	}
}


void line_sense_init()
{

	lineSensors.initThreeSensors();
	
	delay(100);
	
	lineSensors.calibrate();

	
	ledYellow(1);
	lcd.clear();
	lcd.print(F("cal"));
	
	motors.setSpeeds(40,40);

	for (uint16_t i = 0; i < 400; i++)
	{
		lcd.gotoXY(0, 1);
		lcd.print(i);
		lineSensors.calibrate();
	}
	
	motors.setSpeeds(0,0);
	
	while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    lcd.gotoXY(0, 0);
    lcd.print(String("A"));
    lcd.print(F("   "));
  }
}


bool line_sense()
{

	int lineSensorValues[5] = {0};
	double avg = 0;
	
	bool state;
	
	for (int i=0;i<5; i++)
	{
		lineSensors.readCalibrated(lineSensorValues);
		avg += lineSensorValues[2];
	}
	
	avg = avg/5;
		
	Serial.println(avg);
	delay(50);
	
	if ((avg) < 500)
	{
		state = 1;
	}
	else 
	{
		state = 0;
	}
	
	return state;
}


void find_dot()
{
	uint16_t initial_distance = 250;
	uint16_t distance = 500;
	
	motors.setSpeeds(100, 100);
	
	  while(line_sense()==0)
	  {
		delay(10);
	  }

	  motors.setSpeeds(0,0);
		
	
}

void align2(int* initial)
{
	int turn_count;
	uint16_t values[3] = {0};
	int max_index =0;
	int firstInstance=-1;
	int secondInstance=0;
	int split =0;
	int turnNum;
	int thresh = 26;
	
	// determine # of 90degree turns to rotate clockwise after max detect
	
	//Top left
	if ((initial[0] < 5)&&(initial[1] > 4))
	{
		turn_count = 1;
		split = 2;
	}
	// top right
	else if ((initial[0] > 4)&&(initial[1] > 4))
	{
		turn_count = 0;
		split = 1;
	}
	// Lower Left
	else if ((initial[0] < 5)&&(initial[1] < 5))
	{
		turn_count = 2;
		split =3;
	}
	// Lower Right
	else
	{
		turn_count = 3;
		split = 0;
	}
	
	turnSensorReset();
	
	//turn and read front
	for(int k=0; k<4; k++)
	{
		delay(100);
		ir_sense(values);
		delay(100);
		
		uint16_t current = values[1];
		
/* 		lcd.gotoXY(0, 0);
		lcd.print(current);
		lcd.print(F("   ")); */
		
		delay(100);
		
		
		if ((current) > thresh && (firstInstance >-1))
		{
			secondInstance = k;
			break;
		}
		else if(current > thresh)
		{
			firstInstance = k;
			
			delay(1000);
		}
		
/* 		lcd.gotoXY(0, 0);
		lcd.print(current);
		lcd.print(F("   "));
		lcd.gotoXY(0, 1);
		lcd.print(firstInstance);
		lcd.print(F("   ")); */
		
		motors.setSpeeds(0,0);
		
		turn(89, 'R', 0);
		
		motors.setSpeeds(0,0);
		
		turnSensorReset();
		
		delay(100);
		

		
	}
	
		lcd.gotoXY(0, 0);
		lcd.print(firstInstance);
		lcd.print(F("   "));
		
		lcd.gotoXY(0, 1);
		lcd.print(secondInstance);
		lcd.print(F("   "));
		
		delay(3000);
		
		lcd.clear();
		
		delay(1000);
		lcd.gotoXY(0, 0);
		lcd.print(turnNum);
		lcd.print(F("   "));
	
		turnNum = turn_count;
		
		if ((secondInstance-firstInstance) > 2)
		{
			turnNum = split;
		}
		
		for(int j=0; j<turnNum; j++)
		{
		
			turn(89, 'R', 0);
			
			motors.setSpeeds(0,0);

				
			turnSensorReset();
		
			delay(100);
		}
		
}

void turn_control_enc(int desired, int angle)
{	

	int countsLeft = encoders.getCountsLeft();
	int countsRight = encoders.getCountsRight();
	
	int leftEnc_error;
	int rightEnc_error;
	
	double leftSpeed;
	double rightSpeed;
	double gyroError;
	
	leftEnc_error = -desired - countsLeft;
	rightEnc_error = +desired - countsRight;
	
	turnSensorUpdate();
	
    gyroError = 25*((int32_t)((((angle*turnAngle1)-(int32_t)turnAngle))/((int32_t)turnAngle1)));
	
	
    lcd.gotoXY(0, 1);
    lcd.print(gyroError);
	
	//left p controller
	leftSpeed = leftEnc_error - gyroError;
	rightSpeed = rightEnc_error + gyroError;
	
	constrain(leftSpeed, -400, 400);
	constrain(rightSpeed, -400, 400);
	
	motors.setSpeeds(leftSpeed, rightSpeed);


}

void enter_start(int *start)
{
	int x = 0;
	int y = 0;

	lcd.gotoXY(0,0);
	lcd.print(x);
	lcd.gotoXY(1,0);
	lcd.print(y);

	while(1)
	{
		lcd.gotoXY(0,0);
		lcd.print(x);
		lcd.gotoXY(1,0);
		lcd.print(y);
		
		delay(10);
		
		lcd.clear();
		
		delay(10);
		
		if(buttonA.getSingleDebouncedRelease())
		{
			x = x + 1;
		}
		
		if(buttonC.getSingleDebouncedRelease())
		{
			break;
		}
		
		
	}
	
	while(1)
	{
		lcd.gotoXY(0,0);
		lcd.print(x);
		lcd.gotoXY(1,0);
		lcd.print(y);
		
		delay(10);
		
		lcd.clear();
		
		delay(10);
		
		if(buttonA.getSingleDebouncedRelease())
		{
			y = y + 1;
		}
		
		if(buttonC.getSingleDebouncedRelease())
		{
			break;
		}
	}
	
	start[0] = x;
	start[1] = y;
}
	

	
	










