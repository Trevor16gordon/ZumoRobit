#include <Wire.h>
#include <Zumo32U4.h>
#include <TurnSensor.h>
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


uint32_t speed_control(uint32_t* error)
{
	// function takes error to calculate speed
	// 
	//	inputs:
	//			error pointer
	//	output:
	//			speed value
	
	
	uint32_t motorSpeed;
	int speedGain = 15;
	
	motorSpeed = (*error) * speedGain;
	
	return motorSpeed;
	
}


uint32_t turn_control(int angle)
{
	// function for PD control of robot turning
	// 
	//	inputs:
	//			angle - target angle (pass 0 for tracking forward) 
	//	output:
	//			turnSpeed - value for motors from PD controller
	
	uint32_t turnSpeed = -((int32_t)(turnAngle + angle*turnAngle1)) / (turnAngle1 / 45) - turnRate / 30;

	
	return turnSpeed;
}


float vel(uint32_t t1, uint32_t t2, uint32_t deltad)
{
	uint32_t v;
	v = deltad/(t2-t1);
	
	return v;
}


void move_robot(uint32_t* error, int dir, uint32_t vel, uint32_t* max_speed)
{
	
	int leftMotor;
    int rightMotor;
	
	uint32_t e = *error;

	
	if (*error < THRESHOLD)
	{
		
		uint32_t t_speed = turn_control(dir);
		motors.setSpeeds(-t_speed, t_speed);


	}
	else
	{	

		uint32_t m_speed = speed_control(&e);
		uint32_t t_speed = turn_control(dir);
		
		if(vel < 200){
		  *max_speed = + 10; 
		}
		else if(vel > 400){
		  *max_speed =- 10;
		}
		
		constrain(m_speed, -*max_speed, *max_speed);
			
		leftMotor = m_speed - t_speed; 
		rightMotor = m_speed + t_speed;
	
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


void align_frames(uint32_t *initial)
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
		ir_sense(values);
		lengths[i] = values[0]+values[1];
		
		motors.setSpeeds(0,0);
		
		turn(89, 'R');
		
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
	Serial.println(turn_count);
	Serial.println(max_index);
	
	int alignment = (max_index + turn_count);
	char dir = 'R';
	
	if (alignment > 4)
	{
		alignment = alignment-4;
	}		
	//align coordinate frame & world frame
	for (int k=0;k<alignment;k++)
	{
		turn(89, dir);
		delay(1000);
		Serial.println('here');
		turnSensorReset();
	}
}
	

void turn(int angle, char direction)
{
	// function for turning arbritrary angle
	// 
	//	inputs:
	//			angle - target angle (w.r.t gyro 0 degree angle)
	//			direction - char 'L' or 'R' 		
	//	output:
	//			NONE
	
	uint32_t tspeed; 
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
		  tspeed = turn_control(angle);
		  motors.setSpeeds(-(dir)*tspeed, dir*tspeed);
		  
		 lcd.gotoXY(0, 0);
		lcd.print((((int32_t)turnAngle >> 16) * 360) >> 16);
	}
	
	motors.setSpeeds(0,0);
}
	
	








