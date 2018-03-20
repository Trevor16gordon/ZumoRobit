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


uint32_t turn_control(int i)
{
	// function for PD control of robot turning
	// 
	//	inputs:
	//			i - turn angle (pass 0 for tracking forward) 
	//	output:
	//			turnSpeed - value for motors from PD controller
	
	uint32_t turnSpeed = -((int32_t)(turnAngle + i*turnAngle90)) / (turnAngle1 / 90) - turnRate / 3;

	
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
		motors.setSpeeds(t_speed, -t_speed);


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






