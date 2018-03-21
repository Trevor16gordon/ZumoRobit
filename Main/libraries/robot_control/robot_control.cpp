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

uint16_t ir_sense(uint16_t* values)
{
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
	}
	
	*values = left;
	*(values+1) = front;
	*(values+2) = right;
}


void ir_init()
{
	proxSensors.initThreeSensors();
	
	uint16_t levels[16];
	for(int i = 1; i < 16; i ++)
	{
		levels[i-1] = i*2;
	}
	
	proxSensors.setBrightnessLevels(levels, sizeof(levels)/2);
	
	ledYellow(1);
	lcd.clear();
	lcd.print(F("Line cal"));

//	for (uint16_t i = 0; i < 400; i++)
//	{
//		lcd.gotoXY(0, 1);
//		lcd.print(i);
//		lineSensors.calibrate();
//	}
//
//	ledYellow(0);
//	lcd.clear();
}


void align_frames(uint32_t *initial)
{
	int turn_count = 0;
	uint16_t values[3] = {0};
	uint16_t lengths[4] = {0};
	uint32_t error = 0;
	uint32_t m_speed = 400;
	
	// determine # of 90degree turns to rotate clockwise after max detect
	if ((initial[0] < 4)&&(initial[1] > 4))
	{
		turn_count = 1;
	}
	else if ((initial[0] > 4)&&(initial[1] > 4))
	{
		turn_count = 0;
	}
	else if ((initial[0] < 4)&&(initial[1] < 4))
	{
		turn_count = 2;
	}
	else
	{
		turn_count = 3;
	}
	
	turnSensorReset();
	
	//rotate in 90 degree increments recording distances
	for (int i=0; i<3; i++)
	{
		values = ir_sense(&values);
		lengths[i] = values[0]+values[1];
		
		// Turn 90 deg
		while(abs(turnAngle+(turnAngle90)) > (2*turnAngle1))
		{
		  turnSensorUpdate();
		  move_robot(&error, 0, vel, &m_speed);
		}
		
		turnSensorReset();
	}
	
	
	
	
}

}





