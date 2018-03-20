#include "Estimation.h"

double target[3][3] = {0}; // intialize as zeros 
double R[3][3] = {0.1}; 
double Q[3][3] = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1}};
double G[3][3] = {0};


int interrupt_count = (CLOCK_FREQ/TIMER_MAX/10); //10Hz sampling frequency dt=0.1s


void bayes(double* grid, char cardinal)
{	// is this necessary for requirement 1? initial estimation???
	
	switch (cardinal)
	{
	
		case 'N':
			target = {{0, IR_ERROR, 0}, {IR_ERROR, IR_ERROR, IR_ERROR}, {0 (1-IR_ERROR) 0}};
		
		case 'S':
			target = {{0, (1-IR_ERROR), 0}, {IR_ERROR, IR_ERROR, IR_ERROR}, {0 (IR_ERROR) 0}};
		
		case 'E':
			target = {{0, IR_ERROR, 0}, {(1-IR_ERROR), IR_ERROR, IR_ERROR}, {0 (IR_ERROR) 0}};
		
		case 'W':
			target = {{0, IR_ERROR, 0}, {IR_ERROR, IR_ERROR, (1-IR_ERROR)}, {0 IR_ERROR 0}};
	}

	
	for (int x=0; x++; (WIDTH-1))
	{
		for (int y=0; y++; (WIDTH-1))
		{
			
		}
	}
}


void sampleInit()
{	// initializes TIMER1 with interrupt configured on compare value
	// necessary for accurate sampling intervals dt

	noInterrupts(); // disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;

	OCR1A = interrupt_count; // compare match register 
	TCCR1B |= (1 &lt;&lt; WGM12); // CTC mode
	TCCR1B |= (1 &lt;&lt; CS12); // 256 prescaler
	TIMSK1 |= (1 &lt;&lt; OCIE1A); // enable timer compare interrupt
	interrupts(); // enable all interrupts
}


ISR(Timer1_COMPA_vect)
{
	//interupt service routine called when TIMER1 issues interupt
	
	TCNT1 = interrupt_count; // reset count register in timer
	EKF(&state_hat, &p, u, v, w, sensors) // call kalman filter function after DT interval
}


void EKF(double* state_hat, double* p, double u, double v, double w, double sensors)
{
	
}
