#include <Zumo32U4.h>
//#include <Estimation.h>
#include <robot_control.h>
//#include <TurnSensor.h>
Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
L3G gyro;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
//uint16_t values[3] = {0};
uint16_t state;

void setup() {
 
  //ir_init();
  line_sense_init();
}

void loop() {

  state = line_sense();
  // ir_sense(values);

  printReadingsToSerial();
}

void printReadingsToSerial()
{
  static char buffer[80];
  sprintf(buffer, "%d\n",
  state
	//values[1],
	//values[2]
  );
  Serial.print(buffer);
}
