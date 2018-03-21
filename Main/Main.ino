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
uint16_t values[3] = {0};


void setup() {
  
  ir_init();
}

void loop() {
  ir_sense(values);

  printReadingsToSerial();
}

void printReadingsToSerial()
{
  static char buffer[80];
  sprintf(buffer, "%d %d %d\n",
  values[0],
	values[1],
	values[2]
  );
  Serial.print(buffer);
}
