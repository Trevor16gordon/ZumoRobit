#include <Zumo32U4.h>
#define MAX_SPEED ((uint16_t) 400)
#define RADIUS_MM ((int) 19)
#define COUNTS_FACTOR ((float) (2 * 3.14159 * RADIUS_MM / 1200))
#define THRESHOLD ((int) 3)

// Robot Dynamics Prototypes
bool distance_reached(uint16_t counts, uint16_t objective, uint32_t* error, uint32_t* distance);
uint32_t turn_control(int angle);
uint32_t turn_control_moving(int angle);
uint32_t speed_control(uint32_t* error);
float vel(uint32_t t1, uint32_t t2, uint32_t deltad);
void move_robot(uint32_t* error, int angle_desired, uint32_t* max_speed);
void align_frames(int *initial);
void ir_sense(uint16_t* values);
void ir_init();
void turn(int angle, char direction, bool moving);
void forward(uint32_t objective, int theta);
void getCellsToVist(int (*cells_to_visit)[20][2], int* start_position, int* end_position);


// These objects must be defined in your sketch.
extern uint32_t turnAngle;
extern int16_t turnRate;

extern Zumo32U4ButtonA buttonA;
extern Zumo32U4LCD lcd;
extern L3G gyro;
extern Zumo32U4Motors motors;
extern Zumo32U4LineSensors lineSensors;
extern Zumo32U4ProximitySensors proxSensors;
extern Zumo32U4LCD lcd;
extern Zumo32U4Encoders encoders;

