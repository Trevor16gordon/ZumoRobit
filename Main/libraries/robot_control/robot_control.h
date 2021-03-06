#include <Zumo32U4.h>
#define MAX_SPEED ((uint16_t) 400)
#define RADIUS_MM ((int) 19)
#define COUNTS_FACTOR ((float) (2 * 3.14159 * RADIUS_MM / 1200))
#define THRESHOLD ((int) 3)
#define LEFT90 ((int) 768)
#define RIGHT90 ((int) -768)

// Robot Dynamics Prototypes
bool distance_reached(int32_t counts, uint16_t objective, uint32_t* error, uint32_t* distance);
int32_t turn_control(int angle);
int32_t turn_control_moving(int angle);
int32_t speed_control(int32_t error);
void move_robot(int32_t* error, int angle_desired, int16_t vel, int* max_speed);
void line_sense_init();
bool line_sense();
void find_dot();
void enter_start(int *start);


void align_frames(int *initial);
void align2(int *initial);
void ir_sense(uint16_t* values);
void ir_init();
void turn(int angle, char direction, bool moving);
void turn_control_enc(int desired, int angle);
void forward(uint32_t objective, int theta);
void getCellsToVist(int (*cells_to_visit)[20][2], int* start_position, int* end_position);
void line_sense_init();
bool line_sense();
void find_dot();



// These objects must be defined in your sketch.
extern uint32_t turnAngle;
extern int16_t turnRate;

extern Zumo32U4ButtonA buttonA;
extern Zumo32U4ButtonB buttonB;
extern Zumo32U4ButtonC buttonC;

extern Zumo32U4LCD lcd;
extern L3G gyro;
extern Zumo32U4Motors motors;
extern Zumo32U4LineSensors lineSensors;
extern Zumo32U4ProximitySensors proxSensors;
extern Zumo32U4LCD lcd;
extern Zumo32U4Encoders encoders;
//extern lineSensors;

