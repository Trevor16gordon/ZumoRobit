#DEFINE WIDTH ((int) 8)
#DEFINE IR_ERROR (float 0.2)
#DEFINE CLK_FREQ ((int) 16000000) //16MHz ATMEGA clock frequency
#DEFINE TIMER_MAX ((int) 256) //8bit for timer2

//Function Prototypes
void bayes(double* grid)
void EKF(double* state_hat, double* p, double u, double v, double w, double sensors)

//external variables - MUST be declared in MAIN
double state_hat[3];
double u[2];
double p[3][3];
double sensors;
double v;
double w;


