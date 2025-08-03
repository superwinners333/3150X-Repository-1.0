using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LF;
extern motor LM;
extern motor RF;
extern motor RM;
extern motor LB;
extern motor RB;
extern motor FrontRoller;
extern motor BackRoller;
extern motor TopRoller;
extern digital_out Scrapper;
extern digital_out Lift;
extern motor PU;
extern inertial Gyro;

extern const double wheelDiam;
extern const double wheelToMotorRatio;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );