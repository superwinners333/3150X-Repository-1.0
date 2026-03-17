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
extern motor LeftRoller;
extern motor RightRoller;
extern motor lever;


extern digital_out Scrapper;
extern digital_out LiftUp;
extern digital_out LiftDown;
extern digital_out lock;
extern digital_out Wings;
extern inertial Gyro;
extern digital_out Lift;
extern digital_out Funnel;

extern rotation levertracker;
extern rotation odomx;
extern distance backSensor;
extern distance leftSensor;
extern distance rightSensor;
extern distance frontSensor;

extern bumper jumpbutton;

extern const double wheelDiam;
extern const double wheelToMotorRatio;

extern const double DistanceBetweenWheel;
extern const double horizontal_tracker_diameter;
extern const double horizontal_tracker_dist_from_center;
extern const double wheel_distance_in;
extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;  

extern double min_output;
extern double max_slew_accel_fwd;
extern double max_slew_decel_fwd;
extern double max_slew_accel_rev;
extern double max_slew_decel_rev;
extern double chase_power;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );