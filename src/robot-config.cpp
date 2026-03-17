#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

// LEFT SIDE
motor LF = motor(PORT10, ratio6_1, true);
motor LM = motor(PORT16, ratio6_1, true);
motor LB = motor(PORT19, ratio6_1, false); // flipped
// RIGHT SIDE
motor RF = motor(PORT2, ratio6_1, false);
motor RM = motor(PORT12, ratio6_1, false);
motor RB = motor(PORT13, ratio6_1, true); // flipped

motor LeftRoller = motor(PORT20, ratio6_1, false);
motor RightRoller = motor(PORT8, ratio6_1, true); // right

motor lever = motor(PORT15, ratio36_1, true); // lever arm

digital_out Scrapper = digital_out(Brain.ThreeWirePort.G); // scraper
digital_out Lift = digital_out(Brain.ThreeWirePort.B); // changes scoring modes
digital_out lock = digital_out(Brain.ThreeWirePort.F); // blocks front of intake
digital_out Wings = digital_out(Brain.ThreeWirePort.H); // wings
digital_out LiftUp = digital_out(Brain.ThreeWirePort.D); // old
digital_out LiftDown = digital_out(Brain.ThreeWirePort.E); // old
digital_out Funnel = digital_out(Brain.ThreeWirePort.A); // back descore / low goal funnel

rotation levertracker = rotation(PORT14, false); // for lever tracking
rotation odomx = rotation(PORT3,false); // positive should be to the right
inertial Gyro = inertial(PORT11);
distance backSensor = distance(PORT18);
distance leftSensor = distance(PORT17);
distance rightSensor = distance(PORT1);
distance frontSensor = distance(PORT21);

bumper jumpbutton = bumper(Brain.ThreeWirePort.C);


// Important variables
const double wheelDiam = 3.25;
const double wheelToMotorRatio = 36.0/48.0; // make sure one of the numbers is a decimal so it stays a float




const double DistanceBetweenWheel = 10.75;
const double horizontal_tracker_diameter = 2.75;
const double horizontal_tracker_dist_from_center = 6.417323;
const double wheel_distance_in = (36.0 / 48.0) * 3.3 * M_PI;
bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = false;   // Less accel/decel due to expecting direction change at start of movement
bool dir_change_end = false;     // Less accel/decel due to expecting direction change at end of movement

double min_output = 10; // Minimum output voltage to motors while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;

// Prevents too much slipping during boomerang movements
// Decrease if there is too much drifting and inconsistency during boomerang
// Increase for more speed during boomerang
double chase_power = 10;








// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}