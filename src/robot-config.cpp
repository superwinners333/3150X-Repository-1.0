#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

// LEFT SIDE
motor LF = motor(PORT6, ratio6_1, false);
motor LM = motor(PORT5, ratio6_1, false);
motor LB = motor(PORT7, ratio6_1, true); // flipped
// RIGHT SIDE
motor RF = motor(PORT10, ratio6_1, true);
motor RM = motor(PORT9, ratio6_1, true);
motor RB = motor(PORT8, ratio6_1, false); // flipped

motor FrontRoller = motor(PORT1, ratio6_1, false);
motor BackRoller = motor(PORT18, ratio6_1, true); // right
// motor TopRoller = motor(PORT21, ratio18_1, true);

digital_out Scrapper = digital_out(Brain.ThreeWirePort.E);
digital_out Lift = digital_out(Brain.ThreeWirePort.C);
digital_out BackDescore = digital_out(Brain.ThreeWirePort.B); // double park
digital_out Wings = digital_out(Brain.ThreeWirePort.F);

inertial Gyro = inertial(PORT21);
//Naming convention: 
// Important variables
const double wheelDiam = 3.25;
const double wheelToMotorRatio = 36/48.0; // make sure one of the numbers is a decimal so it stays a float


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