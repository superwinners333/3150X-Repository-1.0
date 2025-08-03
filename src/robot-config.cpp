#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LF = motor(PORT6, ratio6_1, false);
motor LM = motor(PORT13, ratio6_1, false);
motor RF = motor(PORT14, ratio6_1, true);
motor RM = motor(PORT10, ratio6_1, true);
motor LB = motor(PORT2, ratio6_1, false);
motor RB = motor(PORT16, ratio6_1, true);
motor FrontRoller = motor(PORT8, ratio18_1, false);
motor BackRoller = motor(PORT15, ratio6_1, false);
motor TopRoller = motor(PORT21, ratio18_1, true);

digital_out Scrapper = digital_out(Brain.ThreeWirePort.H);
digital_out Lift = digital_out(Brain.ThreeWirePort.A);

inertial Gyro = inertial(PORT12);
//Naming convention: 
// Important variables
const double wheelDiam = 2.75;
const double wheelToMotorRatio = 36/48.0;

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