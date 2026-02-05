#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

// LEFT SIDE
motor LF = motor(PORT10, ratio6_1, false);
motor LM = motor(PORT16, ratio6_1, false);
motor LB = motor(PORT19, ratio6_1, true); // flipped
// RIGHT SIDE
motor RF = motor(PORT2, ratio6_1, true);
motor RM = motor(PORT12, ratio6_1, true);
motor RB = motor(PORT13, ratio6_1, false); // flipped

motor LeftRoller = motor(PORT20, ratio6_1, false);
motor RightRoller = motor(PORT8, ratio6_1, true); // right
// motor TopRoller = motor(PORT21, ratio18_1, true);

digital_out Scrapper = digital_out(Brain.ThreeWirePort.G);
digital_out Lift = digital_out(Brain.ThreeWirePort.A);
digital_out DoubleP = digital_out(Brain.ThreeWirePort.F);
digital_out Wings = digital_out(Brain.ThreeWirePort.H);
digital_out LiftUp = digital_out(Brain.ThreeWirePort.D);
digital_out LiftDown = digital_out(Brain.ThreeWirePort.E);

inertial Gyro = inertial(PORT11);
distance backSensor = distance(PORT18);
distance leftSensor = distance(PORT15);
distance rightSensor = distance(PORT14);

// Important variables
const double wheelDiam = 3.25;
const double wheelToMotorRatio = 36.0/48.0; // make sure one of the numbers is a decimal so it stays a float


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