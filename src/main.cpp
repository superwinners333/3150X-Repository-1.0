/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

#include "screen_gui.hpp"
#include "movement.hpp"
#include "odom.hpp"
#include "Odometry.hpp"
#include "VirtualTargetPursuit.hpp"
#include "routes/routes.hpp"

using namespace vex;

// A global instance of competition
competition Competition;


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

bool SP;
bool EXIT;
void pre_auton(void) {
   EXIT=false;
  odomLift.set(false);
  Scrapper.set(false);
  Funnel.set(false);
  Wings.set(false);
  Pistake.set(true);
  leverLift(true);
  levertracker.setPosition(0,degrees);
  levertracker.resetPosition();
  confirmed = false;
  confirmed2 = false;
  PX=0;
  JX=0;
  AutoSelectorVal=0; // 0 = default
  SP=false;

  double global_target_x = 0;
  double global_target_y = 0;

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Gyro.calibrate();

//Ensure Robot Launch Position is set before auto proceeds, once plugged into field control,
//start program and do not temper bot under all circumstances

//1. IF ANY ADJUSTMENT IS NEEDED, QUIT PROGRAM, THEN ADJUST, RESTART PROGRAM AFTER ADJUSTMENTS COMPLETED
//2. DO NOT START PROGRAM BEFORE PLUGGING IN FIELD CONTROL, THIS MAY DISABLE AUTO
//3. ONLY SIGNAL JUDGES TO BEGIN MATCH AFTER THE ZEROING PROMPT ON SCREEN HAS CLEARED

//Print precautionary message
Brain.Screen.drawRectangle(0,0,500,500);
Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.setCursor(2,5);
Brain.Screen.print("DO NOT MOVE ROBOT");

Brain.Screen.setPenWidth(3); // important

waitUntil(!Gyro.isCalibrating());


Zeroing(true,true,true);

wait(100,msec); // just a small delay

// thread Odom = thread(OdomUpdate); // runs the odom stuff
thread odom = thread(OdomWithX);


Brain.Screen.clearScreen();
drawField(); // draws field
confirmed = false;
confirmed2 = false;

while(!confirmed && !EXIT) // waits for field corner selection
{ 
confirmCorner();
wait(20,msec); // gives a cooldown before looping again
}
Brain.Screen.clearScreen();
// greyScreen();

wait(50,msec);
while(!confirmed2 && !EXIT) // waits for auto selection
{ 
  AutoSelection();
  wait(20,msec); // gives a small cooldown before looping again
}

// determines which auto to run and creates confirm menu
AutonLogic(); 
// wait(5,sec);
// drawLogo();

// flappybird();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void) {
  
  if (!confirmed) AutoSelectorVal = 3; // for automatic auto selection


  Brain.Screen.clearScreen();

  Brain.Screen.setFont(monoL);
  Brain.Screen.setPenColor("#808080");
  Brain.Screen.setCursor(3,3);
  Brain.Screen.print("1, 2 miss a few ahhhhhh auto");
  Brain.Screen.setCursor(4,5);
  Brain.Screen.print("AUTO CONFIRMED");

  Wings.set(true);
  
  //Do not change the below
  PIDDataSet TestPara={4,0.1,0.2};
  Zeroing(true,true,true);

  switch (AutoSelectorVal) {
    // high side autos
    case 1: high_basic(); break;
    case 2: high_long(); break;
    case 3: high_rush(); break;
    case 4: high_middle_wing(); break;
    case 5: high_four(); break;
    case 6: high_counter_rush(); break;
    case 7: high_wingbreak(); break;
    case 8: high_all(); break;
    case 9: break;
    case 10: break;
    case 11: break;
    case 12: break;
    // low side autos
    case 13: low_basic(); break;
    case 14: low_long(); break;
    case 15: solo_awp(); break;
    case 16: low_rush(); break;
    case 17: low_middle_wing(); break;
    case 18: low_four(); break;
    case 19: push_awp(); break;
    case 20: break;
    case 21: break;
    case 22: break;
    case 23: break;
    case 24: break;

    // skills autos
    case 25: skills(); break;
    case 26: small_yahuskills(); break;
    case 27: yahuskills(); break;
    case 28: break;
    case 29: break;
    case 30: mangoskills(); break;
    default: break;
  }

  CStop(); 
}

int RV;
int LV;

bool middleActiv = false;
double turnConst = 3.25/3.25;
int DriveTask(void){
  odomLift.set(true);
  while(true)
  {
    EXIT=true;
    odomTracking = false;
    RV=Controller1.Axis3.position(percent)-(Controller1.Axis1.position(percent)*turnConst);
    LV=Controller1.Axis3.position(percent)+(Controller1.Axis1.position(percent)*turnConst);
    Move(LV,RV);
  }

return 0;
}
int V;
bool TopIntake = false;
int UpTaskActiv,DownTaskActiv;
int ButtonPressingRight,RightTaskActiv;
int ButtonPressingR1,R1TaskActiv;
int ButtonPressingLeft,LeftTaskActiv;
int ButtonPressingX,XTaskActiv;
int ButtonPressingY,YTaskActiv;
int ButtonPressingA,ATaskActiv; 
int ButtonPressingB;
int BTaskActiv=0;
int ButtonPressingDown;
int ButtonPressingUp;
int leverSpeed = 100;

int maxLeverAngle = 125;
bool lowgoal = false;

int ATask(void)
{
    while(true)
  {
    if (Controller1.ButtonL2.pressing()==1)
    {
      Pistake.set(false);
      RunIndex(-75); // 40
      lock.set(false);
    }
    else if (Controller1.ButtonL1.pressing()==1) 
    {
      Pistake.set(true);
      RunIndex(100);
      lock.set(false);
    }
    else if (!Controller1.ButtonLeft.pressing() && !Controller1.ButtonUp.pressing() && RightTaskActiv == 0 && R1TaskActiv == 0 && !Controller1.ButtonX.pressing() && !Controller1.ButtonY.pressing())
    {
      Pistake.set(true);
      RunIndex(0);
      lock.set(false);
    } 
  }
  
  return 0;
}

timer levertime;
bool upwards = true;
double exittime = 0;
bool waiting = false;

int PTask(void)
{
    while(true)
    {
      //Toggles Lift
    
    /*
    // if(XTaskActiv == 0) Lift.set(false); 
    if(XTaskActiv==0&&Controller1.ButtonX.pressing()&&ButtonPressingX==0)
    {
      ButtonPressingX=1;
      XTaskActiv=1;
      Lift.set(true);
    }

    else if(!Controller1.ButtonX.pressing())ButtonPressingX=0;

    else if(XTaskActiv==1&&Controller1.ButtonX.pressing()&&ButtonPressingX==0)
    {
      ButtonPressingX=1;
      XTaskActiv=0;
      Lift.set(false);
    }
    */




    //----------------------------------------- LEVER

    if (RightTaskActiv == 0 && R1TaskActiv == 0) {
      if (Controller1.ButtonX.pressing()) {
        RunLever(100);
        RunIndex(100);
        lock.set(true);
      }
      else if (Controller1.ButtonY.pressing()) {
        RunLever(-100);
        // RunIndex(-100);
        lock.set(true);
      }
      else RunLever(0);
    } 

  // -------------------------------------- Scrapper
    // Toggles Scrapper
    if(BTaskActiv == 0) Scrapper.set(false); // turns off scrapper after auton in case if its on
    if(BTaskActiv==0&&Controller1.ButtonB.pressing()&&ButtonPressingB==0)
    {
      ButtonPressingB=1;
      BTaskActiv=1;
      Scrapper.set(true);
    }
    else if(!Controller1.ButtonB.pressing())ButtonPressingB=0;
    else if(BTaskActiv==1&&Controller1.ButtonB.pressing()&&ButtonPressingB==0)
    {
      ButtonPressingB=1;
      BTaskActiv=0;
      Scrapper.set(false);
    }

    // -------------------------------------- Double Park

    // Activates lever (with acceleration)
    if(RightTaskActiv==0&&R1TaskActiv==0&&Controller1.ButtonRight.pressing()) {
      RightTaskActiv=1;
      levertime.clear();
      upwards = true;
      exittime = 0;
      waiting = false;
    }
    if (RightTaskActiv==1) {
      if (upwards) {
        if (liftUp) maxLeverAngle = 115;
        else maxLeverAngle = 135;
        RunIndex(100);
        lock.set(true);
        if (liftUp && levertracker.position(degrees) < 35) RunLever(50); // runs the lever slow to get the blocks in a line
        else if (levertracker.position(degrees) < maxLeverAngle) RunLever(leverSpeed);
        else { // THIS IS THE ACCELERATING CODE
          upwards = false;
          waiting = true;
          exittime = levertime.value() + 0.3; // time to wait for before exiting
        }
      } 
      else if (waiting) { // THIS IS THE ACCELERATING CODE
        if (levertime.value() > exittime) { // pauses to let the lever settle
          waiting = false; // unpauses
        }
      }
      else { // THIS IS THE ACCELERATING CODE
        RunIndex(-100);
        if (levertracker.position(degrees) > 3) RunLever(-100);
        else RightTaskActiv=0;
      }
      // auto exit code to prevent stalling
      if (levertime.value() > 1.3 && liftUp) { // less time because lever moves faster for long goal
        R1TaskActiv=0; 
        RunLever(0);
      }
      else if (levertime.value() > 1.6) { // more time cause lever moves slower for middle goal
        R1TaskActiv=0;
        RunLever(0);
      }
    }

    // Lever with NO acceleration
    if(RightTaskActiv==0&&R1TaskActiv==0&&Controller1.ButtonR1.pressing()) {
      R1TaskActiv=1;
      levertime.clear();
      upwards = true;
      exittime = 0;
      waiting = false;
    }
    if (R1TaskActiv==1) {
      if (upwards) {
        if (liftUp) maxLeverAngle = 115;
        else maxLeverAngle = 135;
        RunIndex(100);
        lock.set(true);
        if (levertracker.position(degrees) < maxLeverAngle) RunLever(leverSpeed);
        else {
          upwards = false;
          waiting = true;
          exittime = levertime.value() + 0.3; // time to wait for before exiting
        }
      } 
      else if (waiting) {
        if (levertime.value() > exittime) { // pauses to let the lever settle
          waiting = false; // unpauses
        }
      }
      else {
        RunIndex(-100);
        if (levertracker.position(degrees) > 3) RunLever(-100);
        else R1TaskActiv = 0;
      }
      // auto exit code to prevent stalling
      if (levertime.value() > 1.3 && liftUp) { // less time because lever moves faster for long goal
        R1TaskActiv=0; 
        RunLever(0);
      }
      else if (levertime.value() > 1.6) { // more time cause lever moves slower for middle goal
        R1TaskActiv=0;
        RunLever(0);
      }
    }

  // -------------------------------------- lift
    // Toggles lift
    if (RightTaskActiv == 0 && R1TaskActiv == 0) {
      if (!Controller1.ButtonL2.pressing() && BTaskActiv == 0) {
        leverLift(liftUp);
        if(Controller1.ButtonDown.pressing()&&ButtonPressingDown==0)
        {
          if (liftUp) liftUp = false;
          else liftUp = true;
          ButtonPressingDown=1;
        }
        else if(!Controller1.ButtonDown.pressing())ButtonPressingDown=0;
      } 
      else {
        Lift.set(false);
      }

      if (liftUp) {
        leverSpeed = 100;
        maxLeverAngle = 135;
      }
      else {
        leverSpeed = 65;
        maxLeverAngle = 115;
      }
    }


    if (liftUp) {
      if(Controller1.ButtonR2.pressing())
      {
        Wings.set(false);
      }
      else
      { 
        Wings.set(true);
      }
    } 
    else Wings.set(false);
  }

  return 0;
}

double screenheading;
int STask(void)
{
  screenheading = Gyro.heading(degrees);
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(monoM);
  Brain.Screen.setPenColor("#808080");
  Brain.Screen.setCursor(3,10);
  Brain.Screen.print("HEADING:");
  Brain.Screen.setCursor(4,10);
  Brain.Screen.print(screenheading);

  return 0;
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  EXIT=true;//Force Exit Autosel once drivercontrol began.
  // User control code here, inside the loop
  while (1) {
    
    // flappybird();
    task Dtask=task(DriveTask);
    task Atask=task(ATask);
    task Ptask=task(PTask);
    // task Stask=task(STask);

    // confirmed = true;
    // confirmed2 = true;

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.


int main() {
  
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
  