/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//#include "STDLib.cpp"
#include "vex.h"

#include "screen_gui.hpp"
#include "movement.hpp"
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
  Lift.set(false);
  Scrapper.set(false);
  BackDescore.set(false);
  confirmed = false;
  confirmed2 = false;
  PX=0;
  JX=0;
  AutoSelectorVal=0; // 0 = default
  SP=false;
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


Zeroing(true,true);

wait(100,msec); // just a small delay



Brain.Screen.clearScreen();
// drawField(); // draws field
confirmed = false;
confirmed2 = false;

while(!confirmed) // waits for field corner selection
{ 
confirmCorner();
wait(20,msec); // gives a cooldown before looping again
}
Brain.Screen.clearScreen();
greyScreen();

wait(50,msec);
while(!confirmed2) // waits for auto selection
{ 
  AutoSelection();
  wait(20,msec); // gives a small cooldown before looping again
}


AutonLogic();

// Brain.Screen.clearScreen();
// NewField();


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void) {
  
  Brain.Screen.clearScreen();
  //Do not change the below
  PIDDataSet TestPara={4,0.1,0.2};
  Zeroing(true,true);

  //Put Auto route function into if statements to use autoselector
  if(AutoSelectorVal==1)// left 1
  {
    high_basic();
  }

  if(AutoSelectorVal==2)// left 2
  {
    high_long();
  }

  if(AutoSelectorVal==3)// left 3
  {
    
  } 

  if(AutoSelectorVal==4)// left 4
  {
    
  }

  if(AutoSelectorVal==5)// right 1
  {
    low_basic();
  }

  if(AutoSelectorVal==6) // right 2
  {
    low_long();
  }

  if(AutoSelectorVal==7) // right 3
  { 
    solo_awp();
  }

  if(AutoSelectorVal==8) // right 4
  { 
  
  }

  if(AutoSelectorVal==9) // PROGRAMMING SKILLS GO HERE
  { 
  
  }

  CStop(); 
}

int RV;
int LV;
int DriveTask(void){
  while(true)
  {
    EXIT=true;
    RV=-Controller1.Axis3.position(percent)+Controller1.Axis1.position(percent);
    LV=-Controller1.Axis3.position(percent)-Controller1.Axis1.position(percent);
    Move(LV,RV);
  }

return 0;
}
int V;
bool TopIntake = false;
int ATask(void)
{
  double pow;
  double pow2;
    while(true)
  {
    pow=((Controller1.ButtonR2.pressing()-Controller1.ButtonR1.pressing())*100);//Calculate intake power, if button pressed, button.pressing returns 1
    
    if (Controller1.ButtonL2.pressing()==1)
    {
      RunRoller(-100); // outtake
      RunTopRoller(-100);
    }
    else if (Controller1.ButtonR1.pressing()==1)
    {
      RunIndex(100);
    }
    else if (Controller1.ButtonL1.pressing()==1)
    {
      RunRoller(100);
      // RunTopRoller(-20);
    }
    else
    {
      RunIndex(0);
    } 
    // RunRoller(pow);
    // RunTopRoller(pow);


  //RunPuncher((Controller1.ButtonB.pressing())*100);
  }
  
  return 0;
}

int ButtonPressingX,XTaskActiv;
int ButtonPressingY,YTaskActiv;
int ButtonPressingA,ATaskActiv; // Button Down
int ButtonPressingB,BTaskActiv;

int PTask(void)
{
    while(true)
    {
      //Toggles Scrapper
    if(XTaskActiv==0&&Controller1.ButtonX.pressing()&&ButtonPressingX==0)
    {
      ButtonPressingX=1;
      XTaskActiv=1;
      // TopIntake = true;
      Lift.set(true);
    }

    else if(!Controller1.ButtonX.pressing())ButtonPressingX=0;

    else if(XTaskActiv==1&&Controller1.ButtonX.pressing()&&ButtonPressingX==0)
    {
      ButtonPressingX=1;
      XTaskActiv=0;
      // TopIntake = false;
      Lift.set(false);
    }

    //----------------------------------------- SCRAPPER
      //Toggles Scrapper
    // if(YTaskActiv==0&&Controller1.ButtonY.pressing()&&ButtonPressingY==0)
    // {
    //   ButtonPressingY=1;
    //   YTaskActiv=1;
    //   Scrapper.set(true);
    // }
    // else if(!Controller1.ButtonY.pressing())ButtonPressingY=0;
    // else if(YTaskActiv==1&&Controller1.ButtonY.pressing()&&ButtonPressingY==0)
    // {
    //   ButtonPressingY=1;
    //   YTaskActiv=0;
    //   Scrapper.set(false);
    // }

  // -------------------------------------- BACKDESCORE
    // Toggles BackDescore
    if(BTaskActiv==0&&Controller1.ButtonB.pressing()&&ButtonPressingB==0)
    {
      ButtonPressingB=1;
      BTaskActiv=1;
      BackDescore.set(true);
    }
    else if(!Controller1.ButtonB.pressing())ButtonPressingB=0;
    else if(BTaskActiv==1&&Controller1.ButtonB.pressing()&&ButtonPressingB==0)
    {
      ButtonPressingB=1;
      BTaskActiv=0;
      BackDescore.set(false);
    }

  // -------------------------------------- LIFT
    // Toggles Lift
    if(ATaskActiv==0&&Controller1.ButtonDown.pressing()&&ButtonPressingA==0)
    {
      ButtonPressingA=1;
      ATaskActiv=1;
      Scrapper.set(true);
    }
    else if(!Controller1.ButtonDown.pressing())ButtonPressingA=0;
    else if(ATaskActiv==1&&Controller1.ButtonDown.pressing()&&ButtonPressingA==0)
    {
      ButtonPressingA=1;
      ATaskActiv=0;
      Scrapper.set(false);
    }
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
  