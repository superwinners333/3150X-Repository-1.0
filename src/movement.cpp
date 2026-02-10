#include "vex.h"

#include "math.h"
#include "screen_gui.hpp"
#include "helper_functions.hpp"
#include "movement.hpp"
#include "odom.hpp"

#include <iostream>
#include <vector>
#include <math.h>

using namespace vex;

int turninverse=-1;//change this to -1 if turning is inversed

int JB;
int PB;
int PX;
int JX;

//General Sect;
//This section includes all general codes for drive and auto



bool dist_reset = false;
/** Resets the robot's drive train and inertial sensor
 * 
 * @param dist the distance encoder reset
 * @param HDG the gyro heading reset
 */
void Zeroing(bool dist, bool HDG)
{
  if(dist){
  dist_reset = true;
  LF.resetPosition();
  LM.resetPosition();
  LB.resetPosition();
  RF.resetPosition();
  RM.resetPosition();
  RB.resetPosition();
  }
  if(HDG){
    Gyro.setHeading(0,degrees);
  }
}

// global odom variables
double globalX = 0;
double globalY = 0;
double prevAvg = 0;


ChassisDataSet ChassisUpdate()
{
  ChassisDataSet CDS;
  CDS.Left=get_dist_travelled((LF.position(degrees)+LM.position(degrees)+LB.position(degrees))/3.0);
  CDS.Right=get_dist_travelled((RF.position(degrees)+RM.position(degrees)+RB.position(degrees))/3.0);
  // std::cout << get_dist_travelled(LF.position(degrees)) << std::endl;
  // std::cout << get_dist_travelled(LM.position(degrees)) << std::endl;
  // std::cout << get_dist_travelled(LB.position(degrees)) << std::endl;
  // std::cout << get_dist_travelled(RF.position(degrees)) << std::endl;
  // std::cout << get_dist_travelled(RM.position(degrees)) << std::endl;
  // std::cout << get_dist_travelled(RB.position(degrees)) << std::endl;
  // std::cout << -1 << std::endl;

  CDS.Avg=(CDS.Left+CDS.Right)/2;

  CDS.Diff=CDS.Left-CDS.Right;
  CDS.HDG=Gyro.heading(degrees);

  CDS.backD = backSensor.objectDistance(inches) - 4.2; // distance - distance of distance sensor from the edge of the bot
  CDS.leftD = leftSensor.objectDistance(inches) - 0.1;
  CDS.rightD = rightSensor.objectDistance(inches) - 0.00;

  return CDS;
}


void Move(int left, int right)
{
  LF.setMaxTorque(100,percent);
  LM.setMaxTorque(100,percent);
  LB.setMaxTorque(100,percent);
  RF.setMaxTorque(100,percent);
  RM.setMaxTorque(100,percent);
  RB.setMaxTorque(100,percent);

  LF.spin(forward,(double)left/100.0*11,volt);
  LM.spin(forward,(double)left/100.0*11,volt);
  LB.spin(forward,(double)left/100.0*11,volt);
  RF.spin(forward,(double)right/100.0*11,volt);
  RM.spin(forward,(double)right/100.0*11,volt);
  RB.spin(forward,(double)right/100.0*11,volt);
}

void BStop()
{
  LF.setStopping(brake);
  LM.setStopping(brake);
  LB.setStopping(brake);
  RF.setStopping(brake);
  RM.setStopping(brake);
  RB.setStopping(brake);

  LF.stop();
  LM.stop();
  LB.stop();
  RF.stop();
  RM.stop();
  RB.stop();
}

void CStop()
{
  LF.setStopping(coast);
  LM.setStopping(coast);
  LB.setStopping(coast);
  RF.setStopping(coast);
  RM.setStopping(coast);
  RB.setStopping(coast);

  LF.stop();
  LM.stop();
  LB.stop();
  RF.stop();
  RM.stop();
  RB.stop();
}




void RunIndex(int val)
{
  LeftRoller.setMaxTorque(100,percent);
  LeftRoller.spin(forward,(double)val/100.0*12,volt);
  LeftRoller.setBrake(hold);
  RightRoller.setMaxTorque(100,percent);
  RightRoller.spin(forward,(double)val/100.0*12,volt);
  RightRoller.setBrake(hold);
}

// default means that its like that when the code is off
// its should be in the high or neutral position as default
void MiddleScore(void)
{
  LiftUp.set(false);
  LiftDown.set(true); // set this to false if its neutral default
}
void NeutralScore(void)
{
  LiftUp.set(false);
  LiftDown.set(false); // set this to false if its neutral default
}
void HighScore(void)
{
  LiftUp.set(true);
  LiftDown.set(false); // set this to true if its neutral default
}

int PrevE;//Error at t-1

/** Moves the robot forward or backward. Negative speed moves
 * the robot forward. Positive value moves it backward. 
 * @param KVals the PID constants
 * @param Speed the speed, from -100 to 100
 * @param dist distance travelled, in inches
 * @param AccT time to max speed (s)
 * @param ABSHDG absolute heading of the robot
 * @param brake Brake at end, or coast
 */
void MoveEncoderPID(PIDDataSet KVals, int Speed, double dist,double AccT, double ABSHDG,bool brake){
  double CSpeed=0;
  Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  double LGV=0;//define local gyro variable.
  PrevE=0;
  double Correction=0;
  // Brain.Screen.clearScreen();
  // double startAvg = SensorVals.Avg;
  // double dist_moved = SensorVals.Avg - startAvg;
  while(fabs(SensorVals.Avg) <= fabs(dist))
  {
    if(fabs(CSpeed)<fabs((double)Speed))
    {
      CSpeed+=Speed/AccT*0.02; // acceleration logic
    }

    SensorVals=ChassisUpdate(); 
    // dist_moved = SensorVals.Avg - startAvg;

    LGV=SensorVals.HDG-ABSHDG; // gyro error
    if(LGV>180) LGV=LGV-360; 
    PVal=KVals.kp*LGV;
    IVal=IVal+KVals.ki*LGV*0.02;
    DVal=KVals.kd*(LGV-PrevE);

    Correction=PVal+IVal+DVal/0.02; 

    Move(CSpeed+Correction,CSpeed-Correction); 
    PrevE=LGV;
    wait(20, msec);
  }
  if(brake){
    BStop();
    wait(120,msec);
  }
  else CStop();
}

/** Moves the robot forward or backward. Negative speed moves
 * the robot forward. Positive value moves it backward.
 * @param KVals the PID constants
 * @param DeltaAngle the absolute heading to turn to
 * @param TE time to calculate turn (not time to turn)
 * @param brake Brake at end, or coast
 */
void TurnMaxTimePID(PIDDataSet KVals,double DeltaAngle,double TE, bool brake){
  double CSpeed=0;
  // Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0; 
  double LGV=0;
  PrevE=0;
  double Correction=0;
  Brain.Timer.reset();

  while(Brain.Timer.value() <= TE)
  {
    SensorVals=ChassisUpdate();
    LGV=SensorVals.HDG-DeltaAngle;
    if(LGV>180) LGV=LGV-360;
    PVal=KVals.kp*LGV;
    IVal=IVal+KVals.ki*LGV*0.02;
    DVal=KVals.kd*(LGV-PrevE);

    Correction=PVal+IVal+DVal/0.02;

    Move(CSpeed+Correction,CSpeed-Correction);
    PrevE=LGV;
    wait(20, msec);
  }

  if(brake){BStop();
  wait(180,msec);}
  else CStop();
}

void MaxTimePIDTurnOneSide(PIDDataSet KVals,double DeltaAngle,double TE, bool brake){
  double CSpeed=0;
  // Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  double LGV=0;
  PrevE=0;
  double Correction=0;
  double LV,RV;
  Brain.Timer.reset();

  while(Brain.Timer.value() <= TE)
  {
  SensorVals=ChassisUpdate();
  LGV=SensorVals.HDG-DeltaAngle;
  if(LGV>180) LGV=LGV-360;
  PVal=KVals.kp*LGV;
  IVal=IVal+KVals.ki*LGV*0.02;
  DVal=KVals.kd*(LGV-PrevE);

  Correction=PVal+IVal+DVal/0.02;
LV=-CSpeed-Correction;
RV=-CSpeed+Correction;
if(LV>=0)LV=0;
if(RV>=0)RV=0;
  Move(LV,RV);
  PrevE=LGV;
  wait(20, msec);
  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}


void MoveTimePID(PIDDataSet KVals, int Speed, double TE,double AccT,double ABSHDG, bool brake) {
  double CSpeed=0;
  // Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  double LGV=0;
  PrevE=0;
  double Correction=0;
  Brain.Timer.reset();


  while(Brain.Timer.value() <= TE)
  {
    if(fabs(CSpeed)<fabs((double)Speed))
    {
      CSpeed+=Speed/AccT*0.02;
    }

    SensorVals=ChassisUpdate();
    LGV=SensorVals.HDG-ABSHDG;
    if(LGV>180) LGV=LGV-360;
    PVal=KVals.kp*LGV;
    IVal=IVal+KVals.ki*LGV*0.02;
    DVal=KVals.kd*(LGV-PrevE);

    Correction=PVal+IVal+DVal/0.02;

    Move(-CSpeed+Correction,-CSpeed-Correction);
    PrevE=LGV;
    wait(20, msec);
  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}


void curvePID(PIDDataSet KVals, int Speed, double radius, double dist, double AccT, double ABSHDG, bool brake) {
  double CSpeed=0;
  // Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  double LGV=0;
  PrevE=0;
  double Correction=0;
  Brain.Timer.reset();

  bool atTarget = false;

  while(atTarget = false && Brain.Timer.value() < 5.0)
  {
    if(fabs(CSpeed)<fabs((double)Speed))
    {
      CSpeed+=Speed/AccT*0.02;
    }

    SensorVals=ChassisUpdate();
    LGV=SensorVals.HDG-ABSHDG;
    if(LGV>180) LGV=LGV-360;
    PVal=KVals.kp*LGV;
    IVal=IVal+KVals.ki*LGV*0.02;
    DVal=KVals.kd*(LGV-PrevE);

    Correction=PVal+IVal+DVal/0.02;

    Move(-CSpeed+Correction,-CSpeed-Correction);
    PrevE=LGV;
    wait(20, msec);
  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}






