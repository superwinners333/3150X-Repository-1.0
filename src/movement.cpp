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

  double startL = SensorVals.Left;
  double startR = SensorVals.Right;
  double movedL = SensorVals.Left - startL;
  double movedR = SensorVals.Right - startR;
  double movedAvg = (movedL + movedR) / 2.0;
  while(fabs(SensorVals.Avg) <= fabs(dist))
  {
    movedL = SensorVals.Left - startL;
    movedR = SensorVals.Right - startR;
    movedAvg = (movedL + movedR) / 2.0;
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

double trackwidth = 10.728346;
void curvePID(PIDDataSet KVals, int Speed, double radius, double dist, double AccT, double ABSHDG, bool brake) {
  double CSpeed=0;
  Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0, IVal=0, DVal=0, LGV=0, Correction=0;//define local gyro variable.
  PrevE=0;

  double startL = SensorVals.Left;
  double startR = SensorVals.Right;
  double movedL = SensorVals.Left - startL;
  double movedR = SensorVals.Right - startR;
  double movedAvg = (movedL + movedR) / 2.0;
  while(fabs(SensorVals.Avg) <= fabs(dist))
  {
    movedL = SensorVals.Left - startL;
    movedR = SensorVals.Right - startR;
    movedAvg = (movedL + movedR) / 2.0;
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
    wait(100,msec);
  }
  else CStop();
}


/*
void AccuratePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, double maxAccel, int Speed, double timeout, double ABSHDG, bool brake)
{
  // Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double outputSpeed = 0.0;

  double P_heading=0.0, I_heading=0.0, D_heading=0.0, E_heading=0.0, PrevE_heading=0.0; // heading PID variables
  double P_dist=0.0, I_dist=0.0, D_dist=0.0, E_dist=0.0, PrevE_dist=0.0; // distance PID variables

  double rawSpeed=0.0, Correction=0.0; // motor input variables
  double dt = 20.0; // how often the PID error checks loops in ms

  Brain.Timer.reset(); // resets brain timer for exit

  int settledTime = 200; // exit variable

  // TUNEABLE EXIT VARIABLES
  double exitError = 1.0;        // how close to target before stopping
  double exitDerivative = 2.0;   // how still the robot must be
  int exitTime = 150;              // ms required to be stable
  // int timeout = 0;               // ms max runtime

  if (timeout <= 0) timeout = 5.0; // creates a default exit time 
  bool settled = false, timedOut = false;

  double speedSign = Speed / (fabs(Speed));

  double startAvg = SensorVals.Avg;
  double startL = SensorVals.Left; // gets starting left side encoder movement
  double startR = SensorVals.Right; // gets starting right side encoder movement

  // ---------------------------------------------------------------
  // ------------------------ main PID loop ------------------------
  // ---------------------------------------------------------------
  while (!settled && !timedOut) {
    SensorVals = ChassisUpdate(); // gets drivetrain values
    
    double left_moved = SensorVals.Left - startL; // gets distance that left side moved
    double right_moved = SensorVals.Right - startR; // gets distance that right side moved

    double dist_moved = (left_moved + right_moved) / 2.0; // averages left moved and right moved
    double percent_dist = (dist_moved / dist) * 100.0;

    // distance PID calculations
    E_dist = -(100.0 - percent_dist);
    P_dist = DistK.kp*E_dist;
    I_dist += (DistK.ki+(dist/1000.0))*E_dist*(dt/1000.0);
    D_dist = (DistK.kd*(E_dist-PrevE_dist)) / (dt/1000.0);

    // std::cout << D_dist << std::endl;

    rawSpeed = P_dist + I_dist + D_dist + ((dist/4.3) * (E_dist / fabs(E_dist))); // output speed

    if (Speed < 0) rawSpeed = rawSpeed * -1.0;
    if (fabs(rawSpeed) > fabs(Speed)) { // clamps outputSpeed at max speed
      if (Speed >= 0 && rawSpeed >= 0) rawSpeed = Speed;
      else if (Speed < 0 && rawSpeed < 0) rawSpeed = Speed;
    }

    double speedChange = rawSpeed - outputSpeed; // gets change in speed
    if (speedChange > maxAccel) speedChange = maxAccel;
    if (speedChange < -maxAccel) speedChange = -maxAccel;
    outputSpeed += speedChange; // accelerates


    // heading PID adjustment calculations
    E_heading = SensorVals.HDG-ABSHDG;
    if (E_heading > 180) E_heading -= 360;
    else if (E_heading < -180) E_heading += 360;
    P_heading = HeadK.kp*E_heading;
    I_heading += HeadK.ki*E_heading*(dt/1000.0);
    D_heading = (HeadK.kd*(E_heading-PrevE_heading)) / (dt/1000.0);

    Correction = P_heading + I_heading + D_heading; // correction


    // tells the bot how much to run each side
    Move(outputSpeed+Correction,outputSpeed-Correction);
    
    PrevE_heading = E_heading; // updates previous headings
    PrevE_dist = E_dist;


    // distance exit conditions calculations
    // (dist_moved+0.95336)/(1.01514)
    bool errorSmall = fabs(dist - dist_moved) <= exitError;
    bool derivativeSmall = fabs(D_dist) <= exitDerivative;

    if (errorSmall && derivativeSmall) settledTime += dt; // if we are close to the target, start counting
    else settledTime = 0; // if we start adjusting again, stop counting


    // exit condition checks
    settled = settledTime >= exitTime; 
    timedOut = Brain.Timer.value() >= timeout;

    std::cout << "Timer: " << Brain.Timer.value() << std::endl;
    std::cout << "distance: " << dist_moved << std::endl;
    std::cout << "error: " << E_dist << std::endl;
    std::cout << "percent: " << percent_dist << std::endl;
    std::cout << " " << std::endl;

    if (settled || timedOut) break; // if either exit condition is met, exit

    wait(20,msec); // wait to stop constant looping
  }

  std::cout << "exit" << std::endl;
  std::cout << "actual: " << SensorVals.backD <<std::endl;
  if(brake){BStop(); // braking logic
  wait(200,msec);}
  else CStop();
}

*/
