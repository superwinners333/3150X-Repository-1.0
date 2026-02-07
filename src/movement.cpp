#include "vex.h"

#include "math.h"
#include "screen_gui.hpp"
#include "helper_functions.hpp"
#include "movement.hpp"

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


/** Resets the robot's drive train and inertial sensor
 * 
 * @param dist the 
 * @param HDG the
 */
void Zeroing(bool dist, bool HDG)
{
  if(dist){
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
  CDS.leftD = leftSensor.objectDistance(inches);
  CDS.rightD = rightSensor.objectDistance(inches);

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



void RunRoller(int val)
{
  //FrontRoller.setMaxTorque(100,percent);
  //FrontRoller.spin(forward,(double)val/100.0*12,volt);
  //FrontRoller.setBrake(hold);
}

void RunTopRoller(int val)
{
  //BackRoller.setMaxTorque(100,percent);
  //BackRoller.spin(forward,(double)val/100.0*12,volt);
  //BackRoller.setBrake(hold);
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






/** function makes the bot travel the specified distance
*   (negative values for speed to make the bot go backwards) 
* @param DistK PID constants for distance
* @param HeadK PID constants for heading
* @param dist distance to travel (inches)
* @param maxAccel max increase of voltage % per loop (every 20 ms)
* @param Speed max speed that the bot can travel. Untested with values other than 100. 
* @param timeout max time the function can run for before giving up
* @param ABSHDG heading relative to the starting position of the bot
* @param brake true for braking, false for coasting
*/
void MovePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, double maxAccel, int Speed, double timeout, double ABSHDG, bool brake)
{
  // Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double outputSpeed = 0.0;

  double P_heading=0.0, I_heading=0.0, D_heading=0.0, E_heading=0.0, PrevE_heading=0.0; // heading PID variables
  double P_dist=0.0, I_dist=0.0, D_dist=0.0, E_dist=0.0, PrevE_dist=0.0; // distance PID variables

  double rawSpeed=0.0, Correction=0.0; // motor input variables
  double dt = 20.0; // how often the PID error checks loops in ms

  double DPS = (600*wheelToMotorRatio*wheelDiam*M_PI)/60; // distance per sec (inches)
  Brain.Timer.reset(); // resets brain timer for exit

  int settledTime = 200; // exit variable

  // TUNEABLE EXIT VARIABLES
  double exitError = 0.2;        // how close to target before stopping
  double exitDerivative = 2.5;   // how still the robot must be
  int exitTime = 150;              // ms required to be stable
  // int timeout = 0;               // ms max runtime

  if (timeout <= 0) timeout = (dist/DPS)*1000*3; // creates a default exit time 
  bool settled = false, timedOut = false;

  double speedSign = Speed / (fabs(Speed));

  double startAvg = SensorVals.Avg;

  // Brain.Timer.value() <= ((dist/DPS)+0.1+extraTime) 

  // ---------------------------------------------------------------
  // ------------------------ main PID loop ------------------------
  // ---------------------------------------------------------------
  while (!settled && !timedOut) {
    SensorVals = ChassisUpdate(); // gets drivetrain values
    
    double dist_moved = SensorVals.Avg - startAvg;

    // distance PID calculations
    E_dist = dist - dist_moved;
    P_dist = DistK.kp*E_dist;
    I_dist += DistK.ki*E_dist*(dt/1000.0);
    D_dist = (DistK.kd*(E_dist-PrevE_dist)) / (dt/1000.0);

    // std::cout << D_dist << std::endl;

    rawSpeed = P_dist + I_dist + D_dist; // output speed
    if (dist <= 8.0) {
      rawSpeed += (E_dist * 3.20); // increases the P value for small distances
    }
    if (Speed < 0) rawSpeed = rawSpeed * -1.0;
    if (abs(rawSpeed) > abs(Speed)) { // clamps outputSpeed at max speed
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
    Move(-outputSpeed+Correction,-outputSpeed-Correction);
    
    PrevE_heading = E_heading; // updates previous headings
    PrevE_dist = E_dist;


    // distance exit conditions calculations
    bool errorSmall = fabs(dist-((dist_moved+0.95336)/(1.01514))) <= exitError;
    if (dist <= 8.0) errorSmall = fabs(dist-(0.768092*(dist_moved)+1.33073)) <= exitError;
    bool derivativeSmall = fabs(D_dist) <= exitDerivative;

    if (errorSmall && derivativeSmall) settledTime += dt; // if we are close to the target, start counting
    else settledTime = 0; // if we start adjusting again, stop counting


    // exit condition checks
    settled = settledTime >= exitTime; 
    timedOut = Brain.Timer.value() >= timeout;

    std::cout << "Timer: " << Brain.Timer.value() << std::endl;
    std::cout << "distance: " << dist_moved << std::endl;
    std::cout << "error: " << E_dist << std::endl;
    // std::cout << "predicted: " << (SensorVals.Avg+0.95336)/(1.01514) << std::endl;
    std::cout << "small: predicted: " << 0.768092*(dist_moved)+1.33073 << std::endl;

    if (settled || timedOut) break; // if either exit condition is met, exit

    wait(20,msec); // wait to stop constant looping
  }

  std::cout << "exit" << std::endl;
  if(brake){BStop(); // braking logic
  wait(200,msec);}
  else CStop();
}









/** function to compare two values
* returns true if compared values are within range
* @param range range to compare within
* @param target item 1 to compare
* @param check item 2 to compare
*/
bool inRangeOf(double range, double target, double check) {
  double upper = target+range;
  double lower = target-range;

  if (check < upper && check > lower) return true;
  else return false;
}



// COORDINATE SYSTEM
const double BDC = 0.0; // distance from edge of the bot to the center
const double LDC = 0.0;
const double RDC = 0.0;
const double FDC = 0.0;

double globalHeading = 0; // should be zero when front of the bot faces the side with the other park

Point CPos; // current position

Point StartingPosition(void) {
  Point SP;
  ChassisDataSet SenVals = ChassisUpdate();

  // distance from wall to center of the bot in the starting pos when front of bot is facing wall
  double xwallConst = 0.0; 
  double ywallConst = 0.0;
  double swallConst = 0.0;

  if (corner == 1 || corner == 4) { // left autos
    if (globalHeading == -90.0) {
      SP.x = xwallConst;
      SP.y = SenVals.leftD + LDC;
    }
    else if (globalHeading == 0.0) {
      SP.x = SenVals.leftD + LDC;
      SP.y = SenVals.backD + BDC;
    }
    else if (globalHeading == 90.0) {
      SP.x = SenVals.backD + BDC;
      SP.y = SenVals.rightD + RDC;
    }
    else if (globalHeading == 180.0 || globalHeading == -180) {
      SP.x = SenVals.rightD + RDC;
      SP.y = ywallConst;
    }
  }
  else if (corner == 2 || corner == 3) { // right autos
    if (globalHeading == -90.0) {
      SP.x = 144.0 - (SenVals.backD + BDC);
      SP.y = SenVals.leftD + LDC;
    }
    else if (globalHeading == 0.0) {
      SP.x = 144.0 - (SenVals.rightD + RDC);
      SP.y = SenVals.backD + BDC;
    }
    else if (globalHeading == 90.0) {
      SP.x = 144.0 - xwallConst;
      SP.y = SenVals.rightD + RDC;
    }
    else if (globalHeading == 180.0 || globalHeading == -180) {
      SP.x = 144.0 - (SenVals.leftD + LDC);
      SP.y = ywallConst;
    }
  }
  else { // skills
    SP.x = ((SenVals.leftD + LDC) + (144.0 - (SenVals.rightD + RDC)))/2.0;
    SP.y = swallConst;
  }
  SP.h = globalHeading;
  return SP;
}

/** function returns distance to specified wall
* @param TH is the trueHeading value of the bot
* @param side is the wall that you want the distance to
* @param EP is the estimated position that we are at
*/
double getWallDist(double TH, double side, Point EP) { 
  ChassisDataSet SenVals = ChassisUpdate();
  // side values are:
  // 1 = left
  // 2 = right
  // 3 = near
  // 4= far
  
  double wallDist = 0.0; // distance from specified wall
  
  if (side == 1) {
    if (inRangeOf(1.5,0.0,TH)) {
      wallDist = SenVals.leftD + LDC;
    }
    else if (inRangeOf(1.5,90.0,TH)) {
      wallDist = 0;
    }
  }

  return wallDist;
}


Point resetPosition(Point EP) {
  Point NP;
  ChassisDataSet SenVals = ChassisUpdate();

  double trueHeading = globalHeading + SenVals.HDG; // gets true heading relative to the field
  if (trueHeading > 180.0) trueHeading -= 360.0;
  else if (trueHeading < -180.0) trueHeading += 360.0;
  NP = EP;
  
  // get coords of NP and then check if they are similar to NP


  return NP;
}



// returns a new position of the bot
/** @param EP encoder predicted position
*/
Point longGoalReset(Point EP) { // resets base on long goal position
  Point LG; // long goal point
  ChassisDataSet SenVals = ChassisUpdate();
  double trueHeading = globalHeading + SenVals.HDG;
  if (trueHeading > 180.0) trueHeading -= 360.0;
  else if (trueHeading < -180.0) trueHeading += 360.0;

  LG = EP; // creates values to output in case the bot thinks our position is too innacurate
  
  // resets x vals
  if (SenVals.leftD <= SenVals.rightD) {
    if (inRangeOf(1.5,180.0,trueHeading)) LG.x = 144.0 - (SenVals.leftD + LDC); // close right
    else if (inRangeOf(1.5,0.0,trueHeading)) LG.x = SenVals.leftD + LDC;
  }
  else if (SenVals.rightD < SenVals.leftD) {
    if (inRangeOf(1.5,180.0,trueHeading)) LG.x = SenVals.rightD + RDC;
    else if (inRangeOf(1.5,0.0,trueHeading)) LG.x = 144.0 - (SenVals.rightD + RDC);
  }

  // resets y vals with constants 
  // the value of the constant should be the distance from the wall to the center of the bot when the bot is scoring on the long goal
  double LGConst = 0.0;
  if (inRangeOf(1.0,180.0,trueHeading)) LG.y = LGConst; 
  else if (inRangeOf(1.0,0.0,trueHeading)) LG.y = 144.0 - LGConst;

  return LG;
}



// only meant for resetting against the left and right walls
Point wallReset(Point EP, bool botBack) { // if back == true, then the back of our bot is facing the wall
  Point NP;
  ChassisDataSet SenVals = ChassisUpdate();

  double trueHeading = globalHeading + SenVals.HDG; // gets true heading relative to the field
  if (trueHeading > 180.0) trueHeading -= 360.0;
  else if (trueHeading < -180.0) trueHeading += 360.0;

  NP = EP;

  if (botBack) { // if back of bot is facing wall
    if (EP.x > 132) NP.x = 144 - (SenVals.backD + BDC);
    else if (EP.x < 12) NP.x = SenVals.backD + BDC;
  }
  else { // if front of bot is facing wall
    if (EP.x > 132) NP.x = 144 - FDC;
    else if (EP.x < 12) NP.x = FDC;
  }

  if (SenVals.leftD <= SenVals.rightD) { // if left side is closer to the wall
    if (inRangeOf(1.0,90.0,trueHeading)) NP.y = 144.0 - (SenVals.leftD + LDC); // close right
    else if (inRangeOf(1.0,-90.0,trueHeading)) NP.y = SenVals.leftD + LDC;
  }
  else if (SenVals.rightD < SenVals.leftD) { // if right side is closer to the wall
    if (inRangeOf(1.0,90.0,trueHeading)) NP.y = SenVals.rightD + RDC;
    else if (inRangeOf(1.0,-90.0,trueHeading)) NP.y = 144.0 - (SenVals.rightD + RDC);
  }

  return NP;
}

Point resetBack(Point EP);
Point resetLeft(Point EP);
Point resetRight(Point EP);

// use negative distance values for moving backwards?



double degToRad(double deg) {
  double rad = deg * M_PI / 180.0;
  return rad;
}

double dist_between_wheels = 10.728346; // distance between the wheels in inches

// odom function
void OdomUpdate(){
  Zeroing(true,true);

  
  double prev_heading_rad = globalHeading;
  double prev_left_in = 0, prev_right_in = 0;
  double d_local_y_in = 0;

  while (true) {
    ChassisDataSet SensorVals = ChassisUpdate();
    double true_heading = globalHeading+SensorVals.HDG;
    double heading_rad = degToRad(true_heading);
    double left_in = SensorVals.Left; // distance travelled in inches
    double right_in = SensorVals.Right;
    double d_heading_rad = heading_rad - prev_heading_rad; // change in heading
    double d_left_in = left_in - prev_left_in; // change in left and right distances
    double d_right_in = right_in - prev_right_in;

    // checks if there is change in heading
    if (fabs(d_heading_rad) < 1e-6) {
      // if no change, assume we moved in a straight line
      d_local_y_in = (d_left_in + d_right_in) / 2.0; // gets distance moved
    }
    else { 
      // calculate movement per wheel
      // gets chord length when multiplied with radius
      double sin_multiplier = 2.0 * sin(d_heading_rad / 2.0); // ratio of radius to chord length
      // gets chord length by multiplying the ratio and radius (coming from arc length)
      // d_left_in / d_heading_rad gives arc length
      // this gives straight line movement
      double d_local_y_left_in = sin_multiplier * (d_left_in / d_heading_rad + dist_between_wheels / 2.0);
      double d_local_y_right_in = sin_multiplier * (d_right_in / d_heading_rad + dist_between_wheels / 2.0);

      d_local_y_in = (d_local_y_left_in + d_local_y_right_in) / 2.0; // averages the distance
    }

    // updates global position using polar coordinates
    double polar_angle_rad = prev_heading_rad + d_heading_rad / 2.0; // polar angle
    double polar_radius_in = d_local_y_in; // polar radius

    CPos.x += polar_radius_in * sin(polar_angle_rad);
    CPos.y += polar_radius_in * cos(polar_angle_rad);

    
    prev_heading_rad = heading_rad; // updates prev values
    prev_left_in = left_in;
    prev_right_in = right_in;

    wait(20,msec);
  }
}


// things to do
/*
* change starting position reset logic
* 
*/







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

  double DPS = (600*wheelToMotorRatio*wheelDiam*M_PI)/60; // distance per sec (inches)
  Brain.Timer.reset(); // resets brain timer for exit

  int settledTime = 200; // exit variable

  // TUNEABLE EXIT VARIABLES
  double exitError = 1.0;        // how close to target before stopping
  double exitDerivative = 2.0;   // how still the robot must be
  int exitTime = 150;              // ms required to be stable
  // int timeout = 0;               // ms max runtime

  if (timeout <= 0) timeout = (dist/DPS)*1000*3; // creates a default exit time 
  bool settled = false, timedOut = false;

  double speedSign = Speed / (fabs(Speed));

  double startAvg = SensorVals.Avg;

  // ---------------------------------------------------------------
  // ------------------------ main PID loop ------------------------
  // ---------------------------------------------------------------
  while (!settled && !timedOut) {
    SensorVals = ChassisUpdate(); // gets drivetrain values
    
    double dist_moved = SensorVals.Avg - startAvg;
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