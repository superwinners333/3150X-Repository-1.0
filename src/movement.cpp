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
  CDS.Avg=(CDS.Left+CDS.Right)/2;

  // odom stuff
  double delta = CDS.Avg - prevAvg;
  prevAvg = CDS.Avg;

  double headingRad = CDS.HDG * M_PI / 180.0;

  globalX += delta * cos(headingRad);
  globalY += delta * sin(headingRad);

  CDS.X = globalX;
  CDS.Y = globalY;
  // odom stuff ended

  CDS.Diff=CDS.Left-CDS.Right;
  CDS.HDG=Gyro.heading(degrees);

  return CDS;
}

ChassisDataSet ChassisUpdate2()
{
  ChassisDataSet CDS;

  CDS.Left  = get_dist_travelled((LF.position(degrees)+LM.position(degrees)+LB.position(degrees))/3.0);
  CDS.Right = get_dist_travelled((RF.position(degrees)+RM.position(degrees)+RB.position(degrees))/3.0);
  CDS.Avg   = (CDS.Left + CDS.Right) / 2;

  // FIRST: get heading from gyro
  CDS.HDG = Gyro.heading(degrees);

  // odom stuff
  double delta = CDS.Avg - prevAvg;
  prevAvg = CDS.Avg;

  double headingRad = -CDS.HDG * M_PI / 180.0;

  globalX += delta * cos(headingRad);
  globalY += delta * sin(headingRad);

  CDS.X = globalX;
  CDS.Y = globalY;

  CDS.Diff = CDS.Left - CDS.Right;

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
 * the robot forward. Positive value moves it backward. (Ik it's fucked up)
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

  while(fabs(SensorVals.Avg) <= fabs(dist))
  {
    //std::cout << SensorVals.Avg << " " << dist << std::endl;
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
 * the robot forward. Positive value moves it backward. (Ik it's fucked up)
 * @param KVals the PID constants
 * @param DeltaAngle the absolute heading to turn to
 * @param TE time to calculate turn (not time to turn)
 * @param brake Brake at end, or coast
 */
void TurnMaxTimePID(PIDDataSet KVals,double DeltaAngle,double TE, bool brake){
  double CSpeed=0;
  Zeroing(true,false);
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
  Zeroing(true,false);
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


void MoveTimePID(PIDDataSet KVals, int Speed, double TE,double AccT,double ABSHDG, bool brake){
  double CSpeed=0;
  Zeroing(true,false);
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

void MovePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, int Speed, double extraTime, double ABSHDG, bool brake)
{
  double CSpeed=0;
  Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double P_heading=0; // pid values
  double I_heading=0;
  double D_heading=0;
  double P_dist=0;
  double I_dist=0;
  double D_dist=0;
  double E_heading=0; // heading error
  double E_dist=0; // distance error
  double PrevE_heading=0; // previous errors
  double PrevE_dist=0;

  double Correction=0;
  double DPS = (600*wheelToMotorRatio*wheelDiam*M_PI)/60; // distance per sec (inches)

  Brain.Timer.reset();
  
  while (Brain.Timer.value() <= ((dist/DPS)+0.1+extraTime)) {
    SensorVals=ChassisUpdate();
    E_dist = dist-SensorVals.Avg;
  }


  if(brake){BStop(); // braking logic
  wait(200,msec);}
  else CStop();
}



/** Distance PID
 *  NEGATIVE speed = forward
 *  POSITIVE speed = backward
 *
 * @param DistK   PID constants for distance
 * @param HeadK   PID constants for heading
 * @param dist    distance to travel (always positive inches)
 * @param dir     1 = forward, -1 = backward
 * @param MaxSpd  max speed magnitude (0–100)
 * @param AccT    ramp time to max speed (seconds)
 * @param ABSHDG  absolute heading to hold
 * @param brake   true = brake, false = coast
 */

// void MoveDistancePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, int dir, int MaxSpd, double AccT, double ABSHDG, bool brake)
// {
//   // dir: 1 = forward, -1 = backward
//   if(dir != 1 && dir != -1) dir = 1;

//   Zeroing(true, false);
//   ChassisDataSet SensorVals = ChassisUpdate();

//   // Encoder: forward = positive
//   // Motor: forward = negative
//   double targetDist = (dir == 1 ? fabs(dist) : -fabs(dist));

//   double CSpeed = 0;
//   double D_P = 0, D_I = 0, D_D = 0;
//   double H_P = 0, H_I = 0, H_D = 0;

//   double prevDistErr = 0;
//   double prevHeadErr = 0;

//   // --- OVERSHOOT DETECTION ---
//   bool overshot = false;
//   int lastSign = 0;

//   while(true)
//   {
//     SensorVals = ChassisUpdate();
//     double currDist = SensorVals.Avg;

//     // --- DISTANCE PID ---
//     double distErr = targetDist - currDist;

//     // distance deadband
//     if (fabs(distErr) < 0.3) {
//       distErr = 0;
//     }

//     // --- OVERSHOOT LOGIC ---
//     int signNow = (distErr > 0) - (distErr < 0);  // +1, -1, or 0

//     // Detect first overshoot
//     if (lastSign != 0 && signNow != 0 && signNow != lastSign) {
//         overshot = true;
//     }

//     // If already overshot, do NOT allow crossing back
//     if (overshot && signNow != lastSign && signNow != 0) {
//         Move(0, 0);
//         break;
//     }

//     lastSign = signNow;

//     // --- PID CALC ---
//     D_P = DistK.kp * distErr;
//     D_I += DistK.ki * distErr * 0.02;
//     D_D = DistK.kd * (distErr - prevDistErr);

//     double distOut = D_P + D_I + D_D / 0.02;

//     // Clamp
//     if(distOut > MaxSpd) distOut = MaxSpd;
//     if(distOut < -MaxSpd) distOut = -MaxSpd;

//     // --- RAMPING ---
//     double step = (double)MaxSpd / (AccT * 0.5) * 0.02;  // 2x faster ramp
//     if(CSpeed < distOut) {
//       CSpeed += step;
//       if(CSpeed > distOut) CSpeed = distOut;
//     } else if(CSpeed > distOut) {
//       CSpeed -= step;
//       if(CSpeed < distOut) CSpeed = distOut;
//     }

//     // --- HEADING PID ---
//     double LGV = SensorVals.HDG - ABSHDG;
//     if(LGV > 180) LGV -= 360;
//     if(LGV < -180) LGV += 360;

//     H_P = HeadK.kp * LGV;
//     H_I += HeadK.ki * LGV * 0.02;
//     H_D = HeadK.kd * (LGV - prevHeadErr);

//     double Correction = H_P + H_I + H_D / 0.02;

//     // --- APPLY DRIVE ---
//     double motorCmd = -CSpeed;  // negative = forward
//     Move(motorCmd + Correction, motorCmd - Correction);

//     prevDistErr = distErr;
//     prevHeadErr = LGV;

//     // --- EXIT CONDITION ---
//     if (fabs(distErr) < 0.5 && fabs(CSpeed) < 10) {
//       Move(0, 0);
//       break;
//     }

//     wait(20, msec);
//   }

//   if(brake){
//     BStop();
//     wait(120, msec);
//   } else {
//     CStop();
//   }
// }


 /*
void MoveDistancePID(PIDDataSet DistK, PIDDataSet HeadK,
                     double dist, int dir,
                     int MaxSpd, double AccT,
                     double ABSHDG, bool brake)
{
  // dir: 1 = forward, -1 = backward
  if(dir != 1 && dir != -1) dir = 1;

  Zeroing(true, false);
  ChassisDataSet SensorVals = ChassisUpdate();

  // Encoder: forward = positive
  // Motor: forward = negative
  //
  // So target distance must match encoder sign:
  double targetDist = (dir == 1 ? fabs(dist) : -fabs(dist));

  double CSpeed = 0;
  double D_P = 0, D_I = 0, D_D = 0;
  double H_P = 0, H_I = 0, H_D = 0;

  double prevDistErr = 0;
  double prevHeadErr = 0;

  while(true)
  {
    SensorVals = ChassisUpdate();
    double currDist = SensorVals.Avg;

    // --- DISTANCE PID ---
    double distErr = targetDist - currDist;

    // distance deadband: treat tiny error as zero
    if (fabs(distErr) < 0.3) {
      distErr = 0;
    }
    

    D_P = DistK.kp * distErr;
    D_I += DistK.ki * distErr * 0.02;
    D_D = DistK.kd * (distErr - prevDistErr);

    double distOut = D_P + D_I + D_D / 0.02;

    // Clamp
    if(distOut > MaxSpd) distOut = MaxSpd;
    if(distOut < -MaxSpd) distOut = -MaxSpd;

    // --- RAMPING ---
    double step = (double)MaxSpd / (AccT * 0.5) * 0.02;  // 2x faster ramp
    // double step = (double)MaxSpd / AccT * 0.02;
    if(CSpeed < distOut) {
      CSpeed += step;
      if(CSpeed > distOut) CSpeed = distOut;
    } else if(CSpeed > distOut) {
      CSpeed -= step;
      if(CSpeed < distOut) CSpeed = distOut;
    }

    // --- HEADING PID ---
    double LGV = SensorVals.HDG - ABSHDG;
    if(LGV > 180) LGV -= 360;
    if(LGV < -180) LGV += 360;

    H_P = HeadK.kp * LGV;
    H_I += HeadK.ki * LGV * 0.02;
    H_D = HeadK.kd * (LGV - prevHeadErr);

    double Correction = H_P + H_I + H_D / 0.02;

    // --- APPLY DRIVE ---
    // INVERSION FIX:
    // PID output: positive = forward
    // Move(): negative = forward
    double motorCmd = -CSpeed;

    Move(motorCmd + Correction, motorCmd - Correction);

    prevDistErr = distErr;
    prevHeadErr = LGV;

    // --- EXIT CONDITION ---
    if (fabs(distErr) < 0.5 && fabs(CSpeed) < 10) {
      CSpeed = 0;  // force zero command
      Move(0, 0);
      break;
    }

    wait(20, msec);
  }

  if(brake){
    BStop();
    wait(120, msec);
  } else {
    CStop();
  }
}
*/



// ODOM =======================================================

// double globalX = 0;
// double globalY = 0;
// double prevAvg = 0;

double dist(Point a, Point b) {
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

Point findLookahead(Point robot, std::vector<Point> path, double L) {
  Point best = path.back();
  double bestDist = 1e9;

  for (int i = 0; i < path.size() - 1; i++) {
    Point p1 = path[i];
    Point p2 = path[i+1];

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    double fx = p1.x - robot.x;
    double fy = p1.y - robot.y;

    double a = dx*dx + dy*dy;
    double b = 2*(fx*dx + fy*dy);
    double c = fx*fx + fy*fy - L*L;

    double disc = b*b - 4*a*c;
    if (disc < 0) continue;

    disc = sqrt(disc);

    double t1 = (-b + disc) / (2*a);
    double t2 = (-b - disc) / (2*a);

    auto check = [&](double t) {
      if (t >= 0 && t <= 1) {
        Point hit = { p1.x + t*dx, p1.y + t*dy };
        double d = dist(robot, hit);
        if (d < bestDist) {
          bestDist = d;
          best = hit;
        }
      }
    };

    check(t1);
    check(t2);
  }

  return best;
}
/*
void PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)
{
    // Reset odometry (LOCAL MODE)
    globalX = 0;
    globalY = 0;
    prevAvg = 0;
    Zeroing(true, false);

    double dt = 0.02;
    double I = 0;
    double prevErr = 0;

    while (true)
    {
        ChassisDataSet S = ChassisUpdate2();
        Point robot = { S.X, S.Y };

        // End condition
        if (dist(robot, path.back()) < 2.0)
            break;

        // Lookahead point
        Point target = findLookahead(robot, path, lookahead);

        // Desired heading (math CCW)
        double desired = atan2(target.y - robot.y, target.x - robot.x) * 180.0 / M_PI;

        // Reverse mode: flip heading
        if (reverse)
            desired -= 180.0;

        // Normalize desired
        while (desired > 180) desired -= 360;
        while (desired < -180) desired += 360;

        // ----------------------------------------------------
        // FIX #1: Your gyro increases CLOCKWISE.
        // Pure Pursuit expects COUNTER-CLOCKWISE.
        // So heading error must be: err = desired + S.HDG
        // ----------------------------------------------------
        double err = desired + S.HDG;

        // Normalize error
        if (err > 180) err -= 360;
        if (err < -180) err += 360;

        // PID
        double P = KTurn.kp * err;
        I += KTurn.ki * err * dt;
        double D = KTurn.kd * (err - prevErr) / dt;
        prevErr = err;

        // ----------------------------------------------------
        // FIX #2: Your robot turns LEFT when turn > 0
        // This matches Pure Pursuit, so no inversion needed.
        // ----------------------------------------------------
        double turn = P + I + D;

        // ----------------------------------------------------
        // FIX #3: Forward/backward for YOUR robot
        // Positive motor = backward
        // Negative motor = forward
        // ----------------------------------------------------
        double forward = -maxSpeed;   // negative = forward

        if (reverse)
            forward = +maxSpeed;      // positive = backward

        // Slow down when turning sharply
        forward *= (1.0 - fabs(turn) / 100.0);

        // Minimum speed clamp
        if (fabs(forward) < 10)
            forward = (reverse ? +10 : -10);

        // ----------------------------------------------------
        // FIX #4: CORRECT MOTOR MIXING FOR YOUR ROBOT
        // turn > 0 = LEFT
        // left motor must go MORE NEGATIVE (more forward)
        // ----------------------------------------------------
        double left  = forward - turn;
        double right = forward + turn;

        Move(left, right);

        wait(20, msec);
    }

    if (brake)
        BStop();
    else
        CStop();
}
*/

void PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)
{
    // Reset odometry (LOCAL MODE)
    globalX = 0;
    globalY = 0;
    prevAvg = 0;
    Zeroing(true, false);

    double dt = 0.02;
    double I = 0;
    double prevErr = 0;

    while (true)
    {
        ChassisDataSet S = ChassisUpdate2();
        Point robot = { S.X, S.Y };

        // Distance to end
        double d = dist(robot, path.back());

        // ----------------------------------------------------
        // DYNAMIC LOOKAHEAD (prevents spinning at the end)
        // ----------------------------------------------------
        double L = lookahead;

        if (d > 24)
            L = lookahead * 1.5;      // larger lookahead far away

        if (d < 18)
            L = std::max(12.0, d * 0.8);  // shrink but never below 12

        // Lookahead point using dynamic L
        Point target = findLookahead(robot, path, L);

        // Desired heading
        double desired = atan2(target.y - robot.y, target.x - robot.x) * 180.0 / M_PI;

        if (reverse)
            desired -= 180.0;

        while (desired > 180) desired -= 360;
        while (desired < -180) desired += 360;

        // Correct heading error for your gyro
        double err = desired + S.HDG;

        if (err > 180) err -= 360;
        if (err < -180) err += 360;

        double headingErrAbs = fabs(err);

        // ----------------------------------------------------
        // IMPROVED STOP CONDITION (kills end jerk)
        // ----------------------------------------------------
        if (d < 2.0 && headingErrAbs < 5.0)
            break;

        // PID
        double P = KTurn.kp * err;
        I += KTurn.ki * err * dt;
        double D = KTurn.kd * (err - prevErr) / dt;
        prevErr = err;

        double turn = P + I + D;

        // ----------------------------------------------------
        // TURN FADE-OUT NEAR END (smooth finish)
        // ----------------------------------------------------
        double turnScale = 1.0;
        if (d < 12.0) {
            turnScale = d / 12.0;
            if (turnScale < 0.2)
                turnScale = 0.2;
        }
        turn *= turnScale;
        // Limit max turn so it can't go crazy
        if (turn > 25)  turn = 25;
        if (turn < -25) turn = -25;

        // ----------------------------------------------------
        // TURN DEADBAND (kills tiny twitching)
        // ----------------------------------------------------
        if (fabs(turn) < 2.0)
            turn = 0;

        // ----------------------------------------------------
        // POSITIVE SPEED = FORWARD
        // NEGATIVE SPEED = BACKWARD
        // Your robot: negative motor = forward
        // ----------------------------------------------------
        double forward = -maxSpeed;
        if (reverse)
            forward = +maxSpeed;

        // Slow down when turning sharply
        //forward *= (1.0 - fabs(turn) / 100.0);

        // Minimum speed clamp
        if (fabs(forward) < 10)
            forward = (reverse ? +10 : -10);

        // Motor mixing
        double left  = forward - turn;
        double right = forward + turn;

        Move(left, right);

        wait(20, msec);
    }

    if (brake)
        BStop();
    else
        CStop();
}