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



// --- ROBOT POSITIONING & ODOMETRY ---
double Old_distance=0;
double odom_left_offset = 0;  
double odom_right_offset = 0; 

void Zeroing(bool dist, bool HDG, bool odom)
{ 
  ChassisDataSet SensorVals;
  
  if(dist){
    if(!odom){
      SensorVals=ChassisUpdate();
      odom_left_offset += (LF.position(degrees)+LM.position(degrees)+LB.position(degrees))/3.0;
      odom_right_offset += (RF.position(degrees)+RM.position(degrees)+RB.position(degrees))/3.0;  
      Old_distance = SensorVals.Avg;
    }
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

  CDS.hor = odomx.position(degrees); 

  CDS.Avg=(CDS.Left+CDS.Right)/2;

  CDS.Diff=CDS.Left-CDS.Right;
  CDS.HDG=Gyro.heading(degrees);

  CDS.Average = CDS.Avg-Old_distance;
  CDS.AbsoluteLeft = odom_left_offset + (LF.position(degrees)+LM.position(degrees)+LB.position(degrees))/3.0;
  CDS.AbsoluteRight = odom_right_offset + (RF.position(degrees)+RM.position(degrees)+RB.position(degrees))/3.0;

  CDS.backD = backSensor.objectDistance(inches) - 4.2; // distance - distance of distance sensor from the edge of the bot
  CDS.leftD = leftSensor.objectDistance(inches) - 0.1;
  CDS.rightD = rightSensor.objectDistance(inches) - 0.00;

  return CDS;
}


void Move(double left, double right)
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

void Move(int left, int right)
{
  Move((double)left, (double)right);
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
  // LeftRoller.setMaxTorque(100,percent);
  // LeftRoller.spin(forward,(double)val/100.0*12,volt);
  // LeftRoller.setBrake(hold);
  // RightRoller.setMaxTorque(100,percent);
  // RightRoller.spin(forward,(double)val/100.0*12,volt);
  // RightRoller.setBrake(hold);
  Roller.setMaxTorque(100,percent);
  Roller.spin(forward,(double)val/100.0*12,volt);
  Roller.setBrake(hold);
}

void RunLever(int val) {
  lever.setMaxTorque(100,percent);
  lever.spin(forward,(double)val/100.0*12,volt);
  lever.setBrake(hold);
}

// default means that its like that when the code is off
// its should be in the high or neutral position as default
void MiddleScore(void)
{
  // LiftUp.set(false);
  // LiftDown.set(true); // set this to false if its neutral default
}
void NeutralScore(void)
{
  // LiftUp.set(false);
  // LiftDown.set(false); // set this to false if its neutral default
}
void HighScore(void)
{
  // LiftUp.set(true);
  // LiftDown.set(false); // set this to true if its neutral default
}

double wrapAngle(double angle){
  while(angle <= -180) angle += 360;
  while(angle > 180) angle -= 360;
  return angle;
}

bool liftUp = false;
void leverLift(bool up) {
  if (up) {
    lift.set(true);
  }
  else {
    lift.set(false);
  }
  liftUp = up;
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
  Zeroing(true,false,true);
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

    Move(CSpeed-Correction,CSpeed+Correction); 
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

    Move(CSpeed-Correction,CSpeed+Correction);
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

    Move(CSpeed-Correction,CSpeed+Correction);
    PrevE=LGV;
    wait(20, msec);
  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}

/** Moves the robot forward or backward. Negative speed moves
 * the robot forward. Positive value moves it backward. 
 * @param KVals the PID constants
 * @param Speed the speed, from -100 to 100
 * @param dist distance travelled, in inches
 * @param AccT time to max speed (s)
 * @param ABSHDG absolute heading of the robot
 * @param brake Brake at end, or coast
 */
void CurveEncoderPID(PIDDataSet KVals, int SpeedL, int SpeedR, double dist,double AccT, double ABSHDG,bool brake) {
  double CSpeedL=0, CSpeedR=0;
  Zeroing(true,false,true);
  ChassisDataSet SensorVals;
  SensorVals = ChassisUpdate();
  double PVal=0, IVal=0, DVal=0, LGV=0;//define local gyro variable.
  PrevE=0;
  double Correction=0;
  bool notatpos = true;

  while(notatpos)
  {
    if(fabs(CSpeedL)<fabs((double)SpeedL)) {
      CSpeedL+=SpeedL/AccT*0.02; // acceleration logic
    }
    if(fabs(CSpeedR)<fabs((double)SpeedR)) {
      CSpeedR+=SpeedR/AccT*0.02;  
    }

    SensorVals=ChassisUpdate(); 

    // LGV=SensorVals.HDG-ABSHDG; // gyro error
    // if(LGV>180) LGV=LGV-360; 
    // PVal=KVals.kp*LGV;
    // IVal=IVal+KVals.ki*LGV*0.02;
    // DVal=KVals.kd*(LGV-PrevE);

    Correction=PVal+IVal+DVal/0.02; 

    Move(CSpeedL-Correction,CSpeedR+Correction); 
    PrevE=LGV;

    wait(20, msec);
    SensorVals = ChassisUpdate();
    if (fabs(SpeedL) < fabs(SpeedR)) notatpos = fabs(SensorVals.Right) <= fabs(dist);
    else if (fabs(SpeedL) > fabs(SpeedR)) notatpos = fabs(SensorVals.Left) <= fabs(dist);
    else notatpos = fabs(SensorVals.Avg) <= fabs(dist);
  }
  if(brake){
    BStop();
    wait(120,msec);
  }
  else CStop();
}




double trackwidth = 10.728346;
void curvePID(PIDDataSet KVals, int Speed, double radius, double dist, double AccT, double ABSHDG, bool brake) {
  double CSpeed=0;
  Zeroing(true,false,true);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0, IVal=0, DVal=0, LGV=0, Correction=0;//define local gyro variable.
  PrevE=0;


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
    wait(100,msec);
  }
  else CStop();
}

// PIDDataSet TestPara={2,0.1,0.295};

/** Moves the robot to a specific point. Negative speed moves backwards
 * the robot forward. Positive value moves it backward. 
 * @param KVals the PID constants (use TestPara={2,0.1,0.295})
 * @param target_x the x coordinate of the target point, in inches
 * @param target_y the y coordinate of the target point, in inches
 * @param max_speed the maximum speed to move at, from -100 to 100
 * @param final_decel_speed the speed to decelerate to when within decel_distance, from 0 to abs(max_speed)
 * @param timeout_ms the maximum time to spend trying to reach the point, in milliseconds
 * @param brake Brake at end, or chain into next movement
 * @param decel_distance the distance from the target at which to start decelerating, in inches (default 8.0)
 * @param Matchload_distance the distance from the target at which to set the scraper to matchload position (default -1, meaning disabled)
 * @param depoly_distance the distance from the target at which to set the scraper to deploy position (default -1, meaning disabled)
 */

void driveToPoint(PIDDataSet KVals, double target_x, double target_y, double max_speed, double final_decel_speed, double timeout_ms, bool brake, double decel_distance, double Matchload_distance, double depoly_distance) {

    Brain.Timer.reset();
    double distance_threshold = 0.1;
    double settling_zone = 1;
    double robot_radius = 2.41;

    double PVal = 0, IVal = 0, DVal = 0;
    double PrevE = 0;
    double Correction = 0;
    
    // Anti-orbit: Track minimum distance to target
    double min_raw_distance = 1000000.0; // Initialize with a very large number

    bool moving_forward = (max_speed >= 0);
    double abs_max_speed = fabs(max_speed); // absolute max speed
    final_decel_speed = fmin(fabs(final_decel_speed), abs_max_speed); // Ensure final decel speed is not greater than max speed
    final_decel_speed = fmax(final_decel_speed, 0); // Ensure final decel speed is not negative

    while (Brain.Timer.value() < timeout_ms) {
        ChassisDataSet SensorVals = ChassisUpdate();

        double dx = target_x - CPos.x;
        double dy = target_y - CPos.y;
        double raw_distance = sqrt(dx*dx + dy*dy);
        
        // Anti-orbit exit condition:
        // If we are close (e.g. within 15 cm) and distance starts increasing, exit.
        if (raw_distance < min_raw_distance) {
          min_raw_distance = raw_distance; // update our closest distance
        } else if (raw_distance > min_raw_distance + 0.5 && raw_distance < 7) { 
          std::cout << "Anti-orbit triggered. Exiting drive loop." << std::endl;
          break; // exits loop if we start moving away from the target after getting close
        }

        double speed_factor = 0.05 * abs_max_speed;
        double distance_factor = fmin(raw_distance / 5.0, 1.0);
        double effective_momentum = speed_factor * distance_factor;

        double distance = fmax(raw_distance - robot_radius + effective_momentum, 0.0); 
        if (distance <= distance_threshold) break; 

        double target_angle = radToDeg(atan2(dx, dy));
        if (!moving_forward) target_angle = wrapAngle(target_angle + 180.0); // reverse target angle when moving backwards

        double angle_error = wrapAngle(target_angle - SensorVals.HDG); // error for heading

        if (Matchload_distance >= fabs(distance)) {
          Scrapper.set(false);
        }
        if (depoly_distance >= fabs(distance)) {
          Scrapper.set(true);
        }
        if (distance <= settling_zone) { // If we're close enough, stop correcting heading
            Correction = 0;
            PrevE = 0;
            IVal = 0;
        } else { // PID calculations for heading correction
            PVal = KVals.kp * angle_error;
            IVal= IVal+KVals.ki*angle_error*0.01;
            DVal = KVals.kd * wrapAngle(angle_error - PrevE);
            PrevE = angle_error;
            Correction = PVal + IVal + DVal/0.01;
        }


        double ratio = raw_distance / decel_distance; // ratio of current distance to deceleration distance
        ratio = fmin(fmax(ratio, 0.0), 1.0); // When outside decel_distance, ratio is 1 (full speed). When at target, ratio is 0 (final_decel_speed).

        double base_speed = final_decel_speed + ((abs_max_speed - final_decel_speed) * ratio); // linear deceleration

        double min_move_speed = final_decel_speed + 5;
        if (base_speed < min_move_speed) base_speed = min_move_speed; // ensure we don't go too slow until we're very close

        double left_correction_term = moving_forward ? Correction : -Correction; // invert correction when moving backwards
        double right_correction_term = moving_forward ? -Correction : Correction; // opposite correction for right side

        double left_speed = base_speed + left_correction_term; // add correction to base speed (left side)
        double right_speed = base_speed + right_correction_term; // add correction to base speed (right side)

        if (!moving_forward) { // invert speeds when moving backwards
          left_speed *= -1.0;
          right_speed *= -1.0;
        }

        double max_req = fmax(fabs(left_speed), fabs(right_speed)); // find the maximum required speed
        if (max_req > abs_max_speed) { // if the required speed exceeds max, scale both speeds down proportionally
          left_speed  = (left_speed / max_req) * abs_max_speed;
          right_speed = (right_speed / max_req) * abs_max_speed;
        }

        Move(left_speed, right_speed);
        wait(10, msec);
    }
    IVal = 0;
    if (brake) {
        BStop();
        wait(100, msec);
    }
    // When chaining (brake=false), don't stop motors — next function takes over
}
