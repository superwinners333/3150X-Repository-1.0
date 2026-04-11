#include "vex.h"

#include "math.h"
#include "screen_gui.hpp"
#include "helper_functions.hpp"
#include "movement.hpp"
#include "odom.hpp"
#include "Odometry.hpp"
#include "VirtualTargetPursuit.hpp"

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
    Lift.set(false);
  }
  else {
    Lift.set(true);
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
  Zeroing(true,false,false);
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
  // else CStop();
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

timer autolevertime;
void leverFull(int speed) {
  autolevertime.clear();
  int levering = 1;
  bool upwards = true;
  double exittime = 0;
  bool waiting = false;
  int maxLeverAngle;
  int leverSpeed = 100;

  if (liftUp) {
    leverSpeed = 100;
    maxLeverAngle = 135;
  }
  else {
    leverSpeed = 60;
    maxLeverAngle = 115;
  }

  while (levering == 1) {
    if (upwards) {
      if (liftUp) maxLeverAngle = 115;
      else maxLeverAngle = 135;
      RunIndex(100);
      lock.set(true);
      if (levertracker.position(degrees) < maxLeverAngle) RunLever(fabs(speed)); // leverspeed
      else {
        upwards = false;
        waiting = true;
        exittime = autolevertime.value() + 0.3; // time to wait for before exiting
      }
    } 
    else if (waiting) {
      if (autolevertime.value() > exittime) { // pauses to let the lever settle
        waiting = false; // unpauses
      }
    }
    else {
      RunIndex(-100);
      if (levertracker.position(degrees) > 3) RunLever(-100);
      else levering=0;
    }
    if (autolevertime.value() > 1.2) { // time before assuming the lever has stalled and exiting
      levering=0;
      RunLever(0);
      RunIndex(0);
    }
    wait(10,msec);
  }
  RunIndex(0);
  RunLever(0);
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
    
    double min_raw_distance = 1000000.0;
    double max_correction = 1000000.0;

    Point targetpoint = {target_x, target_y};
  
    double initial_dist = pointDist(CPos,targetpoint);

    double closest_approach = 999.0;
    int frames_since_closest = 0;

    bool moving_forward = (max_speed >= 0);
    int add = moving_forward ? 0 : 180;
    double abs_max_speed = fabs(max_speed);
    final_decel_speed = fmin(fabs(final_decel_speed), abs_max_speed);
    final_decel_speed = fmax(final_decel_speed, 0);

    VTPVelocityFilter vx_filter(8);
    VTPVelocityFilter vy_filter(8);

    double prev_x = CPos.x, prev_y = CPos.y;
    double prev_time = Brain.Timer.value();
    int loop_count = 0;

    double prev_h_tracker_deg = odomx.position(degrees);
    double prev_heading = ChassisUpdate().HDG;
    VTPVelocityFilter lat_vel_filter(6);

    // ── Perpendicular line state — NOT static ──
    bool prev_perpendicular_line = true;

    // global_target_x = target_x;
    // global_target_y = target_y;

    while (Brain.Timer.value() < timeout_ms) {
        ChassisDataSet SensorVals = ChassisUpdate();
        double curr_x = CPos.x;
        double curr_y = CPos.y;
        double curr_time = Brain.Timer.value();
        double dt = curr_time - prev_time;
        if (dt < 0.005) dt = 0.005;

        double dx = target_x - curr_x;
        double dy = target_y - curr_y;
        double raw_distance = sqrt(dx*dx + dy*dy);

        double raw_vx = (curr_x - prev_x) / dt;
        double raw_vy = (curr_y - prev_y) / dt;
        if (fabs(raw_vx) > 100.0) raw_vx = 0.0;
        if (fabs(raw_vy) > 100.0) raw_vy = 0.0;
        vx_filter.add(raw_vx);
        vy_filter.add(raw_vy);
        double spd = hypot(vx_filter.get(), vy_filter.get());

        double curr_theta = SensorVals.HDG;
        double omega_deg = wrapAngle(curr_theta - prev_heading) / dt;
        double lateral_vel = 0.0;

        double curr_h_deg = odomx.position(degrees);
        double delta_h_deg = curr_h_deg - prev_h_tracker_deg;
        double delta_h_inches = delta_h_deg * horizontal_tracker_diameter * M_PI / 360.0;
        double delta_heading_rad = degToRad(omega_deg * dt);
        double rotation_component = horizontal_tracker_dist_from_center * delta_heading_rad;
        double pure_lateral = delta_h_inches - rotation_component;
        lat_vel_filter.add(pure_lateral / dt);
        lateral_vel = lat_vel_filter.get();
        prev_h_tracker_deg = curr_h_deg;
        prev_heading = curr_theta;

        if (raw_distance < closest_approach) {
            closest_approach = raw_distance;
            frames_since_closest = 0;
        } else {
            frames_since_closest++;
        }

        double target_angle_raw = radToDeg(atan2(dx, dy));
        if (!moving_forward)
            target_angle_raw = wrapAngle(target_angle_raw + 180.0);
        double raw_angle_error = wrapAngle(target_angle_raw - curr_theta);
        double abs_raw_ae = fabs(raw_angle_error);

        if (loop_count > 30 && closest_approach < 8.0 &&
            raw_distance > closest_approach + 1.5 &&
            abs_raw_ae > 45.0 && frames_since_closest > 5) {
            std::cout << "Divergence exit raw=" << raw_distance
                      << " closest=" << closest_approach
                      << " ae=" << abs_raw_ae << std::endl;
            break;
        }

        if (raw_distance < min_raw_distance) {
            min_raw_distance = raw_distance;
        } else if (raw_distance > min_raw_distance + 0.5 && raw_distance < 7) {
            std::cout << "Anti-orbit triggered. Exiting drive loop." << std::endl;
            break;
        }

        double speed_factor = 0.05 * abs_max_speed;
        double distance_factor = fmin(raw_distance / 5.0, 1.0);
        double effective_momentum = speed_factor * distance_factor;

        double distance = fmax(raw_distance - robot_radius + effective_momentum, 0.0);
        if (distance <= distance_threshold) break;

        double target_angle = radToDeg(atan2(dx, dy));
        if (!moving_forward) target_angle = wrapAngle(target_angle + 180.0);

        double angle_error = wrapAngle(target_angle - SensorVals.HDG);
        if (Matchload_distance >= fabs(distance)) {
          Scrapper.set(false);
        }
        if (depoly_distance >= fabs(distance)) {
          Scrapper.set(true);
        }
        if (distance <= settling_zone) {
            Correction = 0;
            PrevE = 0;
            IVal = 0;
        } else {
            PVal = KVals.kp * angle_error;
            IVal= IVal+KVals.ki*angle_error*0.01;
            DVal = KVals.kd * wrapAngle(angle_error - PrevE);
            PrevE = angle_error;
            Correction = PVal + IVal + DVal/0.01;
        }

        if (initial_dist*0.2 < distance) {
          if (max_correction > fabs(Correction)) {
            max_correction = fabs(Correction);
          }
          if (fabs(Correction) > max_correction) {
            int signCorrection = (Correction > 0) ? 1 : -1;
            Correction = signCorrection * max_correction;
          }
        }

        double ratio = raw_distance / decel_distance;
        ratio = fmin(fmax(ratio, 0.0), 1.0);

        double base_speed = final_decel_speed +
                            ((abs_max_speed - final_decel_speed) * ratio);

        double min_move_speed = final_decel_speed + 5;
        if (base_speed < min_move_speed) base_speed = min_move_speed;

        double left_correction_term = moving_forward ? Correction : -Correction;
        double right_correction_term = moving_forward ? -Correction : Correction;

        double left_speed = base_speed + left_correction_term;
        double right_speed = base_speed + right_correction_term;

        if (!moving_forward) {
            left_speed *= -1.0;
            right_speed *= -1.0;
        }

        double max_req = fmax(fabs(left_speed), fabs(right_speed));
        if (max_req > abs_max_speed) {
            left_speed  = (left_speed / max_req) * abs_max_speed;
            right_speed = (right_speed / max_req) * abs_max_speed;
        }

        Move(left_speed, right_speed);

        // ── LATERAL-ERROR LANDING EXIT ──
        if (raw_distance < 4.0 && spd > 3.0 && loop_count > 15) {
            double approach_vel_raw =
                (dx * vx_filter.get() + dy * vy_filter.get()) /
                fmax(raw_distance, 0.1);
            double approach_ratio = approach_vel_raw / fmax(spd, 0.1);

            if (approach_ratio > 0.5) {
                double vx = vx_filter.get();
                double vy = vy_filter.get();
                double lateral_error = fabs(dx * vy - dy * vx) / fmax(spd, 0.1);

                double braking_time_est = spd / fmax(60.0, 10.0);
                double lateral_drift_during_braking = fabs(lateral_vel) * braking_time_est;
                double predicted_lateral_error = lateral_error + lateral_drift_during_braking;

                if (predicted_lateral_error < 1.5) {
                    std::cout << "Landing exit raw=" << raw_distance
                              << " lat_err=" << lateral_error
                              << " pred_lat=" << predicted_lateral_error
                              << " approach=" << approach_ratio << std::endl;
                    break;
                }
            }
        }

        // ── PERPENDICULAR LINE EXIT (not static!) ──
        bool perpendicular_line =
            ((curr_y - target_y) *
                 -cos(degToRad(normalizeTarget(SensorVals.HDG + add))) <=
             (curr_x - target_x) *
                     sin(degToRad(normalizeTarget(SensorVals.HDG + add))) +
                 1.0);
        if (perpendicular_line && !prev_perpendicular_line) {
            std::cout << "Perp exit raw=" << raw_distance << std::endl;
            break;
        }
        prev_perpendicular_line = perpendicular_line;

        prev_x = curr_x;
        prev_y = curr_y;
        prev_time = curr_time;
        loop_count++;
        wait(10, msec);
    }
    // count += 1;
    IVal = 0;
    // std::cout<<count<< "st movement "<<"X: "<<x_pos<<" Y: "<<y_pos<<std::endl;
    if (brake) {
        BStop();
        wait(100, msec);
    }
}
