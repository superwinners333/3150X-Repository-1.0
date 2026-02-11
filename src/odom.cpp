#include "vex.h"

#include "math.h"
#include "screen_gui.hpp"
#include "helper_functions.hpp"
#include "movement.hpp"
#include "odom.hpp"

#include <algorithm>
#include <iostream>
#include <vector>
#include <math.h>

using namespace vex;


// support functions for odom

double globalHeading = 0; // should be zero when front of the bot faces the side with the other park


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

double degToRad(double deg) {
  double rad = deg * (M_PI / 180.0);
  return rad;
}

double radToDeg(double rad) {
    double deg = rad / (M_PI / 180.0);
    return deg;
}

double pointDist(Point a, Point b) {
    double dist = hypot((a.x-b.x),(a.y-b.y));
    return dist;
}

double pointHeading(Point current, Point target) {
    double dx = target.x - current.x; // difference in x coords
    double dy = target.y - current.y; // difference in y coords
    
    double hyp = pointDist(current, target);

    double heading = 0;
    // if (dx >= 0) heading = 90.0 - radToDeg(asin(dy/hyp));
    // else heading = -90.0 + radToDeg(asin(dy/hyp));
    heading = radToDeg(atan2(dx,dy));

    return heading;
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



// COORDINATE SYSTEM
const double BDC = 7.992126; // distance from edge of the bot to the center
const double LDC = 8.4448819;
const double RDC = 8.4448819;
const double FDC = 7.992126;

Point CPos; // current position
// 25.4 cm is width between front back wheels
// 33 cm is distance from back to center of front wheel
// 35.2 cm is width of drivetrain
// 27.5 is width between left and right wheels

Point StartingPosition(void) {
  Point SP;
  ChassisDataSet SenVals = ChassisUpdate();

  // distance from wall to center of the bot in the starting pos when front of bot is facing wall
  double xwallConst = 45.2756 + FDC; 
  double ywallConst = 15.94488 + FDC;
  double swallConst = 16.25984 + FDC;

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
  return SP;
}



// returns a new position of the bot
/** @param EP encoder predicted position
*/
Point longGoalReset(Point EP) { // resets base on long goal position
  Point LG; // long goal point
  ChassisDataSet SenVals = ChassisUpdate();
  double trueHeading = globalHeading + SenVals.HDG;
  while (trueHeading > 180.0) trueHeading -= 360.0;
  while (trueHeading < -180.0) trueHeading += 360.0;

  LG = EP; // creates values to output in case the bot thinks our position is too innacurate
  
  // resets x vals
  if (SenVals.leftD <= SenVals.rightD) {
    if (inRangeOf(1.5,180.0,trueHeading) || inRangeOf(1.5,-180.0,trueHeading)) LG.x = 144.0 - (SenVals.leftD + LDC); // close right
    else if (inRangeOf(1.5,0.0,trueHeading)) LG.x = SenVals.leftD + LDC;
  }
  else if (SenVals.rightD < SenVals.leftD) {
    if (inRangeOf(1.5,180.0,trueHeading) || inRangeOf(1.5,-180.0,trueHeading)) LG.x = SenVals.rightD + RDC;
    else if (inRangeOf(1.5,0.0,trueHeading)) LG.x = 144.0 - (SenVals.rightD + RDC);
  }

  // resets y vals with constants 
  // the value of the constant should be the distance from the wall to the center of the bot when the bot is scoring on the long goal
  double LGConst = 42.362205;
  if (inRangeOf(1.0,180.0,trueHeading) || inRangeOf(1.5,-180.0,trueHeading)) LG.y = LGConst; 
  else if (inRangeOf(1.0,0.0,trueHeading)) LG.y = 144.0 - LGConst;

  return LG;
}



// only meant for resetting against the left and right walls
Point wallReset(Point EP, bool botBack) { // if back == true, then the back of our bot is facing the wall
  Point NP;
  ChassisDataSet SenVals = ChassisUpdate();

  double trueHeading = globalHeading + SenVals.HDG; // gets true heading relative to the field
  while (trueHeading > 180.0) trueHeading -= 360.0;
  while (trueHeading < -180.0) trueHeading += 360.0;

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

Point resetBack(Point EP) {
  ChassisDataSet SenVals = ChassisUpdate();
  double trueHeading = globalHeading + SenVals.HDG;
  while (trueHeading > 180.0) trueHeading -= 360.0;
  while (trueHeading < -180.0) trueHeading += 360.0;

  Point NP = EP;
  if (inRangeOf(2.0, 0.0, trueHeading)) NP.y = SenVals.backD + BDC;
  else if (inRangeOf(2.0, 90.0, trueHeading)) NP.x = SenVals.backD + BDC;
  else if (inRangeOf(2.0, 180.0, trueHeading) || inRangeOf(1.5, -180.0,trueHeading)) NP.y = 144.0 - (SenVals.backD + BDC);
  else if (inRangeOf(2.0, -90.0, trueHeading)) NP.x = 144.0 - (SenVals.backD + BDC);

  return NP;
}
Point resetLeft(Point EP) {
  ChassisDataSet SenVals = ChassisUpdate();
  double trueHeading = globalHeading + SenVals.HDG;
  while (trueHeading > 180.0) trueHeading -= 360.0;
  while (trueHeading < -180.0) trueHeading += 360.0;
  Point NP = EP; // gets current position in case our heading is too off to return anything

  if (inRangeOf(2.0, 0.0, trueHeading)) NP.x = SenVals.leftD + LDC;
  else if (inRangeOf(2.0, 90.0, trueHeading)) NP.y = 144.0 - (SenVals.leftD + LDC);
  else if (inRangeOf(2.0, 180.0, trueHeading) || inRangeOf(1.5, -180.0,trueHeading)) NP.x = 144.0 - (SenVals.leftD + LDC);
  else if (inRangeOf(2.0, -90.0, trueHeading)) NP.y = SenVals.leftD + LDC;

  return NP;
}
Point resetRight(Point EP) {
  ChassisDataSet SenVals = ChassisUpdate();
  double trueHeading = globalHeading + SenVals.HDG;
  while (trueHeading > 180.0) trueHeading -= 360.0;
  while (trueHeading < -180.0) trueHeading += 360.0;
  Point NP = EP;

  if (inRangeOf(2.0, 0.0, trueHeading)) NP.x = 144.0 - (SenVals.rightD + RDC);
  else if (inRangeOf(2.0, 90.0, trueHeading)) NP.y = SenVals.rightD + RDC;
  else if (inRangeOf(2.0, 180.0, trueHeading) || inRangeOf(1.5, -180.0,trueHeading)) NP.x = SenVals.rightD + RDC;
  else if (inRangeOf(2.0, -90.0, trueHeading)) NP.y = 144.0 - (SenVals.rightD + RDC);

  return NP;
}

// use negative distance values for moving backwards?


double dist_between_wheels = 10.728346; // distance between the wheels in inches

// odom function
bool odomTracking = true;

void OdomUpdate(){
  Zeroing(true,true);

  
  double prev_heading_rad = globalHeading;
  double prev_left_in = 0, prev_right_in = 0;
  double d_local_y_in = 0;

  while (odomTracking) {
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
* retune inches pid 
* figure out hwo to use the position calculations if we reset
* or figure out how to use the current move functions without resetting
* add a constant or do smth to make encoders more accurate
* add a move to point functioon
*/

void MoveToPoint(PIDDataSet DistK, PIDDataSet HeadK, Point target, double Speed, double timeout, double curveFactor, bool brake)
{
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
  double exitError = 2.0;        // how close to target before stopping
  double exitDerivative = 5.0;   // how still the robot must be
  int exitTime = 50;              // ms required to be stable
  // int timeout = 0;               // ms max runtime

  if (timeout <= 0) timeout = 5.0;
  bool settled = false, timedOut = false;

  double speedSign = Speed / (fabs(Speed));

  double startdist = pointDist(CPos, target);
  Point startpoint = CPos;
  double maxAccel = 4.0;

  double prevCorrection = 0.0, FCorrection = 0.0;

  // ---------------------------------------------------------------
  // ------------------------ main PID loop ------------------------
  // ---------------------------------------------------------------
  bool errorSmall = false;
  while (!settled && !timedOut && !errorSmall) {
    SensorVals = ChassisUpdate(); // gets drivetrain values

    double dist = pointDist(CPos, target); // gets current distance from target
    double dist_moved = startdist - dist; // gets total distance theoretically travelled
    double percent_dist = (dist_moved / startdist) * 100.0;

    // distance PID calculations
    E_dist = -(100.0 - percent_dist); // sets error as percent left to travel
    P_dist = DistK.kp*E_dist;
    I_dist += (DistK.ki)*E_dist*(dt/1000.0);
    D_dist = (DistK.kd*(E_dist-PrevE_dist)) / (dt/1000.0);

    // std::cout << D_dist << std::endl;

    rawSpeed = P_dist + I_dist + D_dist + 10.0;

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
    double targetHeading = pointHeading(CPos, target);
    double relativeHeading = targetHeading - SensorVals.HDG;
    // if (fabs(relativeHeading) > 90.0) {
    //     targetHeading += 180;
    //     outputSpeed *= -1;
    // }
    E_heading = SensorVals.HDG-targetHeading;
    while (E_heading > 180) E_heading -= 360;
    while (E_heading < -180) E_heading += 360;
    P_heading = HeadK.kp*E_heading;
    I_heading += HeadK.ki*E_heading*(dt/1000.0);
    D_heading = (HeadK.kd*(E_heading-PrevE_heading)) / (dt/1000.0);

    Correction = P_heading + I_heading + D_heading; // correction
    Correction *= curveFactor; // adjusts correction power for curve aggressiveness tuning

    double correctionChange = Correction - FCorrection;
    if (correctionChange > 20.0) correctionChange = 20.0;
    else if (correctionChange < 20.0) correctionChange = -20.0;
    FCorrection += correctionChange;

    // tells the bot how much to run each side
    Move(outputSpeed+FCorrection,outputSpeed-FCorrection);
    
    PrevE_heading = E_heading; // updates previous headings
    PrevE_dist = E_dist;
    prevCorrection = Correction;

    // distance exit conditions calculations
    errorSmall = fabs(pointDist(CPos,target)) <= exitError;
    bool derivativeSmall = fabs(D_dist) <= exitDerivative;

    if (errorSmall && derivativeSmall) settledTime += dt; // if we are close to the target, start counting
    else settledTime = 0; // if we start adjusting again, stop counting


    // exit condition checks
    settled = settledTime >= exitTime; 
    timedOut = Brain.Timer.value() >= timeout;

    std::cout << "Timer: " << Brain.Timer.value() << std::endl;
    std::cout << "target dist: " << dist << std::endl;
    std::cout << "target heading: " << targetHeading << std::endl;
    double Pheading = SensorVals.HDG;
    if (SensorVals.HDG > 180) Pheading -= 360.0;
    std::cout << "current heading: " << Pheading << std::endl;
    std::cout << "CPos: " << CPos.x << "," << CPos.y <<std::endl;
    std::cout << " " << std::endl;

    if (settled || timedOut) break; // if either exit condition is met, exit

    wait(20,msec); // wait to stop constant looping
  }

  std::cout << "exit" << std::endl;
  std::cout << CPos.x << std::endl;
  std::cout << CPos.y << std::endl;
  std::cout << pointDist(CPos,target) << std::endl;
  if(brake){BStop(); // braking logic
  wait(100,msec);}
  else CStop();
}

void startTracking(Point start) {
    CPos = start;
    wait(50,msec);
    thread odom_tracking = thread(OdomUpdate);
}



/** function makes the bot travel the specified distance
*   (negative values for speed to make the bot go backwards) 
* @param target point to curve to
* @param speed max speed
* @param curveP curve proportional
* @param brake braking
*/ 
void curveToPoint(Point target, double speed, double curveP, bool brake) {
  ChassisDataSet SensorVals;
  double dist = pointDist(CPos,target);
  double startdist = dist;
  Brain.Timer.reset();
  while (dist >= 1.5 && Brain.Timer.value() < 5.0) {
    ChassisDataSet SensorVals = ChassisUpdate();
    double targetHeading = pointHeading(CPos,target);

    if (speed < 0) targetHeading += 180.0;
    // if (targetHeading > 180) targetHeading - 360.0;

    double E_heading = SensorVals.HDG - targetHeading;
    while (E_heading > 180) E_heading -= 360.0;
    while (E_heading < -180) E_heading += 360.0;

    double curvature = degToRad(E_heading) / curveP; // curving math
    curvature *= (dist/startdist)*2.0;
    // std::clamp(curvature,-1,1);

    double left = -speed * (1.0-curvature); // calculates powers for left and right sides
    double right = -speed * (1.0+curvature);
    // clamps left and right
    if (left > 100) left = 100.0;
    else if (left < -100) left = -100.0;
    if (right > 100) right = 100.0;
    else if (right < -100) right = -100.0;

    Move(left,right); // move

    dist = pointDist(CPos,target); // exit condition check
    std::cout << dist << std::endl;
    wait(20,msec); // prevents crashing
  }

  std::cout << " " << std::endl;
  std::cout << "exit" << std::endl;
  std::cout << Brain.Timer.value() << std::endl;
  std::cout << CPos.x << std::endl;
  std::cout << CPos.y << std::endl;

  if(brake){
    BStop(); // braking logic
    wait(100,msec);}
  else CStop();
}


void straightToPoint(PIDDataSet turnK, PIDDataSet HeadK, PIDDataSet DistK, Point target, double speed, double timeout, bool brake) {
  ChassisDataSet SensorVals = ChassisUpdate();
  double targetHeading = pointHeading(CPos, target);
  if (speed < 0) targetHeading += 180.0; // for backwards movement
  if (targetHeading > 180) targetHeading -= 360.0;

  double dHeading = targetHeading-SensorVals.HDG;
  if (dHeading < -180) dHeading += 360.0;

  TurnMaxTimePID(turnK, targetHeading, fabs(dHeading)/225.0, false); // turns to face target

  double dist = pointDist(CPos, target);
  Brain.Timer.reset(); // resets timer for exiting

  double outputSpeed = 0.0, rawSpeed = 0.0, correction = 0.0;
  double E_dist = 0.0, PrevE_dist = 0.0, P_dist = 0.0, I_dist = 0.0, D_dist = 0.0;
  double E_head = 0.0, PrevE_head = 0.0, P_head = 0.0, I_head = 0.0, D_head = 0.0;
  double dt = 20.0;
  
  int settledTime = 200, exitTime = 50;
  double exitError = 1.0, exitDerivative = 2.0, maxAccel = 6.0;
  bool settled = false;

  double startdist = dist;

  while (!settled && Brain.Timer.value() < timeout) {
    SensorVals = ChassisUpdate();

    dist = pointDist(CPos, target);
    double percent_dist = (dist / startdist) * 100.0; // percent of movement done

    E_dist = -percent_dist;
    P_dist = DistK.kp*E_dist;
    I_dist += (DistK.ki+(startdist/1000.0))*E_dist*(dt/1000.0);
    D_dist = (DistK.kd*(E_dist-PrevE_dist)) / (dt/1000.0);
    rawSpeed = P_dist + I_dist + D_dist + ((startdist/4.3) * (E_dist / fabs(E_dist))); // output speed

    if (speed < 0) rawSpeed = rawSpeed * -1.0;
    if (fabs(rawSpeed) > fabs(speed)) { // clamps outputSpeed at max speed
      if (speed >= 0 && rawSpeed >= 0) rawSpeed = speed;
      else if (speed < 0 && rawSpeed < 0) rawSpeed = speed;
    }
    double speedChange = rawSpeed - outputSpeed; // gets change in speed
    if (speedChange > maxAccel) speedChange = maxAccel;
    if (speedChange < -maxAccel) speedChange = -maxAccel;
    outputSpeed += speedChange; // accelerates


    // heading PID adjustment calculations
    targetHeading = pointHeading(CPos, target);
    if (speed < 0) targetHeading += 180.0; // for backwards movement
    if (targetHeading > 180) targetHeading -= 360.0;
    
    E_head = SensorVals.HDG-targetHeading;
    while (E_head > 180) E_head -= 360;
    while (E_head < -180) E_head += 360;

    P_head = HeadK.kp*E_head;
    I_head += HeadK.ki*E_head*(dt/1000.0);
    D_head = (HeadK.kd*(E_head-PrevE_head)) / (dt/1000.0);

    correction = P_head + I_head + D_head; // correction
    correction *= ((E_dist/100.0)*(E_dist/100.0));

    if (fabs(E_head) > 90) outputSpeed *= -1.0;

    // tells the bot how much to run each side
    Move(outputSpeed+correction,outputSpeed-correction);

    PrevE_head = E_head; // updates previous headings
    PrevE_dist = E_dist;

    // exit condition
    bool errorSmall = fabs(dist) <= exitError;
    bool derivativeSmall = fabs(E_dist-PrevE_dist) <= exitDerivative;
    if (errorSmall && derivativeSmall) settledTime += dt; // if we are close to the target, start counting
    else settledTime = 0; // if we start adjusting again, stop counting
    settled = settledTime >= exitTime; 

    std::cout << "Timer: " << Brain.Timer.value() << std::endl;
    std::cout << "distance: " << dist << std::endl;
    std::cout << "error: " << E_dist << std::endl;
    std::cout << "output: " << outputSpeed << std::endl;
    std::cout << " " << std::endl;

    wait(20,msec);
  }

  std::cout << "exit" << std::endl;
  std::cout << CPos.x << std::endl;
  std::cout << CPos.y << std::endl;
  if(brake) {
    BStop(); // braking logic
    wait(100,msec);}
  else CStop();
}

void boohoo(PIDDataSet HeadK, PIDDataSet DistK, Point target, double max, double min, double timeout, bool brake) {
  ChassisDataSet SensorVals = ChassisUpdate();
  double targetHeading = pointHeading(CPos, target);
  double dist = pointDist(CPos, target);
  Brain.Timer.reset(); // resets timer for exiting

  double outputSpeed = 0.0, rawSpeed = 0.0, correction = 0.0, prevTargetHeading = 0.0;
  double E_dist = 0.0, PrevE_dist = 0.0, P_dist = 0.0, I_dist = 0.0, D_dist = 0.0;
  double E_head = 0.0, PrevE_head = 0.0, P_head = 0.0, I_head = 0.0, D_head = 0.0;
  double dt = 20.0;
  
  int settledTime = 200, exitTime = 50;
  double exitError = 1.0, exitDerivative = 2.0, maxAccel = 100.0;
  bool settled = false;

  double startdist = dist;

  while (!settled & Brain.Timer.value() < 5.0) { // maybe try to just stop when dist < x if settled does not work
    SensorVals = ChassisUpdate();

    dist = pointDist(CPos, target);
    double percent_dist = (dist / startdist) * 100.0; // percent of movement done

    E_dist = -percent_dist;
    P_dist = DistK.kp*E_dist;
    // I_dist += (DistK.ki+(startdist/1000.0))*E_dist*(dt/1000.0);
    // D_dist = (DistK.kd*(E_dist-PrevE_dist)) / (dt/1000.0);
    rawSpeed = P_dist + I_dist + D_dist; // output speed

    if (max < 0) rawSpeed = rawSpeed * -1.0;
    if (fabs(rawSpeed) > fabs(max)) { // clamps outputSpeed at max speed
      if (max >= 0 && rawSpeed >= 0) rawSpeed = max;
      else if (max < 0 && rawSpeed < 0) rawSpeed = max;
    }
    if (fabs(rawSpeed) < fabs(min)) { // clamps outputSpeed at max speed
      if (min >= 0 && rawSpeed >= 0) rawSpeed = min;
      else if (min < 0 && rawSpeed < 0) rawSpeed = min;
    }
    double speedChange = rawSpeed - outputSpeed; // gets change in speed
    if (speedChange > maxAccel) speedChange = maxAccel;
    if (speedChange < -maxAccel) speedChange = -maxAccel;
    outputSpeed += speedChange; // accelerates

    // heading PID adjustment calculations
    targetHeading = pointHeading(CPos, target);
    if (max < 0) targetHeading += 180.0; // for backwards movement
    if (targetHeading > 180) targetHeading -= 360.0;
    
    E_head = SensorVals.HDG-targetHeading;
    while (E_head > 180) E_head -= 360;
    while (E_head < -180) E_head += 360;

    P_head = HeadK.kp*E_head;
    I_head += HeadK.ki*E_head*(dt/1000.0);
    D_head = (HeadK.kd*(E_head-PrevE_head)) / (dt/1000.0);

    correction = P_head + I_head + D_head; // correction
    // correction *= ((E_dist/100.0)*(E_dist/100.0));

    if (fabs(E_head) > 90) outputSpeed *= -1.0;

    // tells the bot how much to run each side
    Move(outputSpeed+correction,outputSpeed-correction);

    PrevE_head = E_head; // updates previous headings
    PrevE_dist = E_dist;

    // exit condition
    bool errorSmall = fabs(dist) <= exitError;
    bool derivativeSmall = fabs(E_dist-PrevE_dist) <= exitDerivative;
    if (errorSmall && derivativeSmall) settledTime += dt; // if we are close to the target, start counting
    else settledTime = 0; // if we start adjusting again, stop counting
    settled = settledTime >= exitTime; 

    std::cout << "Timer: " << Brain.Timer.value() << std::endl;
    std::cout << "distance: " << dist << std::endl;
    std::cout << "error: " << E_dist << std::endl;
    std::cout << "output: " << outputSpeed << std::endl;
    std::cout << " " << std::endl;

    wait(20,msec);
  }
  if(brake) {
    BStop(); // braking logic
    wait(100,msec);}
  else CStop();
}