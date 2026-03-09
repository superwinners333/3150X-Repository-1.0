#include "vex.h"

#include "math.h"
#include "screen_gui.hpp"
#include "helper_functions.hpp"
#include "movement.hpp"
#include "odom.hpp"
#include "Odometry.hpp"
#include <iostream>
#include <vector>
#include <numeric>
using namespace vex;

void OdomNoWheel() {
    double prev_heading_rad = 0;
    double prev_left_deg = 0, prev_right_deg = 0;
    ChassisDataSet SensorVals;

    while (true) {
        SensorVals = ChassisUpdate();

        double current_heading_rad = degToRad(SensorVals.HDG);
        double delta_left_in = (SensorVals.AbsoluteLeft - prev_left_deg) * wheel_distance_in / 360.0;
        double delta_right_in = (SensorVals.AbsoluteRight - prev_right_deg) * wheel_distance_in / 360.0;
        double delta_heading_rad = current_heading_rad - prev_heading_rad;

        while (delta_heading_rad > M_PI) delta_heading_rad -= 2 * M_PI;
        while (delta_heading_rad < -M_PI) delta_heading_rad += 2 * M_PI;

        double delta_x = 0, delta_y = 0;

        if (fabs(delta_heading_rad) < 1e-6) {
            delta_y = (delta_left_in + delta_right_in)/2.0;
        } else {
            double R = (delta_left_in + delta_right_in) / (2 * delta_heading_rad);
            double chord = 2.0 * R * sin(delta_heading_rad/2.0);
            delta_y = chord;
        }

        CPos.x += delta_y * sin(prev_heading_rad + delta_heading_rad/2.0);
        CPos.y += delta_y * cos(prev_heading_rad + delta_heading_rad/2.0);

        prev_left_deg = SensorVals.AbsoluteLeft;
        prev_right_deg = SensorVals.AbsoluteRight;
        prev_heading_rad = current_heading_rad;

        // display_odom();
        wait(10, msec);
    }
}








void OdomWithX() {
  ChassisDataSet SensorVals;
  double delta_local_x_in = 0, delta_local_y_in = 0;
  double local_polar_angle_rad = 0;

  // Initialize prev values to ACTUAL sensor readings (avoids phantom delta on first loop)
  SensorVals = ChassisUpdate();
  double prev_heading_rad = degToRad(SensorVals.HDG);
  double prev_horizontal_pos_deg = odomx.position(degrees);
  double prev_left_deg = SensorVals.AbsoluteLeft;
  double prev_right_deg = SensorVals.AbsoluteRight;

  while (true) {
    SensorVals = ChassisUpdate();
    double heading_rad = degToRad(SensorVals.HDG);
    double horizontal_pos_deg = odomx.position(degrees);
    double left_deg = SensorVals.AbsoluteLeft;
    double right_deg = SensorVals.AbsoluteRight;
    double delta_heading_rad = heading_rad - prev_heading_rad;
    // Wrap to [-π, π] (matches OdomNoWheel — prevents spike when gyro crosses 0°/360°)
    while (delta_heading_rad > M_PI) delta_heading_rad -= 2 * M_PI;
    while (delta_heading_rad < -M_PI) delta_heading_rad += 2 * M_PI;
    double delta_horizontal_in = (horizontal_pos_deg - prev_horizontal_pos_deg) * horizontal_tracker_diameter * M_PI / 360.0; // horizontal tracker delta (inches)
    double delta_left_in = (left_deg - prev_left_deg) * wheel_distance_in / 360.0;   // Left wheel delta in inches (signed!)
    double delta_right_in = (right_deg - prev_right_deg) * wheel_distance_in / 360.0; // Right wheel delta in inches (signed!)

    // Change based on heading
    if (fabs(delta_heading_rad) < 1e-6) {
      delta_local_x_in = delta_horizontal_in;
      delta_local_y_in = (delta_left_in + delta_right_in) / 2.0;
    } else {
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      delta_local_x_in = sin_multiplier * ((delta_horizontal_in / delta_heading_rad) - horizontal_tracker_dist_from_center);
      double delta_local_y_left_in = sin_multiplier * (delta_left_in / delta_heading_rad + DistanceBetweenWheel / 2.0);
      double delta_local_y_right_in = sin_multiplier * (delta_right_in / delta_heading_rad - DistanceBetweenWheel / 2.0);
      delta_local_y_in = (delta_local_y_left_in + delta_local_y_right_in) / 2.0;
    }

    if (fabs(delta_local_x_in) < 1e-6 && fabs(delta_local_y_in) < 1e-6) {
      local_polar_angle_rad = 0;
    } else {
      local_polar_angle_rad = atan2(delta_local_y_in, delta_local_x_in);
    }
    double polar_radius_in = sqrt(pow(delta_local_x_in, 2) + pow(delta_local_y_in, 2));
    // Mid-angle rotation: θ_mid = heading_rad - dθ/2 = prev_heading + dθ/2
    // global_angle = local_angle - θ_mid = local_angle - heading + dθ/2
    double polar_angle_rad = local_polar_angle_rad - heading_rad + (delta_heading_rad / 2);

    CPos.x += polar_radius_in * cos(polar_angle_rad);
    CPos.y += polar_radius_in * sin(polar_angle_rad);
    
    prev_heading_rad = heading_rad;
    prev_horizontal_pos_deg = horizontal_pos_deg;
    prev_left_deg = left_deg;
    prev_right_deg = right_deg;
    // display_odom();
    wait(10, msec);
  }
}


// ===============================
double FRONT_X = 0.0,  FRONT_Y = 6.0;
double BACK_X  = -4.5,  BACK_Y  = -4.5;
double LEFT_X  = -4.5, LEFT_Y  = 1.5;
double RIGHT_X =  4.5, RIGHT_Y = 1.5;

double ORIGIN_X = 0.0; 
double ORIGIN_Y = 0.0;

double MAX_DIST = 80.0; 
double FUSION_WEIGHT = 1; 




void OdomReset(bool IGNORE_LEFT, bool IGNORE_RIGHT, bool IGNORE_BACK, bool IGNORE_FRONT) { 
    
    // 1. Get Distances (reject invalid/no-object readings)
    const double MIN_VALID_DIST = 1.0; // Sensor minimum reliable range
    double dF = IGNORE_FRONT ? 9999.0 : frontSensor.objectDistance(inches);
    double dL = IGNORE_LEFT  ? 9999.0 : leftSensor.objectDistance(inches);
    double dR = IGNORE_RIGHT ? 9999.0 : rightSensor.objectDistance(inches);
    double dB = IGNORE_BACK  ? 9999.0 : backSensor.objectDistance(inches);

    // Filter out bad readings: 0, negative, or impossibly small = no object detected
    if (dF < MIN_VALID_DIST) { std::cout << "OdomReset: FRONT sensor rejected (" << dF << ")" << std::endl; dF = 9999.0; }
    if (dL < MIN_VALID_DIST) { std::cout << "OdomReset: LEFT sensor rejected (" << dL << ")" << std::endl; dL = 9999.0; }
    if (dR < MIN_VALID_DIST) { std::cout << "OdomReset: RIGHT sensor rejected (" << dR << ")" << std::endl; dR = 9999.0; }
    if (dB < MIN_VALID_DIST) { std::cout << "OdomReset: BACK sensor rejected (" << dB << ")" << std::endl; dB = 9999.0; }

    const double FIELD_W = 140.0;
    const double FIELD_L = 140.0;

    ChassisDataSet S = ChassisUpdate();
    double hdg = S.HDG;        

    // --- STEP 1: STANDARD CONVERSION ---
    // Physical = Logical + Origin
    // This ensures Right is ALWAYS Positive/Increasing
    double x_od_phys = CPos.x + ORIGIN_X;
    double y_od_phys = CPos.y + ORIGIN_Y;

    // Normalize heading (0-360)
    while (hdg < 0) hdg += 360;
    while (hdg >= 360) hdg -= 360;
    double rad = degToRad(hdg); 
   
  
    auto getSensorGlobalOffset = [&](double sX, double sY, double& offX, double& offY) {
        offX = sX * cos(rad) + sY * sin(rad); 
        offY = sY * cos(rad) - sX * sin(rad); 
    };

    double fx_off, fy_off, bx_off, by_off, lx_off, ly_off, rx_off, ry_off;
    getSensorGlobalOffset(FRONT_X, FRONT_Y, fx_off, fy_off);
    getSensorGlobalOffset(BACK_X, BACK_Y,   bx_off, by_off);
    getSensorGlobalOffset(LEFT_X, LEFT_Y,   lx_off, ly_off);
    getSensorGlobalOffset(RIGHT_X, RIGHT_Y, rx_off, ry_off);

    double d_North, d_East, d_South, d_West;
    double offY_North, offX_East, offY_South, offX_West; 

    if (hdg >= 315 || hdg < 45) { // Facing NORTH
        d_North = dF; offY_North = fy_off;
        d_East  = dR; offX_East  = rx_off;
        d_South = dB; offY_South = by_off;
        d_West  = dL; offX_West  = lx_off;
    } 
    else if (hdg >= 45 && hdg < 135) { // Facing EAST
        d_North = dL; offY_North = ly_off; 
        d_East  = dF; offX_East  = fx_off; 
        d_South = dR; offY_South = ry_off;
        d_West  = dB; offX_West  = bx_off;
    } 
    else if (hdg >= 135 && hdg < 225) { // Facing SOUTH
        d_North = dB; offY_North = by_off;
        d_East  = dL; offX_East  = lx_off;
        d_South = dF; offY_South = fy_off;
        d_West  = dR; offX_West  = rx_off;
    } 
    else { // Facing WEST
        d_North = dR; offY_North = ry_off;
        d_East  = dB; offX_East  = bx_off;
        d_South = dL; offY_South = ly_off;
        d_West  = dF; offX_West  = fx_off;
    }

    auto correctDist = [&](double dist, double ignored) {
        double tilt = fmod(hdg, 90.0);
        if (tilt > 45.0) tilt = 90.0 - tilt;
        if (tilt > 20.0) return 9999.0;
        return dist * cos(degToRad(tilt)); 
    };

    double distN = correctDist(d_North, 0);
    double distE = correctDist(d_East,  90);
    double distS = correctDist(d_South, 180);
    double distW = correctDist(d_West,  270);

    std::vector<double> candX, candY;

    if (fabs(distN) < fabs(MAX_DIST)) candY.push_back(FIELD_L - distN - offY_North);
    if (fabs(distS) < fabs(MAX_DIST)) candY.push_back(0.0     + distS - offY_South);
    if (fabs(distE) < fabs(MAX_DIST)) candX.push_back(FIELD_W - distE - offX_East);
    if (fabs(distW) < fabs(MAX_DIST)) candX.push_back(0.0     + distW - offX_West);

    auto fuse = [&](std::vector<double>& v, double odom_phys) {
        if (v.empty()) return odom_phys;
        double best = v[0];
        double minDiff = fabs(best - odom_phys);

        for(double val : v) {
            double diff = fabs(val - odom_phys);
            if(diff < minDiff) {
                minDiff = diff;
                best = val;
            }
        }
        
        return (best * FUSION_WEIGHT) + (odom_phys * (1.0 - FUSION_WEIGHT));
    };

    double final_phys_x = fuse(candX, x_od_phys);
    double final_phys_y = fuse(candY, y_od_phys);


    CPos.x = final_phys_x - ORIGIN_X; 
    CPos.y = final_phys_y - ORIGIN_Y;

    std::cout << "Reset: X=" << CPos.x << " Y=" << CPos.y << " (Phys: " << final_phys_x << "," << final_phys_y << ")" << std::endl;
}