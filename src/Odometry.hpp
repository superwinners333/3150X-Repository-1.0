#ifndef Odometry_H
#define Odometry_H

extern double x_pos;        //x cord
extern double y_pos; 
extern double correct_angle;
extern double ORIGIN_X; 
extern double ORIGIN_Y;
extern void OdomWithX();
extern void OdomNoWheel();
extern void OdomFusedOmni(); // New Omni-Correction Odometry
extern void OdomReset(bool IGNORE_LEFT = false,bool IGNORE_RIGHT= false,bool IGNORE_BACK= false, bool IGNORE_FRONT= true);
#endif