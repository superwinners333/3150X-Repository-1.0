#ifndef ODOM_H
#define ODOM_H

#include <vector>


// bottom left corner of the field is 0,0
struct Point {
  double x; // left and right of the field
  double y; // forward and backwards
};


extern double globalHeading;
extern Point Cpos;

extern bool inRangeOf(double range, double target, double check);

extern void OdomUpdate();

extern void AccuratePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, double maxAccel, int Speed, double timeout, double ABSHDG, bool brake);
#endif