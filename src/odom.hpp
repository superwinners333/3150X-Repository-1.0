#ifndef ODOM_H
#define ODOM_H

#include <vector>


// bottom left corner of the field is 0,0
struct Point {
  double x; // left and right of the field
  double y; // forward and backwards
};


extern double globalHeading;
extern Point CPos;

extern bool inRangeOf(double range, double target, double check);

extern bool odomTracking;
extern void OdomUpdate();

extern void AccuratePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, double maxAccel, int Speed, double timeout, double ABSHDG, bool brake);

extern void MoveToPoint(PIDDataSet DistK, PIDDataSet HeadK, Point target, double Speed, double timeout, double curveFactor, bool brake);

extern void startTracking(Point start);


void MoveToPoint2(PIDDataSet DistK, PIDDataSet HeadK, Point target[], double Speed, double timeout, double curveFactor, bool brake);
void MoveToPoint3(int targetLength, PIDDataSet HeadK, Point target[], double Speed, double timeout, double curveFactor, bool brake);
#endif