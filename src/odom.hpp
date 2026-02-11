#ifndef ODOM_H
#define ODOM_H

#include <vector>


// bottom left corner of the field is 0,0
struct Point {
  double x; // left and right of the field
  double y; // forward and backwards
};


extern double globalHeading; // starting heading relative to the field
extern Point CPos; // our current position
extern bool odomTracking; // if we should be tracking odom
extern void OdomUpdate(); // odom tracking function
extern bool inRangeOf(double range, double target, double check); // checks if the two values are in range of each other
extern void startTracking(Point start); // start tracking odom

// reset functions
extern Point StartingPosition(void); // gets our starting position
extern Point longGoalReset(Point EP); // resets for when we are at the long goal
extern Point wallReset(Point EP, bool botBack); // resets when front or back of the bot is against the left or righta wall
extern Point resetBack(Point EP); // resets using back distance sensor
extern Point resetLeft(Point EP); // resets using left distance sensor
extern Point resetRight(Point EP); // resets using right distance sensor


// moving in inches
extern void AccuratePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, double maxAccel, int Speed, double timeout, double ABSHDG, bool brake); 

// curving to points 1
extern void MoveToPoint(PIDDataSet DistK, PIDDataSet HeadK, Point target, double Speed, double timeout, double curveFactor, bool brake);

// curving to points 2
extern void curveToPoint(Point target, double speed, double curveP, bool brake);

// turns and then goes straight to point
extern void straightToPoint(PIDDataSet turnK, PIDDataSet HeadK, PIDDataSet DistK, Point target, double speed, double timeout, bool brake);

extern void boohoo(PIDDataSet HeadK, PIDDataSet DistK, Point target, double max, double min, double timeout, bool brake);
#endif