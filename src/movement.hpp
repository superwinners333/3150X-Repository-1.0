#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <vector>

extern int turninverse;
extern int JB;
extern int PB;
extern int PX;
extern int JX;

struct ChassisDataSet{
  double Left;
  double Right;
  double Avg;   // Average between left and right of the drive train
  int Diff;     // Left - Right
  double HDG;   // Robot heading

  double backD;
  double leftD;
  double rightD;

};

struct PIDDataSet{
  double kp;
  double ki;
  double kd;
};


// bottom left corner of the field is 0,0
struct Point {
  double x; // left and right of the field
  double y; // forward and backwards
  double h;
};
extern double globalHeading;
extern Point Cpos;

extern bool inRangeOf(double range, double target, double check);

extern void Zeroing(bool dist, bool HDG);
extern ChassisDataSet ChassisUpdate();
extern ChassisDataSet ChassisUpdate2();
extern void Move(int left, int right);
extern void BStop();
extern void CStop();
extern void RunRoller(int val);
extern void RunTopRoller(int val);
extern void RunIndex(int val);
extern void MiddleScore(void);
extern void NeutralScore(void);
extern void HighScore(void);

extern int PrevE;
extern void MoveEncoderPID(PIDDataSet KVals, int Speed, double dist,double AccT, double ABSHDG,bool brake);
extern void TurnMaxTimePID(PIDDataSet KVals,double DeltaAngle,double TE, bool brake);
void MaxTimePIDTurnOneSide(PIDDataSet KVals,double DeltaAngle,double TE, bool brake);
void MoveTimePID(PIDDataSet KVals, int Speed, double TE,double AccT,double ABSHDG, bool brake);
extern void MovePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, double maxAccel, int Speed, double timeout, double ABSHDG, bool brake);
extern void WallBackPID(PIDDataSet DistK, PIDDataSet HeadK, double distFromWall, double maxAccel, int Speed, double timeout, double ABSHDG, bool brake);

extern void OdomUpdate();

extern void AccuratePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, double maxAccel, int Speed, double timeout, double ABSHDG, bool brake);
#endif