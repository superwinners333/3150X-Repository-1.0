#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <vector>

extern int turninverse;
extern int JB;
extern int PB;
extern int PX;
extern int JX;

struct ChassisDataSet{
  double Left; // left side distance travelled
  double Right; // right side distance travelled
  double Avg;   // Average between left and right of the drive train
  int Diff;     // Left - Right
  double HDG;   // Robot heading

  double hor;

  double Average;
  double AbsoluteLeft;
  double AbsoluteRight;

  double backD;
  double leftD;
  double rightD;

};

struct PIDDataSet{
  double kp;
  double ki;
  double kd;
};

extern double Old_distance;
extern double odom_left_offset;  
extern double odom_right_offset; 

extern void Zeroing(bool dist, bool HDG,bool odom);
extern ChassisDataSet ChassisUpdate();
extern ChassisDataSet ChassisUpdate2();
extern void Move(double left, double right);
extern void Move(int left, int right);
extern void BStop();
extern void CStop();

void RunLever(int val);
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

extern void CurveEncoderPID(PIDDataSet KVals, int SpeedL, int SpeedR, double dist,double AccT, double ABSHDG,bool brake);

extern void MovePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, double maxAccel, int Speed, double timeout, double ABSHDG, bool brake);
#endif