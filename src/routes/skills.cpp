#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void MoveEncoderPID2(PIDDataSet KVals, int Speed, double dist, double AccT, double ABSHDG, bool brake) {
  double CSpeed = 0;
  Zeroing(true, false); // Reset encoders only
  double PVal = 0, IVal = 0, DVal = 0, PrevE = 0;
  double Correction = 0;
  double dt = 0.02;

  Brain.Screen.clearScreen();

  while (true) {
    ChassisDataSet SensorVals = ChassisUpdate(); // Update sensors at top
    if (fabs(SensorVals.Avg) >= fabs(dist)) break;

    // Acceleration ramp
    if (fabs(CSpeed) < fabs(Speed)) {
      CSpeed += Speed / AccT * dt;
      CSpeed = fmin(CSpeed, Speed); // Clamp to max speed
    }

    // Heading error calculation
    double LGV = SensorVals.HDG - ABSHDG;
    if (LGV > 180) LGV -= 360;
    if (LGV < -180) LGV += 360;

    // PID calculations
    PVal = KVals.kp * LGV;
    IVal += KVals.ki * LGV * dt;
    IVal = fmax(fmin(IVal, 100), -100); // Clamp integral
    DVal = KVals.kd * ((LGV - PrevE) / dt);

    Correction = PVal + IVal + DVal;

    // Apply motor power (-100 is forward)
    double leftPower = -(CSpeed - Correction);
    double rightPower = -(CSpeed + Correction);

    // Clamp motor output
    leftPower = fmax(fmin(leftPower, 100), -100);
    rightPower = fmax(fmin(rightPower, 100), -100);

    Move(leftPower, rightPower);

    PrevE = LGV;
    wait(dt * 1000, msec); // Wait 20ms
  }

  if (brake) {
    BStop();
    wait(120, msec);
  } else {
    CStop();
  }
}


void MoveEncoderPID3(PIDDataSet KVals, int Speed, double dist, double AccT, double ABSHDG, bool brake, double brakeTune) {
  double CSpeed = 0;
  Zeroing(true, false); // Reset encoders only
  double PVal = 0, IVal = 0, DVal = 0, PrevE = 0;
  double Correction = 0;
  double dt = 0.02;

  Brain.Screen.clearScreen();

  while (true) {
    ChassisDataSet SensorVals = ChassisUpdate(); // Update sensors at top
    double remaining = fabs(dist - SensorVals.Avg);
    double stoppingDistance = brakeTune * CSpeed; // Predictive braking

    if (remaining <= stoppingDistance + 0.05) break; // Exit early to prevent overshoot

    // Acceleration ramp
    if (fabs(CSpeed) < fabs(Speed)) {
      CSpeed += Speed / AccT * dt;
    }

    // Deceleration ramp near target
    double decelFactor = fmin(remaining / 6.0, 1.0);
    CSpeed = fmin(CSpeed, Speed * decelFactor); // Clamp to decel-adjusted speed

    // Heading error
    double LGV = SensorVals.HDG - ABSHDG;
    if (LGV > 180) LGV -= 360;
    if (LGV < -180) LGV += 360;

    // PID calculations
    PVal = KVals.kp * LGV;
    IVal += KVals.ki * LGV * dt;
    IVal = fmax(fmin(IVal, 100), -100); // Clamp integral
    DVal = KVals.kd * ((LGV - PrevE) / dt);

    Correction = PVal + IVal + DVal;

    // Apply motor power (-100 is forward)
    double leftPower = -(CSpeed + Correction);
    double rightPower = -(CSpeed - Correction);

    // Clamp motor output
    leftPower = fmax(fmin(leftPower, 100), -100);
    rightPower = fmax(fmin(rightPower, 100), -100);

    Move(leftPower, rightPower);

    PrevE = LGV;
    wait(dt * 1000, msec); // Wait 20ms
  }

  if (brake) {
    BStop();
    wait(200, msec); // Let encoders settle
  } else {
    CStop();
  }

  // Final encoder update
  ChassisDataSet finalVals = ChassisUpdate();
  Brain.Screen.printAt(10, 60, "Final Dist: %.2f", finalVals.Avg);
}

void skills() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet TestPara={1.5,0.1,0.12};
    PIDDataSet PidTuner={1.0,0.1,0.1};
    MoveEncoderPID2(TestPara, 100, 80, 0.3, 0,true);
    // MoveEncoderPID3(TestPara, 100, 24, 0.3, 0,true,0.10);
    wait(500,msec);
    // TurnMaxTimePID(TestPara, 90, 0.6, true);
    // wait(500,msec);
    // TurnMaxTimePID(TestPara, -90, 0.6, true);
    // wait(500,msec);
    // TurnMaxTimePID(TestPara, 90, 0.6, true);
    // wait(500,msec);
    // TurnMaxTimePID(TestPara, 0, 0.6, true);
    // wait(500,msec);
    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}