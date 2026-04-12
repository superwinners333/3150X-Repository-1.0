#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "../odom.hpp"
#include "../Odometry.hpp"
#include "vex.h"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

// MoveDistancePID(PIDDataSet KDist, PIDDataSet KTurn, double dist, double ABSHDG, bool brake)
// PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)

void high_rush() { // NEGATIVE TURNS TO THE LEFT
  // declare initial conditions
  PIDDataSet TurnPara={1.5,0.1,0.12};
  PIDDataSet TestPara={2,0.1,0.3};
  PIDDataSet DrivePara={2.4,0.12,0.1};
  PIDDataSet curvePara={1.9,0.1,0.24};

  timer stopwatch;
  ORIGIN_Y = 24.75;
  ORIGIN_X = 56.5;

  RunIndex(100);
  RunLever(-100);
  driveToPoint(DrivePara, -3, 15, 100, 95, 2, false);
  levertracker.setPosition(0,degrees);
  RunLever(0);
  Scrapper.set(true);
  driveToPoint(DrivePara, -30, 9, 75, 40, 2.6, false, 5.0);
  // wait(100,msec);
  // std::cout<< CPos.x <<std::endl;
  MoveTimePID(TestPara, 55, 1.2, 0.02, -180, false);
  CPos.y = -10.0;
  driveToPoint(DrivePara, -35.5, 15, -80, -20, 2.6, false);
  Move(-40,-40);
  wait(50,msec);
  leverFull(100);
  Move(0,0);
  wait(50,msec);
  CPos.y = (41.0-24.75);
  CPos.x = (23.0-56.5);
  wait(50,msec);
  // driveToPoint(DrivePara, -24, 10, 80, 20, 2.6, false);
  MoveEncoderPID(TurnPara, 100, 4, 0.2, 160, false);
  TurnMaxTimePID(TurnPara, -150, 0.2, false);
  MoveEncoderPID(TurnPara, -100, 9, 0.3, -150, false); 
  Wings.set(false); // lowers wings

  TurnMaxTimePID(TurnPara, -180, 0.2, false); // turns to matchload

  MoveEncoderPID(TurnPara, -80, 17, 0.2, 179, false); // backs up to wing
  wait(150,msec);
  Move(40,0);
  wait(100,msec);
  std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
  wait(2000,msec);

  int screenheading = Gyro.heading(degrees);
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(monoL);
  Brain.Screen.setPenColor("#808080");
  Brain.Screen.setCursor(3,10);
  Brain.Screen.print("HEADING:");
  Brain.Screen.setCursor(4,10);
  Brain.Screen.print(screenheading);
}

/*
    MoveEncoderPID(TurnPara, -100, 10.5, 0.2, -30, false); // goes forward
    Scrapper.set(true);
    CurveEncoderPID(TurnPara, 10, -100, 10, 0.2, 0, false);
    // TurnMaxTimePID(TurnPara, -125, 0.35, false); // turns to between long goal and matchload tube

    MoveEncoderPID(TurnPara, -80, 32, 0.2, -125, true); // goes between there

    TurnMaxTimePID(TurnPara, -178, 0.2, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.36, 0.2, -178, false); // goes into matchload
    MoveTimePID(TurnPara, 40, 0.67, 0.1, -178, false); // slows down

    MoveTimePID(TurnPara, -80, 0.7, 0.2, -178, false); // goes backwards into long goal
    HighScore(); // activates long goal scoring
    MoveTimePID(TurnPara, -40, 1.55, 0.2, -178, false); // pushes into long goal

    // wing code
    MoveEncoderPID(TestPara, -100, 2, 0.2, 175, false);
    MoveEncoderPID(TestPara, -110, 3, 0.3, 130, false); 
    MoveEncoderPID(TestPara, -110, 4, 0.1, 90, false); // aggressively curves
    Wings.set(false); // lowers wings
    NeutralScore(); // stops rolling block violations

    MoveEncoderPID(TestPara, 110, 3.7, 0.4, -100, false); // straightens the bot out 

    MoveEncoderPID(TestPara, 80, 18.2, 0.2, 179, false); // backs up to wing
    wait(150,msec);
    Move(-40,0);
    wait(100,msec);
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
    wait(2000,msec);
    */