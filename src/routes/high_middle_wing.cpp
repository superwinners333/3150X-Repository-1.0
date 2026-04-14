#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include <iostream>
#include "../Odometry.hpp"
#include "../odom.hpp"
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);

void high_middle_wing() { // NEGATIVE TURNS TO THE LEFT
  // declare initial conditions

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
  driveToPoint(DrivePara, -34.5, 9, 80, 40, 2.6, true);
  // wait(100,msec);
  // std::cout<< CPos.x <<std::endl;
  MoveTimePID(TestPara, 55, 1.29, 0.02, -180, false);
  CPos.y = -10.0;
  driveToPoint(DrivePara, -34, 15, -80, -40, 2.6, false);
  MoveTimePID(TestPara, 50, 0.15, 0.02, -180, false);
  MoveTimePID(TestPara, -60, 0.1, 0.02, -180, false);
  Move(-40,-40);
  wait(100,msec);
  leverHalf(90);
  Move(0,0);
  wait(50,msec);
  CPos.y = (41.0-24.75);
  OdomReset(false,false,true,true);
  // CPos.x = (23.0-56.5);
  wait(50,msec);
  leverDown();
  
  driveToPoint(DrivePara, -34, 0, 90, 70, 2.6, false);
  leverLift(false);
  driveToPoint(DrivePara, -2, 31, -80, -20, 2.6, true);
  Move(-20,-20);
  leverFull(60);
  driveToPoint(DrivePara, -24, 15, 80, 50, 2.6, true);
  leverLift(true);
  

  Wings.set(false); // lowers wings
  RunLever(-100);
  RunIndex(-100);
  Scrapper.set(false);
  TurnMaxTimePID(TurnPara, -180, 0.2, false); // turns to wing
  MoveEncoderPID(TurnPara, -80, 12, 0.2, 179, false); // backs up to wing
  RunLever(0);
  RunIndex(0);
  wait(150,msec);
  Move(40,-5);
  wait(100,msec);
  // Scrapper.set(true);
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

  /*
  PIDDataSet TurnPara={1.5,0.1,0.12};
  PIDDataSet TestPara={1.5,0.1,0.15};
  PIDDataSet PurePara={1.5,0.1,0.12};

  timer stopwatch;
  // 4+3
  RunIndex(100);
  MoveEncoderPID(TurnPara, -100, 10.5, 0.2, -30, false); // goes forward
  Scrapper.set(true);
  CurveEncoderPID(TurnPara, 10, -100, 10, 0.2, 0, false);
  // TurnMaxTimePID(TurnPara, -125, 0.35, false); // turns to between long goal and matchload tube

  MoveEncoderPID(TurnPara, -80, 30.1, 0.2, -125, true); // goes between there

  TurnMaxTimePID(TurnPara, -180, 0.2, false); // turns to matchload
  MoveTimePID(TurnPara, 100, 0.36, 0.2, -180, false); // goes into matchload
  MoveTimePID(TurnPara, 40, 0.67, 0.1, -180, false); // slows down

  MoveTimePID(TurnPara, -80, 0.7, 0.2, -178, false); // goes backwards into long goal
  HighScore(); // activates long goal scoring
  MoveTimePID(TurnPara, -50, 0.60, 0.2, -178, false); // pushes into long goal
  NeutralScore();
  wait(200,msec);
  MoveEncoderPID(TestPara, -90, 12, 0.2, -178, true); // moves forward
  TurnMaxTimePID(TurnPara, -135, 0.4, false); // turns to face middle
  MoveEncoderPID(TestPara, 100, 31, 0.3, -136, false); // moves back into middle goal
  MoveTimePID(TestPara, -40, 0.5, 0.1, -135, false); // slows down
  RunIndex(80);
  MiddleScore();
  wait(600,msec);
  NeutralScore();
  MoveEncoderPID(TestPara, -100, 22.4, 0.3, -135, false); // moves forward into wing position

  TurnMaxTimePID(TurnPara, 179, 0.4, false); // turns to wing
  Wings.set(false);
  MoveEncoderPID(TestPara, 100, 10, 0.6, 179, false); // backs up to wing
  wait(200,msec);
  Move(-40,0);
  wait(100,msec);
  std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
  wait(2000,msec);
  */
}