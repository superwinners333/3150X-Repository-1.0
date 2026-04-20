#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include "../odom.hpp"
#include "../Odometry.hpp"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);

void low_counter_rush() { // NEGATIVE TURNS TO THE LEFT
  PIDDataSet TurnPara={1.5,0.1,0.12};
  PIDDataSet TestPara={2,0.1,0.3};
  PIDDataSet DrivePara={2.4,0.12,0.1};
  PIDDataSet curvePara={1.9,0.1,0.24};

  timer stopwatch;
  ORIGIN_Y = 24.75;
  ORIGIN_X = 140-56.5;

  RunIndex(100);
  RunLever(-100);
  driveToPoint(DrivePara, 3, 15, 100, 95, 2, false); // grabs 3 blocks in middle
  levertracker.setPosition(0,degrees);
  RunLever(0);
  Scrapper.set(true);
  driveToPoint(DrivePara, 34.7, 7.5, 80, 35, 2.6, true); // goes to between long goal and matchload
  // wait(100,msec);
  // std::cout<< CPos.x <<std::endl;
  MoveTimePID(TestPara, 40, 1.05, 0.02, -180, false); // matchload
  OdomReset(false,false,true,true);
  CPos.y = -8.0;

  MoveEncoderPID(TurnPara, -80, 5.0, 0.3, 180, true); // Move away from matchload
  TurnMaxTimePID(TurnPara, -45, 0.25, false); // turn to low goal
  lock.set(false);
  Scrapper.set(false);
  driveToPoint(DrivePara, -3.4, 19.2, 80, 40, 2.6, true); // go to low goal
  RunIndex(-50);
  // wait(500,msec);
  MoveTimePID(TestPara, 40, 0.6, 0.02, -45, false); // score in low goal
  RunIndex(100);
  MoveEncoderPID(TurnPara, -100, 3, 0.2, -45, false); // back up from low goal
  Wings.set(false);
  leverLift(false);
  TurnMaxTimePID(TurnPara, 88, 0.25, false); // turn to blocks under long goal
  MoveEncoderPID(TurnPara, 90, 40, 0.2, 90, true); // picks up blocks under long goal
  leverLift(true);
  Wings.set(true);
  TurnMaxTimePID(TurnPara, -135, 0.30, false); // turn to matchload wall to prepare to wing out
  MoveEncoderPID(TurnPara, 80, 5, 0.2, -160, false); // move into long goal to wing align
  Wings.set(false); // lowers wings
  MoveEncoderPID(TurnPara, 80, 14, 0.2, -178, false); // wing out long goal
  OdomReset(false,false,true,true);
  MoveEncoderPID(TurnPara, 80, 8, 0.2, -160, false); // go to between matchload and long goal
  TurnMaxTimePID(TurnPara, 180, 0.25, false); // turn to long goal
  CPos.y = 5.0;
  driveToPoint(DrivePara, 33.7, 14.9, -80, -40, 2.6, false); // go into long goal
  leverFull(70);


  

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
}