#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "../odom.hpp"
#include "../Odometry.hpp"
#include "vex.h"
#include <iostream>
#include <vector>
#include <utility>
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);



void counter_awp() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
  PIDDataSet TurnPara={1.5,0.1,0.15};
  PIDDataSet TestPara={2,0.1,0.3};
  PIDDataSet DrivePara={2.4,0.12,0.1};
  PIDDataSet curvePara={1.9,0.1,0.24};

  timer stopwatch;
  // CPos.y = 24.75;
  // CPos.x = 56.5;

  Gyro.setHeading(-90,degrees);
  ORIGIN_X = 86.7;
  ORIGIN_Y = 21.1;

  RunIndex(100);
  RunLever(-100);
  MoveEncoderPID(TurnPara, -100, 13.2, 0.3, -100, false);
  levertracker.setPosition(0,degrees);
  RunLever(0);
  Scrapper.set(true);
  TurnMaxTimePID(TurnPara, 180, 0.2, false); // turns to matchloader
  MoveTimePID(TurnPara, 45, 1.00, 0.2, 180,false); // move into matchloader
  OdomReset(false,false,true,true);
  driveToPoint(DrivePara, 30.3, 16.5, -70, -40, 2.6, true); // go into long goal
  std::cout<< "CPos.y: " <<CPos.y<<std::endl;
  Move(-40,-40);
  wait(250,msec);
  leverFull(90);
  Move(0,0);
  leverDown();
  std::cout<< "CPos.x: " <<CPos.x<<std::endl;
  OdomReset(false,false,true,true);
  CPos.y = (40.0-ORIGIN_Y); // 41.9-21.1= 20.8
  Scrapper.set(false);
  MoveEncoderPID(TurnPara, 60, 1.6, 0.1, 180,false); // move out of long goal
  TurnMaxTimePID(TurnPara, -61, 0.25, false); // turn to next 3 blocks
  RunIndex(100);
  lock.set(false);
  MoveEncoderPID(TurnPara, 80, 25, 0.4, -61,false); // move to next 3 blocks
  Scrapper.set(true);
  TurnMaxTimePID(TestPara, -90, 0.35, true); // turn to next 3 blocks
  OdomReset(false,false,true,true);
  wait(100,msec);
  Scrapper.set(false);

  // MoveTimePID(DrivePara, -40, 0.5, 0.3, -68,false); // turns from long goal to middle blocks
  // TurnMaxTimePID(DrivePara, -68, 0.5, false); // turns to middle blocks
  std::cout<< "x: " <<CPos.x<<std::endl;
  std::cout<< "y: " <<CPos.y<<std::endl;

  driveToPoint(DrivePara, -33.1, 26.0, 80, 60, 2.6, true); // go to next 3 blocks
  Scrapper.set(true);

  // MoveEncoderPID(TurnPara, -80, 1, 0.3, -90, true); // move to middle goal
  TurnMaxTimePID(TestPara, -135, 0.2, false); // turns to middle goal
  // MoveEncoderPID(TurnPara, -60, 8.0, 0.3, -135, true); // go to middle goal
  // CStop();
  // CStop();
  // leverHalf(60);
  // wait(100,msec);
  // lock.set(false);
  // leverDown();
  // RunIndex(100);

  driveToPoint(DrivePara, -66, 3.0, 100, 90, 2.6, true); // go to between long goal and matchload
  wait(100,msec);
  TurnMaxTimePID(TurnPara, 180, 0.2, false); // turn to long goal
  std::cout<< "x2: " <<CPos.x<<std::endl;
  std::cout<< "y2: " <<CPos.y<<std::endl;
  driveToPoint(DrivePara, -62.0, 16.5, -70, -40, 2.6, false); // go into long goal
  Move(-40,-40);
  wait(250,msec);
  leverFull(80);
  Move(0,0);
  wait(100,msec);
  leverDown();
  RunIndex(100);
  std::cout<< "CPos.x: " <<CPos.x<<std::endl;
  OdomReset(false,false,true,true);
  CPos.y = (40.0-ORIGIN_Y); // 41.9-21.1= 20.8
  MoveEncoderPID(TurnPara, 80, 10.0, 0.2, 180,false); // move into matchloader
  lock.set(false);
  MoveTimePID(TurnPara, 50, 0.96, 0.3, 180,false); // slows down into matchloader
  OdomReset(false,false,true,true);
  CPos.y = -10.0;
  MoveEncoderPID(TurnPara, -80, 5.0, 0.3, 180,true); // move out of scrapper 
  TurnMaxTimePID(TurnPara, -135, 0.2, false);
  leverLift(false);
  MoveEncoderPID(TurnPara, -100, 30.5, 0.3, -135, false); // go to middle goal
  MoveEncoderPID(TurnPara, -50, 9.0, 0.3, -135, false); // go to middle goal
  wait(50,msec);
  Move(-15,-15);
  wait(50,msec);
  leverFull(60);
  wait(100,msec);
  RunIndex(-100);
  RunLever(-100);


  std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
  // MoveEncoderPID(TurnPara, 100, 30, 0.3, -135, false);
  
  wait(15,sec);

  int screenheading = Gyro.heading(degrees);
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(monoL);
  Brain.Screen.setPenColor("#808080");
  Brain.Screen.setCursor(3,10);
  Brain.Screen.print("HEADING:");
  Brain.Screen.setCursor(4,10);
  Brain.Screen.print(screenheading);

  
}