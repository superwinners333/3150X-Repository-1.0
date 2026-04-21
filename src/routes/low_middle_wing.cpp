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

void low_middle_wing() { // NEGATIVE TURNS TO THE LEFT
  PIDDataSet TurnPara={1.5,0.1,0.12};
  PIDDataSet TestPara={2,0.1,0.3};
  PIDDataSet DrivePara={2.4,0.12,0.1};
  PIDDataSet curvePara={1.9,0.1,0.24};

  timer stopwatch;
  ORIGIN_Y = 24.75;
  ORIGIN_X = 140-56.5;

  RunIndex(100);
  RunLever(-100);
  driveToPoint(DrivePara, 3, 15, 100, 95, 2, false);
  levertracker.setPosition(0,degrees);
  RunLever(0);
  Scrapper.set(true);
  driveToPoint(DrivePara, 35, 7.5, 80, 35, 2.6, true);
  // wait(100,msec);
  // std::cout<< CPos.x <<std::endl;
  MoveTimePID(TestPara, 40, 1.2, 0.02, -180, false); // matchload
  OdomReset(false,false,true,true);
  CPos.y = -10.0;
  driveToPoint(DrivePara, 33.7, 14.9, -80, -40, 2.6, false); // go into long goal
  // MoveTimePID(TestPara, 50, 0.15, 0.02, -180, false); // go back and forht to shake the blocks
  // MoveTimePID(TestPara, -60, 0.1, 0.02, -180, false);
  Move(-40,-40);
  wait(200,msec);
  leverHalf(60);
  Move(0,0);
  lock.set(false);
  wait(100,msec);
  lock.set(true);
  wait(50,msec);
  CPos.y = (41.0-24.75);
  OdomReset(false,false,true,true);
  thread down = thread(leverDown); // lowers lever
  wait(50,msec);
  // driveToPoint(DrivePara, -24, 10, 80, 20, 2.6, false);

  MoveEncoderPID(TurnPara, 100, 8, 0.2, 180, true); // Move away from long goal
  TurnMaxTimePID(TurnPara, -45, 0.2, false); // turn to low goal
  lock.set(false);
  Scrapper.set(false);
  driveToPointantiOrbit(DrivePara, -2.1, 26.5, 80, 40, 2.6, true); // go into low goal
  // MoveEncoderPID(TurnPara, 100, 35, 0.3, -45, false); // move to low goal
  // MoveEncoderPID(TurnPara, 40, 7, 0.1, -45, false); // slow down
  Move(20,20);
  wait(100,msec);
  RunIndex(-50); // outake to score

  MoveTimePID(TestPara, 40, 0.6, 0.02, -45, false); // score in low goal
  MoveEncoderPID(TurnPara, -100, 18, 0.2, -45, false); // back up from low goal
  RunIndex(0); // stop intake

  // wing code
  Wings.set(false); // lowers wings

  TurnMaxTimePID(TurnPara, 0, 0.13, true); // turns to wing
  MoveEncoderPID(TurnPara, 80, 14, 0.2, 0, false); // goes forward up to wing
  RunLever(0);
  RunIndex(0);
  Move(-60,40);
  wait(500,msec);
  BStop();

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
    // declare initial conditions
    //PIDDataSet TurnPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};
    // SIXSEVEEN 77777777777777777777
    NeutralScore();
    MoveEncoderPID(TurnPara, -80, 14, 0.3, 0,false); // drives to mathcloader
    Scrapper.set(true);
    RunIndex(70);
    TurnMaxTimePID(TurnPara, 90, 0.3, true); // turns to matchloader
    MoveTimePID(TurnPara, 45, 1.1 , 0.3, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -80, 1, 0.3, 90,false); // move backwards to long goal
    HighScore();
    MoveTimePID(TurnPara, -20, 1.25, 0.3, 90,false); // move into long goal
    Scrapper.set(false);
    MoveEncoderPID(TurnPara, -80, 1.5 , 0.3, 90,true); // back up from long goal 
    TurnMaxTimePID(TurnPara, -152, 0.2, false); // turns left
    //MoveEncoderPID(TurnPara, -70, 7, 0.2, -180,true); // moves forward to get into a better position
    //TurnMaxTimePID(TurnPara, -140, 0.2, true); // turns to blocks
    NeutralScore();
    MoveEncoderPID(TurnPara, -70, 23.8, 0.4, -152,false); // gets 3 blocks
    RunIndex(40);
    TurnMaxTimePID(TurnPara, -135, 0.2, true); // tunr to boptlmtom foal;
    MoveEncoderPID(TurnPara, -80, 6, 0.4, -135,true); // goes to low goal
    RunIndex(-40); // outakes
    wait(1000,msec);
    MoveEncoderPID(TurnPara, 80, 22.2, 0.4, -145,true); // backs up to prepare wing
    TurnMaxTimePID(TurnPara, -90, 0.2, true); // turns so wing is facing goal
    RunIndex(0);
    Wings.set(false);
    MoveEncoderPID(TurnPara, -100, 11.2, 0.3, -90,false); // wing
    TurnMaxTimePID(TurnPara, -155, 0.2, false); // tturn
    */
}