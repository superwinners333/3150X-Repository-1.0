#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "../odom.hpp"
#include "vex.h"
#include <iostream>
#include <vector>
#include <utility>
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);



void solo_awp() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet TurnPara={1.5,0.1,0.12};
  PIDDataSet TestPara={2,0.1,0.3};
  PIDDataSet DrivePara={2.4,0.12,0.1};
  PIDDataSet curvePara={1.9,0.1,0.24};

  timer stopwatch;
  // CPos.y = 24.75;
  // CPos.x = 56.5;

  RunIndex(100);
  RunLever(-100);
  driveToPoint(DrivePara, 0, 20, 100, 95, 2, false);
  levertracker.setPosition(0,degrees);
  RunLever(0);
  Scrapper.set(true);
  TurnMaxTimePID(TurnPara, 90, 0.3, false); // turns to matchloader
  MoveTimePID(TurnPara, 50, 1.0, 0.2, 90,false); // move into matchloader
  driveToPoint(DrivePara, -16.5, 32, -70, -40, 2.6, true); // go into long goal
  Move(-40,-40);
  leverFull(100);
  Move(0,0);
  CPos.x = -18.0;
  std::cout<< "CPos.x: " <<CPos.x<<std::endl;
  Scrapper.set(false);
  RunIndex(100);
  MoveEncoderPID(TurnPara, 60, 0.2, 0.1, 90,false); // move out of long goal
  TurnMaxTimePID(TurnPara, -155, 0.25, false); // turn to next 3 blocks
  MoveEncoderPID(TurnPara, 70, 24, 0.5, -155,false); // move to next 3 blocks
  TurnMaxTimePID(TestPara, -180, 0.25, false); // turn to next 3 blocks
  driveToPoint(DrivePara, -20, -31.5, 100, 90, 2.6, true);
  std::cout<< "CPos.x: " <<CPos.x<<std::endl;
  Scrapper.set(true);
  leverLift(false);
  TurnMaxTimePID(TurnPara, 135, 0.2, false); // turn to middle goal
  driveToPoint(DrivePara, -28.5, -28, -80, -70, 2.6, true); // go to middle goal
  wait(300,msec);
  leverFull(65);
  leverLift(true);
  RunIndex(-100);
  RunLever(-100);
  driveToPoint(DrivePara, -3, -57, 100, 90, 2.6, true);
  TurnMaxTimePID(TurnPara, 90, 0.2, false); // turn matchload
  RunLever(0);
  RunIndex(100);
  MoveTimePID(TurnPara, 50, 1.0, 0.2, 90,false); // move into matchloader
  driveToPoint(DrivePara, -17, -57, -70, -40, 2.6, true);
  leverFull(100);
  wait(15,sec);


  wait(50,msec);
  Move(-40,-40);
  leverFull(80);
  Move(0,0);
  wait(50,msec);
  CPos.y = (41.0-24.75);
  CPos.x = (23.0-56.5);
  wait(50,msec);
  // driveToPoint(DrivePara, -28, 10, -80, -20, 2.6, false);
  MoveEncoderPID(TurnPara, 100, 2, 0.2, 160, false);
  MoveEncoderPID(TurnPara, 110, 9, 0.3, 90, false); 
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
//PIDDataSet TurnPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};
    PIDDataSet Hard={3.0,0.1,0.1};
    PIDDataSet straight={1.0,0.0,0.0};
    // SIXSEVEEN 77777777777777777777
    timer stopwatch;
    NeutralScore();
    MoveEncoderPID(TurnPara, -70, 19.1, 0.3, 0,true); // drives to mathcloader
    Scrapper.set(true);
    RunIndex(100);
    TurnMaxTimePID(TurnPara, 90, 0.3, true); // turns to matchloader
    MoveTimePID(TurnPara, 43, 0.85, 0.2, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -75, 0.9, 0.3, 90,false); // move backwards to long goal
    HighScore();
    wait(30,msec);
    // Gyro.setHeading(90,degrees);
    MoveTimePID(TurnPara, -10, 1.0, 0.3, 90,false); // move into long goal
    Scrapper.set(false);
    MoveEncoderPID(TurnPara, -70, 2.5, 0.3, 90,true); // go away from long goal 
    TurnMaxTimePID(TurnPara, -152, 0.3, false); // turns left


    NeutralScore();
    MoveEncoderPID(TurnPara, -50, 25, 0.3, -152,true); // gets 3 blocks
    TurnMaxTimePID(Hard, 178, 0.3, false); // turns to other 3 blocks
    std::cout<< "heading 1: " <<Gyro.heading(degrees)<<std::endl;
    MoveEncoderPID(TurnPara, -100, 25, 0.3, 180,false); // move to other side
    MoveEncoderPID(TurnPara, -50, 6, 0.1, 180,false); // slows down
    std::cout<< "heading 2: " <<Gyro.heading(degrees)<<std::endl;
    Scrapper.set(true); // activates scraper
    wait(50,msec); // lets us coast a bit
    TurnMaxTimePID(TurnPara, 135, 0.2, true); // turns to long goal
    MoveEncoderPID(TestPara, -90, 23.5, 0.3, 135,true); // goes to between long goal and matchload
    TurnMaxTimePID(TurnPara, 90, 0.24, false); // turns to score on long goal

    MoveTimePID(TestPara, -60, 0.3, 0.1, 90,false); // move to long goal
    HighScore();
    wait(30,msec);
    RunIndex(100);
    MoveTimePID(TestPara, -50, 1.5, 0.1, 89,true); // score
    // Gyro.setHeading(90,degrees);
    NeutralScore();
    MoveTimePID(straight, 100, 0.40, 0.2, 89, false); // goes into matchload
    MoveTimePID(straight, 43, 0.95, 0.2, 89, false); // slows down

    MoveEncoderPID(TestPara, 70, 5.1, 0.1, 90, false); // moves backwards
    TurnMaxTimePID(TurnPara, 135, 0.4, false); // turns to face middle
    MoveEncoderPID(TestPara, 100, 29, 0.3, 135, false); // moves back into middle goal
    MoveTimePID(TestPara, -40, 0.4, 0.1, 135, false); // slows down
    // MoveTimePID(TestPara, -30, 0.1, 0.1, 110, false); // slows down
    // MoveTimePID(TestPara, -30, 0.1, 0.1, 160, false); // slows down
    // MoveTimePID(TestPara, -30, 0.1, 0.1, 135, false);
    MiddleScore();
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
    wait(100,msec);
    RunIndex(60);
    wait(300,msec);
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
    */