#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include "../odom.hpp"
#include "../Odometry.hpp"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

// MoveDistancePID(PIDDataSet KDist, PIDDataSet KTurn, double dist, double ABSHDG, bool brake)
// PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)

void high_counter_rush() { // NEGATIVE TURNS TO THE LEFT
  PIDDataSet TurnPara={1.5,0.1,0.12};
  PIDDataSet TestPara={2,0.1,0.3};
  PIDDataSet DrivePara={2.4,0.12,0.1};
  PIDDataSet curvePara={1.9,0.1,0.24};

  timer stopwatch;
  ORIGIN_Y = 24.75;
  ORIGIN_X = 56.5;

  RunIndex(100);
  RunLever(-100);
  driveToPointantiOrbit(DrivePara, -3, 16, 100, 95, 2, false); // 3 blocks in middle
  levertracker.setPosition(0,degrees);
  RunLever(0);
  Scrapper.set(true);
  driveToPointantiOrbit(DrivePara, -31.0, 3.5, 80, 40, 2.6, true); // go to between long goal and matchload
  MoveTimePID(TestPara, 40, 1.25, 0.02, -180, false); // matchload
  OdomReset(false,false,true,true);
  CPos.y = -10.0;

  MoveEncoderPID(TurnPara, -80, 7.1, 0.3, 180,true); // move out of scrapper 
  TurnMaxTimePID(TurnPara, -135, 0.2, false);
  leverLift(false);
  Scrapper.set(false);
  MoveEncoderPID(TurnPara, -67, 31, 0.3, -135, false); // go to middle goal
  RunIndex(0);
  MoveTimePID(TestPara, -35, 0.4, 0.02, -135, false);
  // MoveEncoderPID(TurnPara, -60, 30, 0.3, -135, false); // go to middle goal
  // MoveEncoderPID(TurnPara, -40, 10.0, 0.3, -135, false); // go to middle goal
  RunIndex(100);
  Move(-15,-15);
  wait(50,msec);
  leverHalf(50);
  leverDown(); // lowers lever
  wait(50,msec);
  lock.set(false);
  wait(50,msec);
  TurnMaxTimePID(TurnPara, -60, 0.25, false); // turn to blocks underneath long goal
  RunIndex(100);
  MoveEncoderPID(TurnPara, 30, 4.8, 0.1, -60, true);
  MoveEncoderPID(TurnPara, 40, 21, 0.1, -87, false); // picks up blocks under long goal
  leverLift(true);
  Scrapper.set(true);
  BStop();
  wait(300,msec);
  Scrapper.set(false);
  wait(200,msec);
  MoveEncoderPID(TurnPara, -50, 1.4, 0.2, -45, false); // back up
  CStop();
  
  TurnMaxTimePID(TurnPara, 177.5, 0.4, false); // turn to matchload wall to prepare to wing out
  CStop();
  wait(900,msec);
  std::cout<< "descore: " <<stopwatch/1000.0<<std::endl;
  Wings.set(false); // lowers wings
  MoveEncoderPID(TurnPara, 80, 16, 0.2, 179, false); // wing out long goal
  MoveEncoderPID(TurnPara, 80, 8, 0.2, -145, false); // go to between matchload and long goal
  MoveEncoderPID(TurnPara, 80, 9.5, 0.01, -90, false);
  TurnMaxTimePID(TurnPara, 180, 0.25, true); // turn to long goal
  OdomReset(false,false,true,true);
  CPos.y = 5.0;
  driveToPointantiOrbit(DrivePara, -34.3, 25, -60, -40, 2.5, false); // go into long goal
  Move(-20,-20);
  wait(100,msec);
  leverFull(70);
  CPos.y = (41.0-24.75);
  CStop();
  OdomReset(false,false,true,true);

  // wing code
  MoveEncoderPID(TurnPara, 100, 4.5, 0.2, 170, false); // move forward
  TurnMaxTimePID(TurnPara, -150, 0.2, false); // turn
  Scrapper.set(false);
  MoveEncoderPID(TurnPara, -100, 4, 0.3, -150, false); // back up to the side of long goal
  Wings.set(false); // lowers wings
  RunLever(-100);
  RunIndex(-100);
  TurnMaxTimePID(TurnPara, -180, 0.2, false); // turns to wing
  MoveEncoderPID(TurnPara, -55, 13.4, 0.2, 179, false); // backs up to wing
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
    // declare initial conditions
    PIDDataSet TurnPara={1.5,0.1,0.12};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet PurePara={1.5,0.1,0.12};

    timer stopwatch;

    RunIndex(100);
    MoveEncoderPID(TurnPara, -100, 10.5, 0.2, -30, false); // goes forward
    Scrapper.set(true);
    CurveEncoderPID(TurnPara, 10, -100, 10, 0.2, 0, false);
    // TurnMaxTimePID(TurnPara, -125, 0.35, false); // turns to between long goal and matchload tube

    MoveEncoderPID(TurnPara, -80, 31.3, 0.2, -125, true); // goes between there

    TurnMaxTimePID(TurnPara, -175, 0.4, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.38, 0.2, -178, false); // goes into matchload
    MoveTimePID(TurnPara, 50, 0.6, 0.2, -178, false); // slows down

    // middle goal scoring section
    MoveEncoderPID(TestPara, 70, 4, 0.1, -178, false); // moves backwards
    TurnMaxTimePID(TurnPara, -135, 0.2, false); // turns to face middle
    MoveEncoderPID(TestPara, 110, 30, 0.3, -135, false); // moves back into middle goal
    RunIndex(100);
    MoveTimePID(TestPara, -40, 0.4, 0.1, -135, false); // slows down
    MiddleScore();
    MoveTimePID(TestPara, -40, 0.52, 0.1, -135, false); // pushes into middle goal
    NeutralScore();
    Scrapper.set(false);
    TurnMaxTimePID(TurnPara, -81, 0.35, false); // turns to blocks under long goal
    MoveEncoderPID(TestPara, -100, 14.5, 0.3, -81, false); // goes to blocks under long goal
    MoveTimePID(TestPara, 25, 0.50, 0.1, -87, false); // slows down a bit
    // Scrapper.set(true); // drops scraper onto blocks
    wait(300,msec);
    MoveEncoderPID(TestPara, 80, 0.98, 0.3, -74, false); // backs up a bit
    // wait(150,msec);
    // Scrapper.set(false); // lifts scraper up to not get stuck
    // wait(200,msec); // lets blocks enter intake and to stop scraper from being stuck
    
    // long goal scoring section
    TurnMaxTimePID(TestPara, -177, 0.5, false);
    // maybe replace the wait below with a turn if we are not aligned
    wait(100,msec); // wait a bit for opponent to score
    Wings.set(false); 
    MoveEncoderPID(TestPara, -80, 3, 0.2, -177, false); // align with long goal
    
    MoveEncoderPID(TestPara, -100, 12, 0.2, -180, false); // descore
    MoveEncoderPID(TestPara, -100, 2, 0.2, 170, false); // descore
    Wings.set(true);  // raises wing so we're less likely to get stuck later
    MoveEncoderPID(TestPara, -100, 5, 0.2, -140, false); // goes to between long goal and matchload
    MoveEncoderPID(TestPara, -100, 10, 0.2, -100, false);
    MoveEncoderPID(TestPara, -100, 4.6, 0.2, -90, false);
    MoveTimePID(TestPara, -85, 0.53, 0.2, -180, false); // backs up into long goal
    HighScore();
    MoveTimePID(TestPara, -45, 1.5, 0.2, -180, false); // scores

    // descoring everything section
    MoveEncoderPID(TestPara, -90, 5.7, 0.2, -178, false); // moves forward
    TurnMaxTimePID(TurnPara, -135, 0.2, false); // turns to face middle
    Funnel.set(true);
    MoveEncoderPID(TestPara, 110, 25, 0.2, -135, false); // moves back to descore middle
    MoveTimePID(TestPara, -80, 0.3, 0.2, -135, false); // moves back to descore middle
    MoveEncoderPID(TestPara, -110, 20.5, 0.4, -135, false); // moves forward into wing position
    Funnel.set(false);
    TurnMaxTimePID(TurnPara, 179, 0.4, false); // turns to wing
    Wings.set(false);
    MoveEncoderPID(TestPara, 110, 14, 0.3, 179, true); // backs up to wing
    wait(150,msec);
    Move(-30,0);
    wait(100,msec);
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;

    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    */
}

 // PIDDataSet TestPara={2,0.1,0.3};
  // PIDDataSet curvePara={1.9,0.1,0.24};
  // PIDDataSet DrivePara={2.3,0.12,0.1};
  // ORIGIN_X = 56.5;
  // ORIGIN_Y = 23.5;
  // RunIndex(100);
  // driveToPointantiOrbit(DrivePara, -2.3,15, 100, 90, 2, false);
  
  // Scrapper.set(true);
  // driveToPointantiOrbit(TestPara, -30.8,-4.3, 95, 45, 2, false, 10.0);
  // OdomReset(false,false, true, true);
  // MoveTimePID(TestPara, 40, 0.8, 0.02, -180, false);
  // CPos.y = -10;
  // OdomReset(false,false, true, true);
  // leverLift(false);
  // Scrapper.set(false);
  // driveToPointantiOrbit(TestPara, -0.5,33, -100, -25, 2.6, false,16);
  // MoveTimePID(TestPara, -30, 0.3, 0.02, -135, true);
  // leverHalf(50);
  // CPos.x = 1;
  // CPos.y = 35;
  // leverDown();
  // TurnMaxTimePID(TestPara, -79, 0.3, false);
  // MoveEncoderPID(TestPara, 80, 16.2, 0.3, -79, false);
  // MoveTimePID(TestPara, 40, 0.2, 0.02, -87, false);

  // leverLift(true);
  // Scrapper.set(true);
  // lock.set(false);

  // TurnMaxTimePID(TestPara, -184, 0.5, true);
  // OdomReset(true,false, true, true);
  // Wings.set(false);
  // Scrapper.set(false);
  // MoveEncoderPID(TestPara, 70, 10, 0.3, 178, false);

  // MoveEncoderPID(TestPara, 70, 25, 0.02, -135, true);
  // TurnMaxTimePID(TestPara, 180, 0.7, false);
  // MoveTimePID(TestPara, -60, 0.5, 0.3, 180, false);
  // Move(-30,-30);
  // leverFull(100);
  // wait(100,msec);
  // leverDown();
  // MoveEncoderPID(TestPara, 80, 8, 0.3, 180, false);
  // TurnMaxTimePID(TestPara, -133, 0.4, false);
  // leverLift(false);
  // MoveEncoderPID(TestPara, -100, 45, 0.02, -133, false);
  // MoveTimePID(TestPara, -40, 0.4, 0.02, -133, true);   
  // CPos.x = 1;
  // CPos.y = 35;
  // MoveEncoderPID(TestPara, 100, 16, 0.3, -133, false);
  // leverLift(true);
  // TurnMaxTimePID(TestPara, 180, 0.4, false);
  // MoveEncoderPID(TestPara, -70, 22, 0.4, 180, true);

  // wait(15,sec);