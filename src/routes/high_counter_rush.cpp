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
  driveToPoint(DrivePara, -3, 15, 100, 95, 2, false);
  levertracker.setPosition(0,degrees);
  RunLever(0);
  Scrapper.set(true);
  driveToPoint(DrivePara, -34.9, 9, 80, 35, 2.6, true);

  MoveTimePID(TestPara, 40, 1.35, 0.02, -180, false); // matchload
  OdomReset(false,false,true,true);
  CPos.y = -8.0;

  MoveEncoderPID(TurnPara, -80, 5.0, 0.3, 180,true); // move out of scrapper 
  TurnMaxTimePID(TurnPara, -135, 0.2, false);
  leverLift(false);
  MoveEncoderPID(TurnPara, -100, 30.5, 0.3, -135, false); // go to middle goal
  MoveEncoderPID(TurnPara, -50, 9.0, 0.3, -135, false); // go to middle goal
  wait(50,msec);
  Move(-15,-15);
  wait(50,msec);
  leverHalf(60);
  lock.set(false);
  wait(50,msec);
  lock.set(true);
  Scrapper.set(false);
  thread down = thread(leverDown); // lowers lever

  TurnMaxTimePID(TurnPara, -90, 0.25, false); // turn to blocks underneath long goal
  // Wings.set(false);
  MoveEncoderPID(TurnPara, 90, 24, 0.2, -90, false); // picks up blocks under long goal
  CStop();
  Scrapper.set(true);
  wait(100,msec);
  MoveEncoderPID(TurnPara, -50, 1.3, 0.2, -90, false);
  CStop();
  Scrapper.set(false);
  leverLift(true);
  // Wings.set(true);
  TurnMaxTimePID(TurnPara, -140, 0.30, false); // turn to matchload wall to prepare to wing out
  MoveEncoderPID(TurnPara, 80, 5, 0.2, -160, false); // move into long goal to wing align
  Wings.set(false); // lowers wings
  MoveEncoderPID(TurnPara, 80, 15, 0.2, -178, false); // wing out long goal
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