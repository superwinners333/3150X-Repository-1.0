#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "../odom.hpp"
#include "vex.h"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

// MoveDistancePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, int dir, int MaxSpd, double AccT, double ABSHDG, bool brake)
// PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)

void yahuskills() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet MovePara={1.75,0.0,0.26}; // accurate pid distK values
    PIDDataSet TurnPara={1.9,0.0,0.053}; // accurate pid headK values

    PIDDataSet UhhhPara={1.5,0.1,0.15}; // for basic turning
    PIDDataSet CorrectionPara = {1.0,0.0,0.0}; // for basic correction
    PIDDataSet DrivePara = {1.2, 0.0, 4.0}; 

    PIDDataSet TestPara={1.5,0.1,0.12};
    PIDDataSet pp={1.5,0.1,0.15};

    // AccuratePID(MovePara, TurnPara, 1.0, 5.0, 100, 5.0, 0, true);
    globalHeading = 180;
    Point target = {-12.0,24.0};
    Point target2 = {0.0,0.0};

    globalHeading = 180;
    startTracking({0.0,0.0});

    // straightToPoint(UhhhPara, CorrectionPara, MovePara, target, 100.0, 5.0, true);
    wait(50,msec);
    Gyro.setHeading(-180,degrees);
    wait(50,msec);
    MoveEncoderPID(TestPara, -90, 7, 0.1, -180, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 10, 0.1, -140, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 23, 0.1, -120, true); // curve towards park
    RunIndex(100);
    MoveTimePID(TestPara, 70, 2.6, 0.1, -95,false); // pickup blocks

    //Scrapper.set(true);

    //MoveTimePID(TestPara, 50, 1.5, 0.1, 87,false); // pickup blocks
    //MoveTimePID(TestPara, 60, 0.5, 0.1, 90,false); // pickup blocks
    MoveEncoderPID(TestPara, 90, 4, 0.1, -90, false); // back up
    MoveEncoderPID(TestPara, -90, 6, 0.1, -50, false); // curve around matchloader
    MoveTimePID(TestPara, 90, 1, 0.1, -90,false); // move into wall
    Gyro.setHeading(-90,degrees);
    //MoveEncoderPID(TestPara, 90, 4, 0.1, -90, false); // back up
    MoveEncoderPID(TestPara, 90, 3.5, 0.1,  -90, true); // turn into goal
    MoveEncoderPID(TestPara, 90, 15, 0.1,  -180, false); // turn into goal
    HighScore();
    MoveTimePID(TestPara, -20, 1, 0.1, -180,false); // move into wall
    
    








    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}