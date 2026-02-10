#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "../odom.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

// MoveDistancePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, int dir, int MaxSpd, double AccT, double ABSHDG, bool brake)
// PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)

void mangoskills() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet MovePara={1.75,0.0,0.26}; // accurate pid distK values
    PIDDataSet TurnPara={1.9,0.0,0.053}; // accurate pid headK values

    PIDDataSet TestPara={1.5,0.1,0.15}; // for basic turning
    PIDDataSet CorrectionPara = {1.0,0.0,0.0}; // for basic correction
    PIDDataSet DrivePara = {1.2, 0.0, 4.0}; 

    PIDDataSet APara = {1.2,0.5,0.26};
    PIDDataSet BPara = {1.0,0.0,0.0}; // for basic correction
    PIDDataSet CPara = {1.3,0.0,0.1};


    // AccuratePID(MovePara, TurnPara, 1.0, 5.0, 100, 5.0, 0, true);
    globalHeading = 0;
    Point target = {-12.0,24.0};
    Point target2 = {0.0,0.0};

    Point places[4] = {{-6.0,12.0},{-8.0,11.0},{-28.0,0.0},{-29.0,-7.0}}; // creates an array of points
    double placelength = sizeof(places) / sizeof(places[0]); // gets number of elements in the array

    startTracking({0.0,0.0});


    // curveToPoint(target, 90.0, 2.0, true);

    straightToPoint(TestPara, CorrectionPara, MovePara, target, 100.0, 5.0, true);
    // straightToPoint(TestPara, BPara, MovePara, target2, -100.0, 5.0, true);
    
    wait(50,msec);





    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}