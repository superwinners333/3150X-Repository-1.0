#include "../movement.hpp"
#include "../helper_functions.hpp"
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
    PIDDataSet TestPara={1.5,0.1,0.15};

    timer stopwatch;

    RunIndex(100);
    MoveEncoderPID(TurnPara, -100, 10.5, 0.2, -30, false); // goes forward
    Scrapper.set(true);
    CurveEncoderPID(TurnPara, 10, -110, 10, 0.2, 0, false);
    // TurnMaxTimePID(TurnPara, -125, 0.35, false); // turns to between long goal and matchload tube

    MoveEncoderPID(TurnPara, -80, 32, 0.2, -125, true); // goes between there

    TurnMaxTimePID(TurnPara, -178, 0.2, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.38, 0.2, -178, false); // goes into matchload
    MoveTimePID(TurnPara, 50, 0.67, 0.1, -178, false); // slows down

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
    Move(-30,0);
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