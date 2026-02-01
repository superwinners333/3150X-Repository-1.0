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
    PIDDataSet PurePara={1.5,0.1,0.12};

    timer stopwatch;

    RunIndex(100);
    MoveEncoderPID(TurnPara, -100, 10.5, 0.2, -30, false); // goes forward
    Scrapper.set(true);
    TurnMaxTimePID(TurnPara, -125, 0.35, false); // turns to between long goal and matchload tube

    MoveEncoderPID(TurnPara, -80, 26.0, 0.2, -125, true); // goes between there

    TurnMaxTimePID(TurnPara, -175, 0.4, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.25, 0.2, -178, false); // goes into matchload
    MoveTimePID(TurnPara, 50, 0.92, 0.2, -178, false); // slows down

    MoveTimePID(TurnPara, -80, 0.7, 0.2, -178, false); // goes backwards into long goal
    HighScore(); // activates long goal scoring
    wait(50,msec);
    MoveTimePID(TurnPara, -50, 1.7, 0.2, -178, false); // pushes into long goal

    // wing code
    MoveEncoderPID(TestPara, -90, 10, 0.4, 178, false); // goes away from long goal
    Wings.set(false); // lowers wings
    wait(100,msec);
    MoveEncoderPID(TestPara, 100, 8.65, 0.4, -160, false); // goes to the side of long goal a bit

    MoveEncoderPID(TestPara, 100, 14.5, 0.6, 179, false); // backs up to wing
    wait(200,msec);
    Move(-30,0);
    wait(100,msec);
    std::cout<<stopwatch/1000.0<<std::endl;
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