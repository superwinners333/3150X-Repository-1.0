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

void high_all() { // NEGATIVE TURNS TO THE LEFT
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

    MoveEncoderPID(TurnPara, -80, 30.5, 0.2, -125, true); // goes between there

    MoveTimePID(TurnPara, 100, 0.38, 0.2, -178, false); // goes into matchload
    MoveTimePID(TurnPara, 50, 0.8, 0.1, -178, false); // slows down

    MoveEncoderPID(TurnPara, 100, 5, 0.4, -178, false); // back out of matchload
    MoveEncoderPID(TurnPara, 100, 5, 0.4, -155, false); // curves to the side a bit
    Scrapper.set(false); // lifts up scraper
    MoveEncoderPID(TurnPara, 100, 5, 0.3, -140, false); // curves more
    MoveEncoderPID(TurnPara, 100, 4, 0.3, -160, false); // straightens out a bit
    MoveEncoderPID(TurnPara, 100, 17, 0.2, -180, true); // straightens out and backs up to somewhere nearby the line
    RunIndex(0); // stops intake so we dont overheat too much
    wait(5,sec); // waits for opponents to score

    Wings.set(true);
    wait(100,msec); // waits for wings to drop
    RunIndex(100); // starts intaking again
    MoveEncoderPID(TurnPara, -100, 12, 0.2, -180, false); // wigns out all the blocks sticking out of the control
    MoveEncoderPID(TurnPara, -100, 5, 0.2, 155, false);
    MoveEncoderPID(TurnPara, -100, 6, 0.2, 130, false); // curves to in front of middle
    TurnMaxTimePID(TurnPara, -135, 0.4, false); // turns so back faces middle
    MoveEncoderPID(TurnPara, 100, 12, 0.3, -135, false); // backs up to middle
    MoveEncoderPID(TurnPara, -40, 0.3, 0.1, -135, false); // slows down
    MiddleScore(); // score middle
    MoveEncoderPID(TurnPara, -40, 0.6, 0.1, -135, false); // push into middle


    std::cout<< "time: " <<stopwatch/1000.0<<std::endl; // should output around 14.5

    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
}