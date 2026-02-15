#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);

void high_middle_wing() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet TurnPara={1.5,0.1,0.12};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet PurePara={1.5,0.1,0.12};

    timer stopwatch;

    RunIndex(100);
    MoveEncoderPID(TurnPara, -100, 10.5, 0.2, -30, false); // goes forward
    Scrapper.set(true);
    TurnMaxTimePID(TurnPara, -125, 0.35, false); // turns to between long goal and matchload tube

    MoveEncoderPID(TurnPara, -80, 27.8, 0.2, -125, true); // goes between there

    TurnMaxTimePID(TurnPara, -175, 0.4, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.25, 0.2, -178, false); // goes into matchload
    MoveTimePID(TurnPara, 50, 0.78, 0.2, -178, false); // slows down

    MoveTimePID(TurnPara, -70, 0.7, 0.2, -178, false); // goes backwards into long goal
    HighScore(); // activates long goal scoring
    wait(50,msec);
    MoveTimePID(TurnPara, -50, 0.55, 0.2, -178, false); // pushes into long goal
    NeutralScore();
    wait(50,msec);
    MoveEncoderPID(TestPara, -60, 13.4, 0.1, -178, false); // moves forward
    TurnMaxTimePID(TurnPara, -135, 0.4, false); // turns to face middle
    MoveEncoderPID(TestPara, 100, 32, 0.3, -135, false); // moves back into middle goal
    MoveTimePID(TestPara, -40, 0.5, 0.1, -135, false); // slows down
    MiddleScore();
    wait(600,msec);
    NeutralScore();
    MoveEncoderPID(TestPara, -100, 22.5, 0.3, -135, false); // moves forward into wing position

    TurnMaxTimePID(TurnPara, 179, 0.4, false); // turns to wing
    Wings.set(false);
    MoveEncoderPID(TestPara, 100, 16, 0.6, 179, false); // backs up to wing
    wait(200,msec);
    Move(-30,0);
    wait(100,msec);
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
    wait(2000,msec);
}