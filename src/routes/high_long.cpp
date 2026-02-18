#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void high_long() { // NEGATIVE TURNS TO THE LEFT
    timer stopwatch;
    // declare initial conditions
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};
    RunIndex(100);
    MoveEncoderPID(TurnPara, -100, 11, 0.2, -30, false); // goes forward
    // Scrapper.set(true);
    MoveEncoderPID(TurnPara, -100, 18.6, 0.2, -38, false); // curves towards blocks under long
    // Scrapper.set(false);
    MoveEncoderPID(TurnPara, -100, 10.1, 0.2, -85, false); // goes forward toward blocks
    Scrapper.set(true);
    MoveTimePID(TurnPara, 100, 0.15, 0.2, -85, false);
    MoveEncoderPID(TurnPara, 100, 4, 0.2, -65, false); // moves away
    MoveEncoderPID(TurnPara, 100, 16, 0.2, 20, false); // goes back
    MoveEncoderPID(TurnPara, 100, 13.75, 0.2, 65, false); // goes back towards long goal and matchload
    TurnMaxTimePID(TurnPara, -180, 0.25, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.36, 0.2, -180, false); // goes into matchload
    MoveTimePID(TurnPara, 50, 0.4, 0.2, -180, false); // slows down
    wait(100,msec);
    MoveTimePID(TurnPara, 100, 0.3, 0.2, -180, false); // rams in again
    // HighScore();
    // wait(20,msec);
    // NeutralScore();
    MoveTimePID(TurnPara, -80, 0.7, 0.2, 176, false); // goes backwards into long goal
    HighScore(); // activates long goal scoring
    MoveTimePID(TurnPara, -40, 1.6, 0.2, 180, false); // pushes into long goal

    // wing code
    MoveEncoderPID(TestPara, -90, 9.2, 0.4, 177, false); // goes away from long goal
    Wings.set(false); // lowers wings
    wait(100,msec);
    NeutralScore(); // stops rolling block violations

    MoveEncoderPID(TestPara, 100, 8.7, 0.4, -159, false); // goes to the side of long goal a bit

    MoveEncoderPID(TestPara, 100, 14, 0.6, 179, false); // backs up to wing
    wait(150,msec);
    Move(-35,0);
    wait(100,msec);
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
    /*
    MoveEncoderPID(TestPara, -90, 7, 0.1, 0, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 10, 0.1, 40, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 23, 0.1, 60, true); // curve towards park
    RunIndex(100);
    MoveEncoderPID(TestPara, -80, 35, 0.1, 90, true); // park
    */
}