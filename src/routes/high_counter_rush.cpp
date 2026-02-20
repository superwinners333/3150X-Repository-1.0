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

void high_counter_rush() { // NEGATIVE TURNS TO THE LEFT
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

    MoveEncoderPID(TurnPara, -80, 31.5, 0.2, -125, true); // goes between there

    TurnMaxTimePID(TurnPara, -175, 0.4, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.36, 0.2, -178, false); // goes into matchload
    MoveTimePID(TurnPara, 50, 0.7, 0.2, -178, false); // slows down

    // middle goal scoring section
    MoveEncoderPID(TestPara, 70, 7.0, 0.1, -178, false); // moves backwards
    TurnMaxTimePID(TurnPara, -135, 0.2, false); // turns to face middle
    MoveEncoderPID(TestPara, 110, 34, 0.3, -135, false); // moves back into middle goal
    MoveTimePID(TestPara, -40, 0.4, 0.1, -135, false); // slows down
    MiddleScore();
    MoveTimePID(TestPara, -20, 0.3, 0.1, -135, false); // pushes into long goal
    NeutralScore();
    Scrapper.set(false);
    TurnMaxTimePID(TurnPara, -80, 0.25, false); // turns to blocks under long goal
    MoveEncoderPID(TestPara, -100, 22, 0.3, -80, false); // goes to blocks under long goal
    MoveTimePID(TestPara, -30, 0.4, 0.1, -80, false); // slows down a bit
    Scrapper.set(true); // drops scraper onto blocks
    MoveEncoderPID(TestPara, 80, 1.5, 0.2, -80, false); // backs up a bit
    Scrapper.set(false); // lifts scraper up to not get stuck
    wait(100,msec); // lets blocks enter intake and to stop scraper from being stuck
    
    // long goal scoring section
    MoveEncoderPID(TestPara, -80, 2.5, 0.2, -175, false); // move forward to curve a bit
    MoveEncoderPID(TestPara, -80, 1.5, 0.2, 176, false); // back up to align with long goal
    // maybe replace the wait below with a turn if we are not aligned
    wait(300,msec); // wait a bit for opponent to score
    Wings.set(true); 
    MoveEncoderPID(TestPara, -100, 25, 0.2, -180, false); // descore
    MoveEncoderPID(TestPara, -100, 16, 0.2, -150, false); // goes to between long goal and middle
    MoveTimePID(TestPara, -80, 0.5, 0.2, -180, false); // backs up into long goal
    HighScore();
    Wings.set(false); // raises wing so we're less likely to get stuck later
    MoveTimePID(TestPara, -30, 1.5, 0.2, -180, false); // scores

    // descoring everything section
    MoveEncoderPID(TestPara, -90, 12, 0.2, -178, false); // moves forward
    TurnMaxTimePID(TurnPara, -135, 0.2, false); // turns to face middle
    // insert middle descore function
    MoveTimePID(TestPara, -110, 1.1, 0.2, -135, false); // moves back to descore middle
    MoveEncoderPID(TestPara, -100, 22.5, 0.3, -135, false); // moves forward into wing position

    TurnMaxTimePID(TurnPara, 179, 0.4, false); // turns to wing
    Wings.set(false);
    MoveEncoderPID(TestPara, 110, 15, 0.3, 179, false); // backs up to wing
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
}