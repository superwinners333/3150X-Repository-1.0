#include "../movement.hpp"
#include "../odom.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include <iostream>
#include <vector>
#include <utility>
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);



void push_awp() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    //PIDDataSet TurnPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};

    PIDDataSet MPara={1.75,0.0,0.26}; // accurate pid distK values
    PIDDataSet TPara={1.9,0.0,0.053}; // accurate pid headK values
    // SIXSEVEEN 77777777777777777777
    timer stopwatch;
    NeutralScore();
    RunIndex(100);

    MoveEncoderPID(TurnPara, -40, 4, 0.2, 0,true); // drives to mathcloader
    ChassisDataSet BackSenVals = ChassisUpdate(); // gets back distance sensor vals
    AccuratePID(MPara,TPara, BackSenVals.backD-13.0, 5.0, 100, 5.0, 0.0, true); // mvoes to between long goal and matchload
    MoveEncoderPID(TurnPara, -70, 19.1, 0.3, 0,true); // drives to mathcloader
    Scrapper.set(true);

    TurnMaxTimePID(TurnPara, -90, 0.3, true); // turns to matchloader
    MoveTimePID(TurnPara, 40, 0.85, 0.2, -90,false); // move into matchloader
    MoveTimePID(TurnPara, -75, 0.9, 0.3, -90,false); // move backwards to long goal
    HighScore();
    wait(50,msec);
    MoveTimePID(TurnPara, -10, 1.0, 0.3, -90,false); // move into long goal
    Scrapper.set(false);
    MoveEncoderPID(TurnPara, -70, 2.5, 0.3, -90,true); // go away from long goal 
    TurnMaxTimePID(TurnPara, 25, 0.2, false); // turns left to face 3 blocks in middle

    NeutralScore();
    MoveEncoderPID(TurnPara, -60, 26, 0.2, 25,false); // gets 3 blocks
    MoveEncoderPID(TestPara, -100, 23, 0.2, 10,false); // curves into move to other side
    MoveEncoderPID(TestPara, -100, 7, 0.2, -25,false); // curves into other 3 blocks
    Scrapper.set(true); // activates scraper
    MoveEncoderPID(TestPara, -100, 21.0, 0.2, -45,true); // goes to between long goal and matchload
    TurnMaxTimePID(TurnPara, -90, 0.2, false); // turns to score on long goal

    MoveTimePID(TestPara, -60, 0.3, 0.1, -90,false); // move to long goal
    HighScore();
    wait(50,msec);
    RunIndex(90);
    MoveTimePID(TestPara, -25, 1.5, 0.4, -90,true); // score
    NeutralScore();
    MoveTimePID(TurnPara, 100, 0.50, 0.2, -90, false); // goes into matchload
    MoveTimePID(TurnPara, 40, 0.80, 0.2, -90, false); // slows down

    MoveEncoderPID(TestPara, 70, 7.0, 0.1, -90, false); // moves backwards
    TurnMaxTimePID(TurnPara, -45, 0.4, false); // turns to face middle
    MoveEncoderPID(TestPara, 100, 34, 0.3, -45, false); // moves back into middle goal
    MoveTimePID(TestPara, -40, 0.4, 0.1, -45, false); // slows down
    MiddleScore();
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
    wait(600,msec);
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;

    // NeutralScore();
    // MoveEncoderPID(TurnPara, -40, 25, 0.3, -155,true); // gets 3 blocks
    // TurnMaxTimePID(TurnPara, 178, 0.2, false); // turns to other 3 blocks
    // MoveEncoderPID(TestPara, -100, 27, 0.3, 178,false); // move to other side
    // MoveEncoderPID(TestPara, -40, 14.0, 0.3, 178,true); // pick up other 3 balls
    // Scrapper.set(true); // activates scraper
    // TurnMaxTimePID(TurnPara, 135, 0.2, true); // turns to middle goal
    // MoveTimePID(TurnPara, -45, 0.78 , 0.3, 135,false); // move into middle goal
    // RunIndex(60);
    // MiddleScore();
    // wait(790,msec);
    // NeutralScore();
    // MoveEncoderPID(TurnPara, -80, 40.5, 0.3, 135, true); // moves to between matchload tube and long goal
    // TurnMaxTimePID(TurnPara, 90, 0.2, true); // turn so scraper faces matchload tube
    // NeutralScore();
    // MoveTimePID(TurnPara, 50, 1.2 , 0.4, 90,false); // move into matchloader
    // MoveTimePID(TurnPara, -73, 0.95, 0.4, 90,false); // moves backwards into long goal
    // HighScore();
    // RunIndex(100);
    // wait(75,msec);
    // MoveTimePID(TurnPara, -10, 1.1, 0.4, 90,false); // moves further towards long goal
    // Scrapper.set(false); // turns off scrapper in prep for driver
    // wait(1000,msec);
}