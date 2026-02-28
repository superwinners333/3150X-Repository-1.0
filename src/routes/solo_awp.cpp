#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include <iostream>
#include <vector>
#include <utility>
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);



void solo_awp() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    //PIDDataSet TurnPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};
    PIDDataSet Hard={2.6,0.1,0.1};
    PIDDataSet straight={1.0,0.0,0.0};
    // SIXSEVEEN 77777777777777777777
    timer stopwatch;
    NeutralScore();
    MoveEncoderPID(TurnPara, -70, 19.1, 0.3, 0,true); // drives to mathcloader
    Scrapper.set(true);
    RunIndex(100);
    TurnMaxTimePID(TurnPara, 90, 0.3, true); // turns to matchloader
    MoveTimePID(TurnPara, 40, 0.85, 0.2, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -75, 0.9, 0.3, 90,false); // move backwards to long goal
    HighScore();
    wait(30,msec);
    Gyro.setHeading(90,degrees);
    MoveTimePID(TurnPara, -10, 1.0, 0.3, 90,false); // move into long goal
    Scrapper.set(false);
    MoveEncoderPID(TurnPara, -70, 2.5, 0.3, 90,true); // go away from long goal 
    TurnMaxTimePID(TurnPara, -152, 0.3, false); // turns left


    NeutralScore();
    MoveEncoderPID(TurnPara, -50, 25, 0.3, -152,true); // gets 3 blocks
    TurnMaxTimePID(Hard, 178, 0.3, false); // turns to other 3 blocks
    std::cout<< "heading 1: " <<Gyro.heading(degrees)<<std::endl;
    MoveEncoderPID(TurnPara, -100, 25, 0.3, 180,false); // move to other side
    MoveEncoderPID(TurnPara, -50, 6, 0.1, 180,false); // slows down
    std::cout<< "heading 2: " <<Gyro.heading(degrees)<<std::endl;
    Scrapper.set(true); // activates scraper
    wait(50,msec); // lets us coast a bit
    TurnMaxTimePID(TurnPara, 135, 0.2, true); // turns to long goal
    MoveEncoderPID(TestPara, -90, 22.4, 0.3, 135,true); // goes to between long goal and matchload
    TurnMaxTimePID(TurnPara, 90, 0.24, false); // turns to score on long goal

    MoveTimePID(TestPara, -60, 0.3, 0.1, 90,false); // move to long goal
    HighScore();
    wait(30,msec);
    RunIndex(100);
    MoveTimePID(TestPara, -50, 1.5, 0.1, 89,true); // score
    // Gyro.setHeading(90,degrees);
    NeutralScore();
    MoveTimePID(straight, 100, 0.40, 0.2, 89, false); // goes into matchload
    MoveTimePID(straight, 40, 0.9, 0.2, 89, false); // slows down

    MoveEncoderPID(TestPara, 70, 5.1, 0.1, 90, false); // moves backwards
    TurnMaxTimePID(TurnPara, 135, 0.4, false); // turns to face middle
    MoveEncoderPID(TestPara, 100, 29, 0.3, 135, false); // moves back into middle goal
    MoveTimePID(TestPara, -40, 0.4, 0.1, 135, false); // slows down
    // MoveTimePID(TestPara, -30, 0.1, 0.1, 110, false); // slows down
    // MoveTimePID(TestPara, -30, 0.1, 0.1, 160, false); // slows down
    // MoveTimePID(TestPara, -30, 0.1, 0.1, 135, false);
    MiddleScore();
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
    wait(100,msec);
    RunIndex(70);
    wait(300,msec);
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