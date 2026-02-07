#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);

void solo_awp() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    //PIDDataSet TurnPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};
    // SIXSEVEEN 77777777777777777777
    NeutralScore();
    MoveEncoderPID(TurnPara, -70, 19.3 , 0.3, 0,true); // drives to mathcloader
    Scrapper.set(true);
    RunIndex(100);
    TurnMaxTimePID(TurnPara, 90, 0.3, true); // turns to matchloader
    MoveTimePID(TurnPara, 50, 0.92 , 0.3, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -75, 0.9, 0.3, 90,false); // move backwards to long goal
    HighScore();
    MoveTimePID(TurnPara, -10, 1, 0.3, 90,false); // move into long goal
    Scrapper.set(false);
    MoveEncoderPID(TurnPara, -70, 0.15, 0.3, 90,true); // go away from long goal 
    TurnMaxTimePID(TurnPara, -155, 0.2, false); // turns left


    NeutralScore();
    MoveEncoderPID(TurnPara, -40, 26.3, 0.3, -155,true); // gets 3 blocks
    TurnMaxTimePID(TurnPara, 178, 0.2, false); // turns to other 3 blocks
    MoveEncoderPID(TestPara, -100, 28, 0.3, 178,false); // move to other side
    MoveEncoderPID(TestPara, -40, 14.0, 0.3, 178,true); // pick up other 3 balls
    Scrapper.set(true); // activates scraper
    TurnMaxTimePID(TurnPara, 135, 0.2, true); // turns to middle goal
    MoveTimePID(TurnPara, -45, 0.78 , 0.3, 135,false); // move into middle goal
    RunIndex(60);
    MiddleScore();
    wait(790,msec);
    NeutralScore();
    MoveEncoderPID(TurnPara, -80, 39.7, 0.3, 135, true); // moves to between matchload tube and long goal
    TurnMaxTimePID(TurnPara, 90, 0.2, true); // turn so scraper faces matchload tube
    NeutralScore();
    MoveTimePID(TurnPara, 50, 1.2 , 0.4, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -73, 0.95, 0.4, 90,false); // moves backwards into long goal
    HighScore();
    RunIndex(100);
    wait(75,msec);
    MoveTimePID(TurnPara, -10, 1.1, 0.4, 90,false); // moves further towards long goal
    Scrapper.set(false); // turns off scrapper in prep for driver
    wait(1000,msec);
}