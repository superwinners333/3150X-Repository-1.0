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
    MoveEncoderPID(TurnPara, -70, 19.7 , 0.3, 0,true); // drives to mathcloader
    Scrapper.set(true);
    RunRoller(100);
    TurnMaxTimePID(TurnPara, 90, 0.2, true); // turns to matchloader
    MoveTimePID(TurnPara, 50, 1.0 , 0.3, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -70, 1.1, 0.3, 90,false); // move backwards to long goal
    RunTopRoller(100);
    MoveTimePID(TurnPara, -10, 1, 0.3, 90,false); // move to long goal
    RunRoller(0);
    RunTopRoller(0);
    Scrapper.set(false);
    MoveEncoderPID(TurnPara, -70, 1.5 , 0.3, 90,true); // go back up from long goal 
    TurnMaxTimePID(TurnPara, -155, 0.2, false); // turns left
    RunRoller(100);
    //MoveEncoderPID(TurnPara, -70, 7, 0.2, -180,true); // moves forward to get into a better position
    //TurnMaxTimePID(TurnPara, -140, 0.2, true); // turns to blocks
    MoveEncoderPID(TurnPara, -40, 20.5, 0.4, -155,true); // gets 3 blocks
    TurnMaxTimePID(TurnPara, -183, 0.2, false); // turns to other 3 blocks
    MoveEncoderPID(TestPara, -100, 24, 0.3, -183,false); // move to other side
    MoveEncoderPID(TestPara, -40, 16.7, 0.3, -183,true); // pick up other 3 balls
    Scrapper.set(true); // activates scraper
    TurnMaxTimePID(TurnPara, 135, 0.2, true); // turns to middle goal
    RunRoller(0); // stops intake
    MoveEncoderPID(TurnPara, 60, 11.2, 0.3, 135,true); // move to middle goal
    RunRoller(-100); // stop jam
    wait(100,msec);
    Lift.set(true); // lets us score on middle goal
    RunRoller(100); // activates intake
    RunTopRoller(67);
    wait(750,msec);
    Lift.set(false); // lets us score on long goal
    RunTopRoller(-15);
    MoveEncoderPID(TurnPara, -80, 38.7, 0.3, 135, true); // moves to between matchload tube and long goal
    TurnMaxTimePID(TurnPara, 90, 0.1, true); // turn so scraper faces matchload tube
    RunTopRoller(0);
    MoveTimePID(TurnPara, 50, 1.3 , 0.4, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -70, 1, 0.4, 90,false); // moves backwards into long goal
    RunRoller(-100);
    wait(75,msec);
    RunRoller(100);
    RunTopRoller(100);
    MoveTimePID(TurnPara, -10, 1.1, 0.4, 90,false); // moves further towards long goal
    Scrapper.set(false); // turns off scrapper in prep for driver
    wait(1000,msec);
}