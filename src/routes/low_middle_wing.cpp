#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);

void low_middle_wing() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    //PIDDataSet TurnPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};
    // SIXSEVEEN 77777777777777777777
    NeutralScore();
    MoveEncoderPID(TurnPara, -70, 18.8, 0.3, 0,true); // drives to mathcloader
    Scrapper.set(true);
    RunIndex(70);
    TurnMaxTimePID(TurnPara, 90, 0.3, true); // turns to matchloader
    MoveTimePID(TurnPara, 50, 0.9 , 0.3, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -70, 1, 0.3, 90,false); // move backwards to long goal
    HighScore();
    MoveTimePID(TurnPara, -20, 1.25, 0.3, 90,false); // move into long goal
    Scrapper.set(false);
    MoveEncoderPID(TurnPara, -70, 1.5 , 0.3, 90,true); // go back up from long goal 
    TurnMaxTimePID(TurnPara, -155, 0.2, false); // turns left
    //MoveEncoderPID(TurnPara, -70, 7, 0.2, -180,true); // moves forward to get into a better position
    //TurnMaxTimePID(TurnPara, -140, 0.2, true); // turns to blocks
    NeutralScore();
    MoveEncoderPID(TurnPara, -60, 26, 0.4, -155,true); // gets 3 blocks
    RunIndex(40);
    MoveEncoderPID(TurnPara, -60, 6.9, 0.4, -137,true); // goes to low goal
    RunIndex(-40); // outakes
    wait(1000,msec);
    MoveEncoderPID(TurnPara, 60, 22.5, 0.4, -145,true); // backs up to prepare wing
    TurnMaxTimePID(TurnPara, -90, 0.2, true); // turns so wing is facing goal
    RunIndex(0);
    Wings.set(false);
    MoveEncoderPID(TurnPara, -100, 11, 0.3, -90,false); // wing
    TurnMaxTimePID(TurnPara, -155, 0.2, false); // tturn
}