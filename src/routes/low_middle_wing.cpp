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
    MoveEncoderPID(TurnPara, -80, 16, 0.3, 0,false); // drives to mathcloader
    Scrapper.set(true);
    RunIndex(70);
    TurnMaxTimePID(TurnPara, 90, 0.3, true); // turns to matchloader
    MoveTimePID(TurnPara, 45, 1.1 , 0.3, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -80, 1, 0.3, 90,false); // move backwards to long goal
    HighScore();
    MoveTimePID(TurnPara, -20, 1.25, 0.3, 90,false); // move into long goal
    Scrapper.set(false);
    MoveEncoderPID(TurnPara, -80, 1.5 , 0.3, 90,true); // back up from long goal 
    TurnMaxTimePID(TurnPara, -152, 0.2, false); // turns left
    //MoveEncoderPID(TurnPara, -70, 7, 0.2, -180,true); // moves forward to get into a better position
    //TurnMaxTimePID(TurnPara, -140, 0.2, true); // turns to blocks
    NeutralScore();
    MoveEncoderPID(TurnPara, -70, 23.8, 0.4, -152,false); // gets 3 blocks
    RunIndex(40);
    TurnMaxTimePID(TurnPara, -135, 0.2, true); // tunr to boptlmtom foal;
    MoveEncoderPID(TurnPara, -80, 6, 0.4, -135,true); // goes to low goal
    RunIndex(-40); // outakes
    wait(1000,msec);
    MoveEncoderPID(TurnPara, 80, 22.2, 0.4, -145,true); // backs up to prepare wing
    TurnMaxTimePID(TurnPara, -90, 0.2, true); // turns so wing is facing goal
    RunIndex(0);
    Wings.set(false);
    MoveEncoderPID(TurnPara, -100, 11.2, 0.3, -90,false); // wing
    TurnMaxTimePID(TurnPara, -155, 0.2, false); // tturn
}