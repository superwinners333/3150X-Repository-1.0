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
    MoveEncoderPID(TurnPara, -70, 19.5 , 0.3, 0,true);
    Scrapper.set(true);
    RunIndex(100);
    TurnMaxTimePID(TurnPara, 90, 0.3, true); 
}