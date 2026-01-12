#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);

void low_four() { // NEGATIVE TURNS TO THE LEFT
    /// declare initial conditions
    //PIDDataSet TurnPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};
    // SIXSEVEEN 77777777777777777777
    NeutralScore();
    MoveEncoderPID(TurnPara, -70, 19.3 , 0.3, 0,true); // drives to mathcloader
    Scrapper.set(true);
    RunIndex(100);
    TurnMaxTimePID(TurnPara, 90, 0.3, true); // turns to matchloader
    MoveTimePID(TurnPara, 50, 0.9 , 0.3, 90,false); // move into matchloader
    MoveTimePID(TurnPara, -70, 0.9, 0.3, 90,false); // move backwards to long goal
    HighScore();
    MoveTimePID(TurnPara, -10, 1, 0.3, 90,false); // move to long goal
    Scrapper.set(false);


    MoveEncoderPID(TestPara, -90, 9.5, 0.4, 69, false); // goes away from long goal
    Wings.set(false); // lowers wings
    wait(100,msec);
    MoveEncoderPID(TestPara, 100, 7.85, 0.4, 100, false); // goes to the side of long goal a bit

    MoveEncoderPID(TestPara, 100, 12.3, 0.6, 89, false); // backs up to wing
    //MoveTimePID(TestPara, -40, 0.6, 0.2, -175, false); // slows down
    wait(200,msec);
    Move(-30,0);
    wait(100,msec);
    //Move(-30,0); // turns to lock
    wait(2000,msec);
    //Move(0,0);
    //BStop();
}