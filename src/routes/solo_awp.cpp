#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void solo_awp() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    //PIDDataSet TestPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    // SIXSEVEEN 77777777777777777777
    MoveEncoderPID(TestPara, -70, 22.2 , 0.3, 0,true); // drives to mathcloader
    Scrapper.set(true);
    RunRoller(100);
    TurnMaxTimePID(TestPara, 90, 0.3, true); // turns to matchloader
    MoveTimePID(TestPara, 55, 0.95 , 0.4, 90,false); // move into matchloader
    //MoveTimePID(TestPara, 10, 0.05 , 0.4, 90,false); // move into matchloader
    MoveTimePID(TestPara, -70, 1, 0.4, 90,false); // move to long goal
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.5, 0.4, 90,false); // move to long goal
    RunRoller(0);
    RunTopRoller(0);
    Scrapper.set(false);
    MoveEncoderPID(TestPara, -70, 2 , 0.3, 90,true); // backup
    TurnMaxTimePID(TestPara, -180, 0.2, true); // turns to 3
    RunRoller(100);
    MoveEncoderPID(TestPara, -70, 6.2, 0.3, -180,true); // move into middle
    TurnMaxTimePID(TestPara, -135, 0.1, true); // turns to middle
    MoveEncoderPID(TestPara, -30, 25, 0.3, -135,true); // get 3 blocks
    RunRoller(-100);
    wait(1100,msec);
    RunRoller(100);
    MoveEncoderPID(TestPara, 70, 6, 0.3, -135,true); // back up
    TurnMaxTimePID(TestPara, -180, 0.1, true); // turns to middle
    MoveEncoderPID(TestPara, -80, 25, 0.3, -180,true); // move to other side
    MoveEncoderPID(TestPara, -30, 13.8, 0.3, -180,true); // pick up 3 balls
    RunRoller(0);
    TurnMaxTimePID(TestPara, 135, 0.2, true); // turns to high bar
    RunTopRoller(-25);
    MoveEncoderPID(TestPara, 70, 15.5, 0.3, 135,true); // move to high bar
    Lift.set(true);
    RunRoller(100);
    wait(1000,msec);
}