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
    MoveEncoderPID(TestPara, -60, 22.7 , 0.3, 0,true); // drives to mathcloader
    Scrapper.set(true);
    TurnMaxTimePID(TestPara, 90, 0.3, true); // turns to matchloader
    RunRoller(100);
    MoveTimePID(TestPara, 55, 0.9 , 0.4, 90,false); // move into matchloader
    //wait(200,msec);
    MoveTimePID(TestPara, -50, 1.5, 0.4, 90,false); // move to long goal
    RunTopRoller(100);
    wait(1000,msec);
    RunRoller(0);
    RunTopRoller(0);
    Scrapper.set(false);
    MoveEncoderPID(TestPara, -70, 2 , 0.3, 90,true); // backup
    TurnMaxTimePID(TestPara, -180, 0.2, true); // turns to 3
    RunRoller(100);
    MoveEncoderPID(TestPara, -60, 9, 0.3, -180,true); // move into middle
    TurnMaxTimePID(TestPara, -135, 0.1, true); // turns to middle
    MoveEncoderPID(TestPara, -40, 24, 0.3, -135,true); // get 3 blocks
    RunRoller(-100);
    wait(1000,msec);
    MoveEncoderPID(TestPara, 60, 10, 0.3, -135,true); // back up
    TurnMaxTimePID(TestPara, -180, 0.2, true); // turns to middle
    RunRoller(100);
    MoveEncoderPID(TestPara, -60, 25, 0.3, -180,true); // move to other side
    MoveEncoderPID(TestPara, -20, 13.5, 0.3, -180,true); // pick up 3 balls
    RunRoller(0);
    TurnMaxTimePID(TestPara, 135, 0.2, true); // turns to high bar
    MoveEncoderPID(TestPara, 60, 12, 0.3, 135,true); // move to high bar
    Lift.set(true);
    RunRoller(100);
}