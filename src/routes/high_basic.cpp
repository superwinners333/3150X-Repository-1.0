#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void high_basic() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    //PIDDataSet TestPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};

    TurnMaxTimePID(TestPara, -18, 0.5, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -50, 7 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -20, 14.5 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    //MoveEncoderPID(TestPara, 30, 1 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    wait(400,msec);
    RunRoller(0); // stop intake
    TurnMaxTimePID(TestPara, -135, 0.5, true); // turns to under the bar
    wait(100,msec);
    MoveEncoderPID(TestPara, 50, 8 , 0.4, -135,false); // drives partway to under the bar
    RunRoller(-100); //stop jam
    wait(100,msec);
    Lift.set(true);
    RunRoller(100); // activates intake to score
    wait(1700,msec);
    RunRoller(0);
    RunTopRoller(0);
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -60, 38.5, 0.4, -130,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.5, true); // turns to matchloader
    Lift.set(false);
    RunRoller(100); // activates intake to matchload
    MoveTimePID(TestPara, 55, 0.9 , 0.4, 180,false); // move into matchloader
    //MoveTimePID(TestPara, 10, 0.15, 0.4, 180,false); // mactchload
    MoveTimePID(TestPara, -50, 1.5, 0.4, 180,false); // move to long goal
    RunTopRoller(100);
    wait(1700,msec);
    MoveTimePID(TestPara, 50, 1.5, 0.4, 180,false); // move into matchloader again
    wait(15000,msec);
}


