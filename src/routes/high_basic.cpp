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

    TurnMaxTimePID(TestPara, -20, 0.2, true); // turns to 3 balls
    RunRoller(100); // activates intake
    wait(10,msec);
    MoveEncoderPID(TestPara, -60, 8 , 0.3,-20,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -25, 9 , 0.3,-20,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -60, 5.5 , 0.3,-20,true); // drives towards line
    MoveEncoderPID(TestPara, -60, 12 , 0.3,-75,true); // grab balls
    wait(100,msec);
    Scrapper.set(true);
    RunRoller(100); 
    wait(250,msec);
    TurnMaxTimePID(TestPara, -70, 0.1, true); // turns away
    MoveEncoderPID(TestPara, 60, 20 , 0.3,-70,true); // move back
    TurnMaxTimePID(TestPara, -135, 0.3, true); // turns to mid goal
    MoveEncoderPID(TestPara, 60, 2 , 0.4, -135,false); // drives partway to under the bar
    wait(100,msec);
    Lift.set(true);
    RunTopRoller(67);
    RunRoller(100); // activates intake to score
    wait(750,msec);
    RunRoller(0);
    RunTopRoller(0);
    Lift.set(false);
    MoveEncoderPID(TestPara, -80, 34, 0.4, -135,false); // drives to long goal
    //MoveEncoderPID(TestPara, -60, 39.1, 0.4, -130,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.3, true); // turns to matchloader
    MoveTimePID(TestPara, 45, 1 , 0.4, 180,false); // move into matchloader
    MoveTimePID(TestPara, 10, 0.4, 0.4, 180,false); // mactchload
    MoveTimePID(TestPara, -50, 1.5, 0.4, 180,false); // move to long goal
    wait(100,msec);
    RunRoller(100);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.5, 0.4, 180,false); // move to long goal
    MoveTimePID(TestPara, 50, 1.5, 0.4, 180,false); // move into matchloader again
    wait(15000,msec);
}


