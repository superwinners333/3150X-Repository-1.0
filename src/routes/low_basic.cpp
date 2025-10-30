#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void low_basic() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    //PIDDataSet TestPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};

    Lift.set(false);
    RunRoller(100); // activates intake
    TurnMaxTimePID(TestPara, 18, 0.3, true); // turns to 3 balls
    MoveEncoderPID(TestPara, -50, 7 , 0.3,18,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -15, 13.7, 0.3,18,true); // drives and turns towards the 3 blocks near center
    //MoveEncoderPID(TestPara, 20, 1 , 0.3,18,true); // drives and turns towards the 3 blocks near center
    TurnMaxTimePID(TestPara, -47, 0.3, true); // turns to under the bar
    RunRoller(0); //stop
    MoveEncoderPID(TestPara, -50, 6.7, 0.4, -47,true); // drives partway to under the bar
    RunRoller(-100); // putak
    wait(1000,msec);
    RunRoller(0);
    MoveEncoderPID(TestPara, 60, 37.8, 0.4, -47,true); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.3, true); // turns to matchloader
    Scrapper.set(true);
    RunRoller(100); // activates intake to matchload
    MoveTimePID(TestPara, 40, 0.9 , 0.4, 180,false); // move into matchloader
    MoveTimePID(TestPara, 10, 0.15, 0.4, 180,false); // mactchload
    MoveTimePID(TestPara, -50, 1.5, 0.4, -180,false); // move to  long goal
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 2, 0.4, -180,false); // move to  long goal
    MoveTimePID(TestPara, 50, 1.5, 0.4, -180,false); // move into matchloader again
    wait(15000,msec);

    /*
        TurnMaxTimePID(TestPara, -20, 0.3, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -60, 8 , 0.3,-20,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -25, 9 , 0.3,-20,true); // drives and turns towards the 3 blocks near center
    //MoveEncoderPID(TestPara, 30, 1 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    wait(400,msec);
    TurnMaxTimePID(TestPara, -135, 0.3, true); // turns to under the bar
    // RunTopRoller(-75);
    MoveEncoderPID(TestPara, 60, 7.5 , 0.4, -135,false); // drives partway to under the bar
    wait(100,msec);
    Lift.set(true);
    RunTopRoller(67);
    RunRoller(100); // activates intake to score
    wait(750,msec);
    RunRoller(0);
    RunTopRoller(0);
    Lift.set(false);
    MoveEncoderPID(TestPara, -60, 0.5 , 0.3,-135,true); // back up a bit
    TurnMaxTimePID(TestPara, -65, 0.3, true); // turns to 2 balls
    RunRoller(0); 
    MoveEncoderPID(TestPara, -60, 13.2 , 0.3,-65,true); // grab balls
    //TurnMaxTimePID(TestPara, -80, 0.1, true); // turns to 2 balls
    MoveEncoderPID(TestPara, -60, 5.5 , 0.3,-80,true); // grab balls
    wait(100,msec);
    Scrapper.set(true);
    RunRoller(100); 
    wait(250,msec);
    TurnMaxTimePID(TestPara, -65, 0.3, true); // turns away
    MoveEncoderPID(TestPara, 60, 12 , 0.3,-65,true); // move back
    TurnMaxTimePID(TestPara, -140, 0.4, true); // turns to matchloader
    MoveEncoderPID(TestPara, -60, 24, 0.4, -140,false); // drives to long goal
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
    */
}


