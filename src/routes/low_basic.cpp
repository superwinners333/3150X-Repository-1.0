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

    TurnMaxTimePID(TestPara, 20, 0.1, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -70, 8 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -25, 9 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -70, 6.33 , 0.3,20,true); // drives towards line
    RunRoller(100);
    MoveEncoderPID(TestPara, -60, 14.2 , 0.3,80,true); // grab balls
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -60, 2 , 0.3,80,true); // grab balls
    wait(250,msec);
    TurnMaxTimePID(TestPara, 70, 0.1, true); // turns away
    MoveEncoderPID(TestPara, 70, 21 , 0.3,70,true); // move back
    Scrapper.set(false);
    TurnMaxTimePID(TestPara, -43, 0.2, true); // turns to low goal
    MoveEncoderPID(TestPara, -70, 4.5, 0.4, -43,false); // drives to low goal
    RunRoller(-100);
    wait(900,msec);
    RunRoller(100);
    MoveEncoderPID(TestPara, 70, 2, 0.4, -45,false); // backs away from low goal
    RunRoller(0);
    TurnMaxTimePID(TestPara, 135, 0.2, true);
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -70, 29.3, 0.4, 135,false); // drives to long goal

    TurnMaxTimePID(TestPara, 180, 0.2, true); // turns to matchloader
    RunRoller(100);
    MoveTimePID(TestPara, 45, 1.2 , 0.4, 180,false); // move into matchloader
    MoveTimePID(TestPara, 20, 0.35, 0.4, 180,false); // mactchload
    MoveTimePID(TestPara, -50, 1.5, 0.4, 180,false); // move to long goal
    wait(100,msec);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.5, 0.4, 180,false); // move to long goal
    MoveTimePID(TestPara, 45, 1.5, 0.4, 180,false); // move into matchloader again
    MoveTimePID(TestPara, 20, 100, 0.4, 180,false); // mactchload the opposite coloured blocks out of the tube
    wait(15000,msec);
    
    



    



    /*
    TurnMaxTimePID(TestPara, 20, 0.1, true); // turns to 3 balls
    RunRoller(100); // activates intake
    wait(10,msec);
    MoveEncoderPID(TestPara, -90, 8 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -25, 9 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    TurnMaxTimePID(TestPara, -45, 0.2, true); // turns to mid goal
    MoveEncoderPID(TestPara, -50, 2.9, 0.4, -45,false); // drives partway to under the bar
    wait(100,msec);
    RunRoller(-80); // activates intake to score
    wait(1500,msec);
    RunRoller(0); // activates intake to score
    MoveEncoderPID(TestPara, 90, 4 , 0.4, -45,false); // back up
    TurnMaxTimePID(TestPara, 20, 0.3, true); // turns to mid goal
    MoveEncoderPID(TestPara, -90, 6 , 0.3,20,true); // drives towards line
    RunRoller(100); 
    MoveEncoderPID(TestPara, -90, 10.5 , 0.3,80,true); // grab balls
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -90, 2 , 0.3,80,true); // grab balls
    wait(250,msec);
    TurnMaxTimePID(TestPara, 70, 0.1, true); // turns away
    MoveEncoderPID(TestPara, 90, 17.8 , 0.3,60,true); // move back
    TurnMaxTimePID(TestPara, 125, 0.2, true); // turns to long goal
    MoveEncoderPID(TestPara, -90, 24, 0.4, 125,false); // drives to long goal
    //MoveEncoderPID(TestPara, -60, 39.1, 0.4, -130,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.2, true); // turns to matchloader
    RunRoller(100);
    MoveTimePID(TestPara, 45, 1.2 , 0.4, 180,false); // move into matchloader
    MoveTimePID(TestPara, 20, 0.3, 0.4, 180,false); // mactchload
    MoveTimePID(TestPara, -50, 1.5, 0.4, 180,false); // move to long goal
    wait(100,msec);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.5, 0.4, 180,false); // move to long goal
    MoveTimePID(TestPara, 45, 1.5, 0.4, 180,false); // move into matchloader again
    MoveTimePID(TestPara, 20, 100, 0.4, 180,false); // mactchload
    wait(15000,msec);
    */


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
    LiftUp.set(true);
    RunTopRoller(67);
    RunRoller(100); // activates intake to score
    wait(750,msec);
    RunRoller(0);
    RunTopRoller(0);
    LiftUp.set(false);
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


