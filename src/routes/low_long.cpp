#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void low_long() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    //PIDDataSet TestPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    TurnMaxTimePID(TestPara, 20, 0.1, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -70, 8 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -25, 9 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -70, 6.25 , 0.3,20,true); // drives towards line
    RunRoller(100);
    MoveEncoderPID(TestPara, -60, 11.6 , 0.3,80,true); // grab balls
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -60, 2 , 0.3,80,true); // grab balls
    wait(250,msec);
    TurnMaxTimePID(TestPara, 70, 0.1, true); // turns away
    MoveEncoderPID(TestPara, 70, 15 , 0.3,70,true); // move back
    TurnMaxTimePID(TestPara, 136, 0.3, true); // turns to long goal
    MoveEncoderPID(TestPara, -70, 29.4, 0.4, 136,false); // drives to long goal
    //MoveEncoderPID(TestPara, -60, 39.1, 0.4, -130,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.2, true); // turns to long goal
    MoveTimePID(TestPara, -70, 0.64, 0.4, 180,false); // move to long goal
    RunRoller(-100);
    wait(200,msec);
    RunTopRoller(100);
    RunRoller(100);
    MoveTimePID(TestPara, -10, 1.5, 0.4, 180,false); // score
    RunTopRoller(0);
    MoveTimePID(TestPara, 50, 1.7 , 0.4, 180,false); // move into matchloader
    MoveTimePID(TestPara, 20, 0.1, 0.4, 180,false); // mactchload
    MoveTimePID(TestPara, -60, 1, 0.6, 180,false); // move to long goal
    wait(100,msec);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 0.9, 0.4, 180,false); // score
    MoveTimePID(TestPara, 100, 0.36, 0.4, 180,false); // ram
    MoveTimePID(TestPara, -100, 1, 0.4, 180,false); // ram
    wait(15000,msec);
    
    
}
/*


    TurnMaxTimePID(TestPara, 18, 0.5, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -50, 10 , 0.3,18,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -20, 9 , 0.3,18,true); // drives and turns towards the 3 blocks near center
    TurnMaxTimePID(TestPara, 45, 0.2, true); // turns to center
    RunRoller(0); 
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -55, 13 , 0.3,45,true); // move to dispurt
    MoveEncoderPID(TestPara, 55, 13 , 0.3,45,true); // move to dispurt
    TurnMaxTimePID(TestPara, 120, 0.3, true); // turns to matchloader
    MoveEncoderPID(TestPara, -60, 20.7, 0.4, 120,false); // drives to long goal
    TurnMaxTimePID(TestPara, -180, 0.3, true); // turns to matchloader
    RunRoller(100); // activates intake to matchload
    MoveTimePID(TestPara, 55, 1 , 0.4, -180,false); // move into matchloader
    MoveTimePID(TestPara, 10, 0.1 , 0.4, -180,false); // move into matchloader
    MoveTimePID(TestPara, -50, 1.5, 0.4, -180,false); // move to long goal
    RunTopRoller(100);
    wait(2200,msec);
    MoveTimePID(TestPara, 50, 1.5, 0.4, -180,false); // move into matchloader again
    MoveTimePID(TestPara, 10, 2 , 0.4, -180,false); // move into matchloader
*/