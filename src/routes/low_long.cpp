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
    // SIXSEVEEN 77777777777777777777
    TurnMaxTimePID(TestPara, 18, 0.5, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -50, 9 , 0.3,18,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -15, 10 , 0.3,18,true); // drives and turns towards the 3 blocks near center
    Scrapper.set(true);
    //MoveEncoderPID(TestPara, -55, 13 , 0.3,45,true); // move to dispurt
    //MoveEncoderPID(TestPara, 55, 13 , 0.3,45,true); // move to dispurt
    TurnMaxTimePID(TestPara, 120, 0.3, true); // turns to matchloader
    RunRoller(0); 
    MoveEncoderPID(TestPara, -60, 20.5, 0.4, 120,true); // drives to long goal
    TurnMaxTimePID(TestPara, -180, 0.3, true); // turns to matchloader
    RunRoller(100); // activates intake to matchload
    MoveTimePID(TestPara, 45, 1 , 0.4, -180,false); // move into matchloader
    MoveTimePID(TestPara, 10, 0.4 , 0.4, -180,false); // move into matchloader
    MoveTimePID(TestPara, -50, 1.5, 0.4, -180,false); // move to long goal
    RunTopRoller(100);
    wait(3000,msec);
    MoveEncoderPID(TestPara, -60, 5, 0.4, -180,true); // back up
    MoveEncoderPID(TestPara, 70, 5.5, 0.4, -180,true); // descore
    MoveTimePID(TestPara, 60, 1.5, 0.4, -180,false); // move into matchloader again
    MoveTimePID(TestPara, 10, 2 , 0.4, -180,false); // move into matchloader
    
    
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