#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void high_long() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    //PIDDataSet TestPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    TurnMaxTimePID(TestPara, 20, 0.2, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -50, 23 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    //MoveEncoderPID(TestPara, -70, 8 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    //MoveEncoderPID(TestPara, -25, 9 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    //MoveEncoderPID(TestPara, -70, 5.9 , 0.3,20,true); // drives towards line
    MoveEncoderPID(TestPara, -70, 11.5 , 0.3,80,true); // grab balls
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -70, 1.5 , 0.3,80,true); // grab balls
    wait(100,msec);
    RunRoller(100); 
    wait(250,msec);
    TurnMaxTimePID(TestPara, 70, 0.1, true); // turns away
    MoveEncoderPID(TestPara, 70, 18.7 , 0.3,70,true); // move back
    TurnMaxTimePID(TestPara, 125, 0.2, true); // turns to mid goal
    MoveEncoderPID(TestPara, -70, 34.5, 0.4, 125,false); // drives to long goal
    //MoveEncoderPID(TestPara, -60, 39.1, 0.4, -130,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.3, true); // turns to matchloader
    RunRoller(100);
    MoveTimePID(TestPara, 45, 1.2 , 0.4, 180,false); // move into matchloader
    MoveTimePID(TestPara, 20, 0.4, 0.4, 180,false); // mactchload
    MoveTimePID(TestPara, -50, 1.5, 0.4, 180,false); // move to long goal
    wait(100,msec);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.5, 0.4, 180,false); // move to long goal
    MoveTimePID(TestPara, 45, 1.5, 0.4, 180,false); // move into matchloader again
    MoveTimePID(TestPara, 20, 100, 0.4, 180,false); // mactchload
    wait(15000,msec);
}
/*TurnMaxTimePID(TestPara, -18, 0.5, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -50, 10 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -15, 9 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    TurnMaxTimePID(TestPara, -50, 0.2, true); // turns to center
    RunRoller(0); 
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -55, 13.5 , 0.3,-50,true); // move to dispurt
    MoveEncoderPID(TestPara, 55, 13.5 , 0.3,-50,true); // move to dispurt
    TurnMaxTimePID(TestPara, -120, 0.4, true); // turns to matchloader
    MoveEncoderPID(TestPara, -60, 20, 0.4, -120,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.5, true); // turns to matchloader
    RunRoller(100); // activates intake to matchload
    MoveTimePID(TestPara, 55, 1 , 0.4, 180,false); // move into matchloader
    wait(200,msec);
    MoveTimePID(TestPara, -50, 1.5, 0.4, 180,false); // move to long goal
    RunTopRoller(100);
    wait(3000,msec);
    MoveTimePID(TestPara, 50, 1.5, 0.4, 180,false); // move into matchloader again
    Brain.Screen.setFont(monoXL);
    Brain.Screen.setPenColor("#39FF14");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("6-7 Blocks Scored");
    wait(15000,msec);*/