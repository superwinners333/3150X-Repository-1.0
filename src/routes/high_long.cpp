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
    /*
    TurnMaxTimePID(TestPara, -18, 0.5, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -50, 10 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -20, 9 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    TurnMaxTimePID(TestPara, -38, 0.2, true); // turns to center
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -50, 15.5 , 0.3,-38,true); // move to dispurt
    MoveEncoderPID(TestPara, 50, 15.5 , 0.3,-38,true); // move to dispurt
    TurnMaxTimePID(TestPara, -120, 0.6, true); // turns to matchloader
    MoveEncoderPID(TestPara, -60, 18, 0.4, -120,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.5, true); // turns to matchloader
    RunRoller(100); // activates intake to matchload
    MoveTimePID(TestPara, 55, 0.9 , 0.4, 180,false); // move into matchloader
    MoveTimePID(TestPara, -50, 1.5, 0.4, 180,false); // move to long goal
    RunTopRoller(100);
    wait(1700,msec);
    MoveTimePID(TestPara, 50, 1.5, 0.4, 180,false); // move into matchloader again
    wait(15000,msec);
    */
    TurnMaxTimePID(TestPara, -18, 0.5, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -50, 10 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -20, 9 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    TurnMaxTimePID(TestPara, -50, 0.2, true); // turns to center
    RunRoller(0); 
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -55, 14.5 , 0.3,-50,true); // move to dispurt
    MoveEncoderPID(TestPara, 55, 14.5 , 0.3,-50,true); // move to dispurt
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
    wait(15000,msec);
}