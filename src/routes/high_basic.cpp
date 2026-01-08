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
    //MoveEncoderPID(TestPara, -90, 40 , 0.3, 0,false); // drives and turns towards the 3 blocks near center
    
    
    //TurnMaxTimePID(TestPara, -20, 0.1, true); // turns to 3 balls
    RunIndex(100); // activates intake
    MoveEncoderPID(TestPara, -70, 27.9 , 0.3,-30,false); // drives and turns towards the 3 blocks near center
    //MoveEncoderPID(TestPara, -70, 5.9 , 0.3,-20,true); // drives towards line 
    MoveEncoderPID(TestPara, -80, 10.5 , 0.3,-70,true); // curves to balls ynder logkn goal
    Scrapper.set(true);
    //MoveEncoderPID(TestPara, -60, 2 , 0.3,-80,true); // moves forward a bit more
    wait(250,msec);

    //TurnMaxTimePID(TestPara, -70, 0.1, true); // turns away
    MoveEncoderPID(TestPara, 80, 17.7, 0.3,-76,true); // move back to mis goal
    TurnMaxTimePID(TestPara, -135, 0.3, true); // turns to mid goal
    MoveEncoderPID(TestPara, 50, 1 , 0.2, -135,false); // drives backwards to middle goal
    RunIndex(50);    
    MiddleScore();
    wait(550,msec);
    RunIndex(0);
    NeutralScore();
    wait(50,msec);
    MoveEncoderPID(TestPara, -80, 32.8, 0.4, -125,false); // drives to long goal
    //MoveEncoderPID(TestPara, -60, 39.1, 0.4, -130,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.3, true); // turns to matchloader
    RunIndex(100);
    MoveTimePID(TestPara, 45, 1.4 , 0.4, 180,false); // move into matchloader
    //MoveTimePID(TestPara, 20, 0.4, 0.4, 180,false); // mactchload
    MoveTimePID(TestPara, -70, 0.8, 0.4, 180,false); // move to long goal
    Scrapper.set(false);
    HighScore();
    MoveTimePID(TestPara, -25, 1.5, 0.4, 180,false); // score
    MoveEncoderPID(TestPara, -90, 9.5, 0.4, 158, false); // goes away from long goal
    Wings.set(false); // lowers wings
    wait(100,msec);
    MoveEncoderPID(TestPara, 100, 7.85, 0.4, -170, false); // goes to the side of long goal a bit

    MoveEncoderPID(TestPara, 100, 10, 0.4, 178, false); // backs up to wing
    //MoveTimePID(TestPara, -40, 0.6, 0.2, -175, false); // slows down
    wait(200,msec);
    Move(-30,0);
    wait(100,msec);
    //Move(-30,0); // turns to lock
    wait(2000,msec);
    //Move(0,0);
    //BStop();

    // MoveTimePID(TestPara, 20, 100, 0.4, 180,false); // mactchload the opposite coloured blocks out of the tube
    wait(15000,msec);
    
}   


