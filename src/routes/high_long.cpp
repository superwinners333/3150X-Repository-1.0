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
    
    // Scrapper.set(true);
    Lift.set(true); // activates middle thing
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -100, 1 , 0.4,0,true); // drives away from parking zone
    wait(10,msec);
    MoveEncoderPID(TestPara, -100, 15 , 0.3,-33,false); // drives and turns towards the 3 blocks near center
    // Scrapper.set(true); // activates scrapper to..?
    TurnMaxTimePID(TestPara, -45, 0.5, true); // turns to under the bar
    wait(100,msec);
    MoveEncoderPID(TestPara, -100, 7 , 0.4,-45,false); // drives partway to under the bar
    // Scrapper.set(false); // deactivates scrapper to not push blocks under bar away
    MoveEncoderPID(TestPara, -100, 7 , 0.4,-45,false); // finishes driving to under the bar
    Scrapper.set(true); // activates scrapper to trap blocks?
    MoveEncoderPID(TestPara, 40, 4 , 0.4,-45,false); // moves back 
    wait(100,msec);
    MoveEncoderPID(TestPara, -100, 2 , 0.4,-45,true); // moves forward again
    wait(300,msec);
    Scrapper.set(false);
    RunRoller(0);

    // scoring
    MoveEncoderPID(TestPara, 100, 10 , 0.4,-45,true); // drives backwards quickly
    wait(50,msec);
    TurnMaxTimePID(TestPara, -125, 0.5, true); // around to face to the right of match load zone
    wait(50,msec);
    MoveEncoderPID(TestPara, -100, 15 , 0.4,-125,true); // drives forward a bit
    wait(50,msec);
    TurnMaxTimePID(TestPara, -180, 0.5, true); // rotates so that the back faces the long goal
    wait(50,msec);
    MoveTimePID(TestPara, 100, 1.5 , 0.2,-180, false);
    RunIndex(100);
    wait(400,msec);
    RunIndex(0);
    RunRoller(100);
    Scrapper.set(true);
    MoveTimePID(TestPara, -100, 1.9 , 0.2,-180, false);
    wait(400,msec);
    MoveTimePID(TestPara, 100, 1.9 , 0.2,-180, false);
    RunIndex(100);

}