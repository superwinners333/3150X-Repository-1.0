#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void test() {
    // declare initial conditions
    //PIDDataSet TestPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    MoveEncoderPID(TestPara, 100, 36 , 0.4,0,true);
    Scrapper.set(false);
    wait(200,msec);
    //Lift.set(false);
    TurnMaxTimePID(TestPara, -100, 0.9, true);
    RunRoller(-100);

}
void test2(){  
    
    PIDDataSet TestPara={1.5,0.1,0.15};
    MoveEncoderPID(TestPara, 100, 12, 0.4, 0, true);
    TurnMaxTimePID(TestPara, 30, 0.5,true);
}

// NEGATIVE TURNS TO THE LEFT