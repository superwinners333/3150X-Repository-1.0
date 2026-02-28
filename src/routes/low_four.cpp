#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TurnPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TurnPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TurnPara, motor speed, time traveled (sec), time to full speed, heading, false);

void low_four() { // NEGATIVE TURNS TO THE LEFT
    /// declare initial conditions
    //PIDDataSet TurnPara={4,0.1,0.2};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};
    // SIXSEVEEN 77777777777777777777
    timer stopwatch;

    NeutralScore();
    MoveEncoderPID(TurnPara, -70, 25.5, 0.3, 93,true); // drives to mathcloader
    Scrapper.set(true);
    RunIndex(100);
    // TurnMaxTimePID(TurnPara, -180, 0.3, true); // turns to matchloader
    MoveTimePID(TurnPara, 50, 1.115, 0.3, 180,false); // move into matchloader
    MoveTimePID(TurnPara, -70, 0.9, 0.3, 180,false); // move backwards to long goal
    HighScore();
    MoveTimePID(TurnPara, -40, 0.9, 0.3, 180,false); // pushes into long goal

    // wing code
    MoveEncoderPID(TestPara, -100, 2, 0.2, 175, false);
    MoveEncoderPID(TestPara, -110, 3.2, 0.3, 130, false); 
    MoveEncoderPID(TestPara, -110, 5.6, 0.1, 90, false); // aggressively curves
    Wings.set(false); // lowers wings
    NeutralScore(); // stops rolling block violations

    MoveEncoderPID(TestPara, 110, 4.6, 0.4, -100, false); // straightens the bot out 

    MoveEncoderPID(TestPara, 60, 20.9, 0.5, 179, false); // backs up to wing
    wait(150,msec);
    Move(-40,0);
    wait(100,msec);
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
    wait(2000,msec);

    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
}