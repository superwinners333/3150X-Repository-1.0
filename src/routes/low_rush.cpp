#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

// MoveDistancePID(PIDDataSet KDist, PIDDataSet KTurn, double dist, double ABSHDG, bool brake)
// PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)

void low_rush() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet TurnPara={1.5,0.1,0.12};
    PIDDataSet DrivePara={1.5,0.1,0.12};
    PIDDataSet PurePara={1.5,0.1,0.12};

    RunIndex(100);
    MoveEncoderPID(TurnPara, -100, 10, 0.2, 10, false); // goes forward
    TurnMaxTimePID(TurnPara, 80, 0.33, false); // turn to 3 blocks
    MoveEncoderPID(TurnPara, -100, 10.6, 0.2, 85, false); // goes into the 3 blocks
    Scrapper.set(true);
    TurnMaxTimePID(TurnPara, 130, 0.2, false); // turns to between long goal and matchload tube

    MoveEncoderPID(TurnPara, -80, 16.3, 0.4, 130, true); // goes between there

    TurnMaxTimePID(TurnPara, 170, 0.35, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.5, 0.5, 178, false); // goes into matchload
    MoveTimePID(TurnPara, 40, 0.75, 0.2, 180, false); // slows down
    
    // MoveEncoderPID(TurnPara, 50, 1, 0.1, 180, false); // moves backwards very little so we have room to adjust

    MoveTimePID(TurnPara, -80, 0.9, 1, 180, false); // goes backwards into long goal
    HighScore(); // activates long goal scoring
    wait(100,msec);
    MoveTimePID(TurnPara, -50, 1.7, 0.2, 180, false); // pushes into long goal

    MoveEncoderPID(TurnPara, -100, 9.5, 0.4, 165, false); // goes away from long goal
    Wings.set(false); // lowers wings
    wait(100,msec);
    MoveEncoderPID(TurnPara, 100, 7.85, 0.4, -164.5, false); // goes to the side of long goal a bit

    MoveEncoderPID(TurnPara, 100, 9.4, 0.4, 179, false); // backs up to wing
    MoveTimePID(TurnPara, -40, 0.6, 0.2, 177, false); // slows down
    wait(200,msec);
    MoveTimePID(TurnPara, -60, 1, 0.2, 170, true); // turns a bit while backing up
    wait(100,msec);
    Move(-30,0); // turns to lock
    wait(2000,msec);
    Move(0,0);
    BStop();
    // TurnMaxTimePID(TurnPara, -150, 0.35, true); // turns to lock
    //TurnMaxTimePID(TurnPara, -150, 0.35, true);



    // PIDDataSet TestPara={1.5,0.1,0.15};
    // TurnMaxTimePID(TestPara, 20, 0.1, true); // turns to 3 balls
    // RunIndex(100); // activates intake
    // MoveEncoderPID(TestPara, -100, 8 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    // MoveEncoderPID(TestPara, -25, 9 , 0.3,20,true); // drives and turns towards the 3 blocks near center
    // MoveEncoderPID(TestPara, -70, 6.25 , 0.3,20,true); // drives towards line
    // RunIndex(100);
    // MoveEncoderPID(TestPara, -60, 11.6 , 0.3,80,true); // grab balls
    // Scrapper.set(true);
    // MoveEncoderPID(TestPara, -60, 2 , 0.3,80,true); // grab balls
    // wait(250,msec);
    // TurnMaxTimePID(TestPara, 70, 0.1, true); // turns away
    // MoveEncoderPID(TestPara, 70, 15 , 0.3,70,true); // move back
    // TurnMaxTimePID(TestPara, 136, 0.3, true); // turns to long goal
    // MoveEncoderPID(TestPara, -70, 29.4, 0.4, 136,false); // drives to long goal
    // //MoveEncoderPID(TestPara, -60, 39.1, 0.4, -130,false); // drives to long goal
    // TurnMaxTimePID(TestPara, 180, 0.2, true); // turns to long goal
    // MoveTimePID(TestPara, -70, 0.64, 0.4, 180,false); // move to long goal
    // HighScore();
    // RunIndex(100);
    // MoveTimePID(TestPara, -10, 1.5, 0.4, 180,false); // score
    // NeutralScore();
    // MoveTimePID(TestPara, 50, 1.7 , 0.4, 180,false); // move into matchloader
    // MoveTimePID(TestPara, 20, 0.1, 0.4, 180,false); // mactchload
    // MoveTimePID(TestPara, -60, 1, 0.6, 180,false); // move to long goal
    // wait(100,msec);
    // HighScore();
    // MoveTimePID(TestPara, -10, 0.9, 0.4, 180,false); // score
    // RunIndex(0);
    // MoveTimePID(TestPara, 100, 0.36, 0.4, 180,false); // ram
    // MoveTimePID(TestPara, -100, 1, 0.4, 180,false); // ram
    // wait(15000,msec);
    // std::vector<Point> backCurve = {
    // {0, 0},
    // {-12, -6},
    // {-24, -6},
    // {-36, 0}
    // };

    // PurePursuitDrive(backCurve, PurePara, 10, 80, true, false);


    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}