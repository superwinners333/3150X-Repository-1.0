#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

// MoveDistancePID(PIDDataSet KDist, PIDDataSet KTurn, double dist, double ABSHDG, bool brake)
// PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)

void high_rush() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet TurnPara={1.5,0.1,0.12};
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet PurePara={1.5,0.1,0.12};

    RunIndex(100);
    MoveEncoderPID(TurnPara, -100, 10, 0.2, -10, false); // goes forward
    TurnMaxTimePID(TurnPara, -80, 0.33, false); // turn to 3 blocks
    MoveEncoderPID(TurnPara, -100, 10.6, 0.2, -85, false); // goes into the 3 blocks
    Scrapper.set(true);
    TurnMaxTimePID(TurnPara, -130, 0.2, false); // turns to between long goal and matchload tube

    MoveEncoderPID(TurnPara, -80, 16.3, 0.4, -130, true); // goes between there

    TurnMaxTimePID(TurnPara, -170, 0.35, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.5, 0.5, -178, false); // goes into matchload
    MoveTimePID(TurnPara, 40, 0.75, 0.2, 180, false); // slows down
    
    // MoveEncoderPID(TurnPara, 50, 1, 0.1, 180, false); // moves backwards very little so we have room to adjust

    MoveTimePID(TurnPara, -80, 0.9, 1, 180, false); // goes backwards into long goal
    HighScore(); // activates long goal scoring
    wait(100,msec);
    MoveTimePID(TurnPara, -50, 1.7, 0.2, 180, false); // pushes into long goal

    // wing code
    MoveEncoderPID(TestPara, -90, 9.5, 0.4, 158, false); // goes away from long goal
    Wings.set(false); // lowers wings
    wait(100,msec);
    MoveEncoderPID(TestPara, 100, 7.85, 0.4, -170, false); // goes to the side of long goal a bit

    MoveEncoderPID(TestPara, 100, 12.3, 0.6, 179, false); // backs up to wing
    //MoveTimePID(TestPara, -40, 0.6, 0.2, -175, false); // slows down
    wait(200,msec);
    Move(-30,0);
    wait(100,msec);
    //Move(-30,0); // turns to lock
    wait(2000,msec);
    //Move(0,0);
    //BStop();

    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}