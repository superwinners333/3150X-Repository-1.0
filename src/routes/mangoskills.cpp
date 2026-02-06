#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

// MoveDistancePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, int dir, int MaxSpd, double AccT, double ABSHDG, bool brake)
// PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)

void mangoskills() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet TurnPara={1.9,0.0,0.053}; // movement pid
    PIDDataSet MovePara={4.0,0.0,0.17}; // movement pid 

    PIDDataSet TestPara={1.5,0.1,0.12};
    PIDDataSet DrivePara = {1.2, 0.0, 4.0}; 

    PIDDataSet APara = {1.75,0.0,0.26};
    PIDDataSet BPara = {3.5,1.0,0.28};

    
    // RunIndex(100);
    // wait(1000,msec);
    // RunIndex(0);
    // MiddleScore();
    // wait(100,msec);
    // RunIndex(50);
    // MoveTimePID(TestPara, -10, 0.5, 0.2, 0, false); // score
    // RunIndex(40);
    // MoveTimePID(TestPara, -10, 0.8, 0.2, 0, false); // score
    // RunIndex(28);
    // MoveTimePID(TestPara, -10, 2, 0.2, 0, false); // score
    // NeutralScore();

    // WallBackPID(APara, TurnPara, 15.0, 10.0, 100.0, 5.0, 0, true);
    AccuratePID(APara, TurnPara, 1.0, 5.0, 100, 5.0, 0, true);

    // wait(50,msec);
    // TurnMaxTimePID(TurnPara,90,0.5,true);
    // MovePID(APara, TurnPara, 24, -100, 10.0, 90, true);

    // MoveEncoderPID(TurnPara, -40, 2.5, 0.3, 0, true);





    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}