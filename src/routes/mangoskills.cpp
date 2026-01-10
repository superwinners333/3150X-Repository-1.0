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
    PIDDataSet TurnPara={1.5,0.1,0.12};
    PIDDataSet DrivePara = {1.2, 0.0, 4.0};

    MoveEncoderPID(TurnPara, -40, 1.5, 0.3, 0, true);
    // MoveDistancePID(DrivePara, TurnPara, 30, 1, 100, 0.5, 0, true);

    // PIDDataSet PurePara={0.05,0.0,0.3};
    // std::vector<Point> path = {
    //     {0, 0},
    //     {24, 0}   // 24 inches forward
    // };

    // // Positive speed = forward
    // PurePursuitDrive(path, PurePara, 20, 60, false, true);




    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}