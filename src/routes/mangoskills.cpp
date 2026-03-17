#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "../odom.hpp"
#include "../Odometry.hpp"
#include "../VirtualTargetPursuit.hpp"
#include "vex.h"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

// MoveDistancePID(PIDDataSet DistK, PIDDataSet HeadK, double dist, int dir, int MaxSpd, double AccT, double ABSHDG, bool brake)
// PurePursuitDrive(std::vector<Point> path, PIDDataSet KTurn, double lookahead, double maxSpeed, bool reverse, bool brake)

void mangoskills() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet MovePara={1.75,0.0,0.26}; // accurate pid distK values
    PIDDataSet TurnPara={1.9,0.0,0.053}; // accurate pid headK values

    PIDDataSet TestPara={1.5,0.1,0.15}; // for basic turning
    PIDDataSet CorrectionPara = {1.0,0.0,0.0}; // for basic correction
    PIDDataSet DrivsePara = {1.2, 0.0, 4.0}; 

    PIDDataSet APara = {1.2,0.5,0.26};
    PIDDataSet BPara = {2.8,0.0,0.2}; // for 
    PIDDataSet CPara = {2.0,0.1,0.0};


    // AccuratePID(MovePara, TurnPara, 1.0, 5.0, 100, 5.0, 0, true);
    globalHeading = 0;
    Point target = {-8.0,70.0};
    Point target2 = {-16.0,80.0};
    Point target3 = {-70.0,85.0};

    Point a1 = {-8,23.0};
    Point b1 = {-74.0,30.0};
    Point c1 = {-70.0,85.0};
    Point d1 = {-16.0,80.0};


    // startTracking({0.0,0.0});


    // curveToPoint(target, 90.0, 2.0, true);

    // straightToPoint(TestPara, CorrectionPara, MovePara, target, 100.0, 5.0, true);
    // straightToPoint(TestPara, CorrectionPara, MovePara, target2, -100.0, 5.0, true);

    // boohoo(BPara, MovePara, target, 100.0,60.0,5.0,true);

    // driveToPointVTP(CPara,-9.8,21.6,100.0,90.0,10000,true);
    // driveToPointVTP(CPara,-32.0,0.0,100.0,90.0,10000,true);
    std::cout<<"hi"<<std::endl;
    std::cout<<"hi"<<std::endl;
    std::cout<<"hi"<<std::endl;
    std::cout<<"hi"<<std::endl;
    // driveToPointVTP(CPara,8.0,70.0,100.0,100.0,3,false);

    // // std::cout<<"hi"<<std::endl;
    // driveToPointVTP(CPara,16.0,80.0,100.0,100.0,3,false);
    // wait(200,msec);
    // driveToPointVTP(CPara,70.0,85.0,100.0,90.0,3,false);
    // driveToPointVTP(CPara,70.0,95.0,100.0,40.0,3,true);

    // globalHeading = 0;
    // boohoo(BPara, MovePara, a1, 100.0,90.0,5.0,false);
    // boohoo(BPara, MovePara, b1, 100.0,90.0,5.0,true);
    // TurnMaxTimePID(TestPara, 180.0, 0.3, false);
    // MoveTimePID(TestPara, 100.0, 0.7, 0.5, 180.0, false);
    // MoveTimePID(TestPara, -100.0, 1.2, 0.5, 180.0, false);
    // CPos = longGoalReset(CPos);
    // wait(50,msec);
    // boohoo(BPara, MovePara, c1, -100.0,-90.0,5.0,false);

    boohoo2(BPara, MovePara, a1, 100.0,90.0,5.0,false);
    boohoo2(BPara, MovePara, b1, 60.0,40.0,5.0,false);
    boohoo2(BPara, MovePara, c1, 100.0,90.0,5.0,false);
    boohoo2(BPara, MovePara, d1, 100.0,90.0,5.0,false);
    
    wait(10,sec);





    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}