#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "../odom.hpp"
#include "vex.h"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);



// 102 ROUTE


void yahuskills() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet MovePara={1.75,0.0,0.26}; // accurate pid distK values
    PIDDataSet TurnPara={1.9,0.0,0.053}; // accurate pid headK values

    PIDDataSet UhhhPara={1.5,0.1,0.15}; // for basic turning
    PIDDataSet CorrectionPara = {1.0,0.0,0.0}; // for basic correction
    PIDDataSet DrivePara = {1.2, 0.0, 4.0}; 

    PIDDataSet TestPara={1.5,0.1,0.12};
    PIDDataSet pp={1.5,0.1,0.15};
    PIDDataSet pp2={1.2,0.1,0.15};

    // AccuratePID(MovePara, TurnPara, 1.0, 5.0, 100, 5.0, 0, true);
    globalHeading = 180;
    Point target = {-12.0,24.0};
    Point target2 = {0.0,0.0};

    globalHeading = 180;
    startTracking({0.0,0.0});

    // straightToPoint(UhhhPara, CorrectionPara, MovePara, target, 100.0, 5.0, true);
    
    timer stopwatch;
    
    RunIndex(80);
    MoveTimePID(TestPara, 55, 0.5, 0.2, 0, false); // grab blocks in park
    MoveTimePID(TestPara, -20, 0.3, 0.02, 0, false); // grab blocks in park
    TurnMaxTimePID(TestPara, 10, 0.1, true); // turn
    TurnMaxTimePID(TestPara, -10, 0.1, true); // turn
    MoveTimePID(TestPara, 30, 0.2, 0.02, 0, false); // grab blocks in park
    MoveTimePID(TestPara, -30, 0.2, 0.02, 0, false); // grab blocks in park
    wait(250,msec);
    MoveTimePID(TestPara, 30, 0.2, 0.02, 0, false); // grab blocks in park
    MoveTimePID(TestPara, 90, 1, 0.02, 0, false); // grab blocks in park
    RunIndex(100);
    MoveTimePID(TestPara, -30, 0.1, 0.02, 0, false); // grab blocks in park
    TurnMaxTimePID(TestPara, 10, 0.1, true); // turn
    TurnMaxTimePID(TestPara, -10, 0.1, true); // turn
    MoveTimePID(TestPara, 40, 0.1, 0.02, 0, false); // grab blocks in park
    MoveTimePID(TestPara, -30, 0.1, 0.02, 0, false); // grab blocks in park
    wait(250,msec);
    MoveTimePID(TestPara, 40, 0.1, 0.02, 0, false); // grab blocks in park
    
    wait(250,msec);
    MoveTimePID(TestPara, -80, 0.6, 0.2, 0, true); // back out of park
    MoveTimePID(TestPara, 25, 0.8, 0.2, 0, true); // align with park
    Gyro.setHeading(0,degrees);
    MoveEncoderPID(TestPara, 80, 12 , 0.3, 0, true); // back up 
    TurnMaxTimePID(TestPara, -45, 0.4, true); // turns to middle goal
    RunIndex(100);
    MoveEncoderPID(TestPara, 80, 5.4 , 0.3, -45, false); // move to inbetween middle goal and 4 blocks
    MoveEncoderPID(TestPara, 80, 13 , 0.3, 45, true); // turn into middle goal
    MoveTimePID(TestPara, -50, 0.15, 0.2, 45, false); // aglin
    MoveTimePID(TestPara, -30, 0.1, 0.1, 20, false); // wiggles
    MoveTimePID(TestPara, -30, 0.1, 0.1, 70, false); // wiggles
    MoveTimePID(TestPara, -30, 0.1, 0.1, 45, false); // aglin
    //TurnMaxTimePID(TestPara, 45, 0.3, true); // turns to face middle goal
    //MoveTimePID(TestPara, -40, 0.5, 0.2, 45, false); // backs up into middle goal
    RunIndex(0);
    MiddleScore();
    wait(100,msec);
    RunIndex(75);
    MoveTimePID(TestPara, -10, 0.3, 0.2, 45, false); // score
    RunIndex(60);
    MoveTimePID(TestPara, -10, 0.7, 0.2, 45, false); // score
    RunIndex(40);
    MoveTimePID(TestPara, -10, 1.4, 0.2, 45, false); // score
    RunIndex(60);
    MoveEncoderPID(TestPara, -80, 5.8 , 0.3, 45, true); // grab block blue
    wait(100,msec);
    MoveEncoderPID(TestPara, 40, 5.5 , 0.3, 45, true); // move in to goal
    RunIndex(30);
    MoveTimePID(TestPara, -10, 0.8, 0.2, 45, false); // score
    NeutralScore();
  


    //------------------------------------------------------------------------------------long goal 1
    //wait(100000,msec);
    
    MoveEncoderPID(TestPara, -80, 5, 0.3, 45,false); // drives to long goal
    Scrapper.set(true);
    RunIndex(100);
    MoveEncoderPID(TestPara, -80, 36.5, 0.3, 45,true); // drives to long goal
    TurnMaxTimePID(TestPara, 0, 0.2, true); // turns to matchloader
    MoveTimePID(TestPara, -80, 0.6 , 0.3, 0,false); // move into long goal
    HighScore();
    MoveTimePID(TestPara, -40, 0.9 , 0.3, 0,false); // score
    NeutralScore();
    MoveTimePID(TurnPara, 100, 0.40, 0.2, 0, false); // goes into matchload
    MoveTimePID(TurnPara, 40, 0.80, 0.2, 0, false); // slows down
    MoveTimePID(TestPara, 20, 1.3, 0.2, 0,false); // mactchload 
    
    MoveEncoderPID(TestPara, 80, 7, 0.4, 0,false); // move back
    TurnMaxTimePID(TestPara, -45, 0.2, false); // turns to aglin with goal
    MoveEncoderPID(TestPara, 80, 14.5, 0.4, -45,true); // moves to beside goal
    Scrapper.set(false);
    RunIndex(41);
    // maybe cahnge to like 0
    TurnMaxTimePID(TestPara, -1, 0.4, true); // turns to face other side of the field
    std::cout<< "go accross heading 1: " << Gyro.heading(degrees) <<std::endl;
    MoveEncoderPID(TestPara, 90, 40, 0.4, 0, false); // goes to other side of the field 
    MoveEncoderPID(TestPara, 50, 9, 0.2, 0, true); // slows down
    std::cout<< "after accross heading 1: " << Gyro.heading(degrees) <<std::endl;
    wait(100,msec);
    TurnMaxTimePID(TestPara, 45, 0.2, false); // turns to aglin with goal
    MoveEncoderPID(TestPara, 80, 7.3, 0.2, 45, true); // move to goal and matcjoader

    // -------------- SECOND QUARTER
    TurnMaxTimePID(TestPara, 180, 0.4, true); // turns to have back face long goal
    MoveTimePID(TestPara, -80, 0.5, 0.1, 180,true); // move to long goal
    // -------------- SECOND QUARTER----------------------------------------------------------
    HighScore();
    wait(100,msec);
    Scrapper.set(true);
    RunIndex(100);
    MoveTimePID(TestPara, -30, 1.8, 0.4, 180,true); // score
    NeutralScore();
    // Scrapper.set(true);
    MoveTimePID(TurnPara, 100, 0.40, 0.2, 180, false); // goes into matchload
    MoveTimePID(TurnPara, 40, 0.80, 0.2, 180, false); // slows down
    MoveTimePID(TestPara, 20, 1.3, 0.2, 180,false); // mactchload

    MoveTimePID(TestPara, -75, 0.6, 0.1, 180,true); // move to long goal
    MoveTimePID(TestPara, -40, 0.1, 0.1, 180,true); // move to long goal
    HighScore();
    
    MoveTimePID(TestPara, -60, 0.8, 0.4, 180,true); // score
    RunIndex(50);
    MoveTimePID(TestPara, -40, 1, 0.4, 180,true); // score
    Scrapper.set(false);
    RunIndex(0);
    wait(100,msec);
    
    wait(100,msec);
    MoveEncoderPID(TestPara, -90, 7, 0.1, -180, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 11, 0.1, -140, false); // curve towards park
    MoveEncoderPID(TestPara, -80, 25, 0.1, -120, false); // curve towards park
    NeutralScore();
    RunIndex(100);
    MoveEncoderPID(TestPara, -100, 30, 0.1, -100, true); // curve towards park
    //wait(100,msec);
    MoveTimePID(TestPara, 75, 1.7, 0.1, -90,false); // pickup blocks

    //Scrapper.set(true)

    //MoveTimePID(TestPara, 50, 1.5, 0.1, 87,false); // pickup blocks
    //MoveTimePID(TestPara, 60, 0.5, 0.1, 90,false); // pickup blocks
    wait(100,msec);
    MoveEncoderPID(TestPara, 90, 4, 0.1, -90, false); // back up
    wait(50,msec);
    MoveEncoderPID(TestPara, -90, 6, 0.1, -40, false); // curve around matchloader
    MoveTimePID(TestPara, 70, 1.0, 0.1, -90,false); // move into wall and reset
    std::cout<< "pre reset: " << Gyro.heading(degrees) <<std::endl;
    Gyro.setHeading(-90,degrees);
    //wait(100000,msec);
    //MoveEncoderPID(TestPara, 90, 3.4, 0.1,  -90, true); // turn into goal
    //MoveEncoderPID(TestPara, 90, 15, 0.1,  -180, false); // turn into goal
    //MoveTimePID(TestPara, -90, 0.3, 0.1, -180,false); // line up with goal

    MoveEncoderPID(TestPara, 80, 6.8, 0.1,  -90, true); // move back
    TurnMaxTimePID(TestPara, -180, 0.4, true); // turns to goal
    MoveTimePID(TestPara, -80, 0.55, 0.1, -180,false); // line up with goal
    HighScore();
    Scrapper.set(true);
    MoveTimePID(TestPara, -40, 0.65, 0.1, -180,false); // score
    // Scrapper.set(true);
    RunIndex(100);
    MoveTimePID(TurnPara, 100, 0.40, 0.2, 180, false); // goes into matchload
    NeutralScore();
    MoveTimePID(TurnPara, 40, 0.80, 0.2, 180, false); // slows down
    MoveTimePID(TestPara, 20, 1.2, 0.2, 180,false); // mactchload 
    MoveEncoderPID(TestPara, 80, 7, 0.3, 180,false); // move back
    TurnMaxTimePID(TestPara, 135, 0.2, false); // turns to walll
    MoveEncoderPID(TestPara, 80, 14.5, 0.4, 135 ,true); // moves to beside goal
    Scrapper.set(false);
    RunIndex(41);
    TurnMaxTimePID(TestPara, 178, 0.4, true); // turns to face other side of the field
    std::cout<< "go accross heading 2: " << Gyro.heading(degrees) <<std::endl;
    MoveEncoderPID(TestPara, 90, 40, 0.4, 179, false); // goes to other side ofthe field 
    MoveEncoderPID(TestPara, 50, 9, 0.2, 180, true); // slows down
    std::cout<< "after accross heading 2: " << Gyro.heading(degrees) <<std::endl;
    wait(100,msec);
    TurnMaxTimePID(TestPara, -135, 0.2, false); // turns to aglin with goal
    MoveEncoderPID(TestPara, 80, 7.5, 0.2, -135, true); // move to goal and matcjoader
    TurnMaxTimePID(TestPara, 0, 0.4, true); // turns to have back face long goal
    MoveTimePID(TestPara, -80, 0.4, 0.1, 0,true); // move to long goal
    HighScore();
    RunIndex(100);
    Scrapper.set(true);
    MoveTimePID(TestPara, -25, 1.8, 0.2, 0,true); // score
    NeutralScore();
    // Scrapper.set(true);
    MoveTimePID(TurnPara, 100, 0.40, 0.2, 0, false); // goes into matchload
    MoveTimePID(TurnPara, 40, 0.80, 0.2, 0, false); // slows down
    MoveTimePID(TestPara, 20, 1.25, 0.4, 0,false); // mactchload

    MoveTimePID(TestPara, -75, 0.6, 0.1, 0,true); // move to long goal
    MoveTimePID(TestPara, -40, 0.1, 0.1, 0,true); // move to long goal
    HighScore();
    MoveTimePID(TestPara, -60, 0.8, 0.4, 0,true); // score
    RunIndex(50);
    MoveTimePID(TestPara, -40, 1, 0.4, 0,true); // score
    Scrapper.set(false);
    MoveEncoderPID(TestPara, -100, 7, 0.1, 0, false); // curve towards park
    MoveEncoderPID(TestPara, -100, 10, 0.1, 40, false); // curve towards park
    MoveEncoderPID(TestPara, -110, 23, 0.1, 60, true); // curve towards park
    MoveEncoderPID(TestPara, -110, 34, 0.1, 100, true); // park
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;

    /*
    MoveEncoderPID(TestPara, -90, 7, 0.1, -180, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 10, 0.1, -140, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 23, 0.1, -120, true); // curve towards park
    RunIndex(100);
    MoveTimePID(TestPara, 70, 2.6, 0.1, -95,false); // pickup blocks

    //Scrapper.set(true);

    //MoveTimePID(TestPara, 50, 1.5, 0.1, 87,false); // pickup blocks
    //MoveTimePID(TestPara, 60, 0.5, 0.1, 90,false); // pickup blocks
    MoveEncoderPID(TestPara, 90, 4, 0.1, -90, false); // back up
    MoveEncoderPID(TestPara, -90, 6, 0.1, -50, false); // curve around matchloader
    MoveTimePID(TestPara, 90, 1, 0.1, -90,false); // move into wall
    Gyro.setHeading(-90,degrees);
    //MoveEncoderPID(TestPara, 90, 4, 0.1, -90, false); // back up
    MoveEncoderPID(TestPara, 90, 3.5, 0.1,  -90, true); // turn into goal
    MoveEncoderPID(TestPara, 90, 15, 0.1,  -180, false); // turn into goal
    HighScore();
    MoveTimePID(TestPara, -20, 1, 0.1, -180,false); // move into wall
    */
    
    








    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
 
    



    /*RunIndex(80);
    
    
    
    
    MoveTimePID(TestPara, 55, 0.5, 0.2, 0, false); // grab blocks in park
    MoveTimePID(TestPara, -20, 0.3, 0.02, 0, false); // grab blocks in park
    TurnMaxTimePID(TestPara, 10, 0.1, true); // turn
    TurnMaxTimePID(TestPara, -10, 0.1, true); // turn
    MoveTimePID(TestPara, 30, 0.3, 0.02, 0, false); // grab blocks in park
    MoveTimePID(TestPara, -30, 0.2, 0.02, 0, false); // grab blocks in park
    wait(250,msec);
    MoveTimePID(TestPara, 30, 0.2, 0.02, 0, false); // grab blocks in park
    MoveTimePID(TestPara, 90, 1.3, 0.02, 0, false); // grab blocks in park
    RunIndex(100);
    MoveTimePID(TestPara, -30, 0.2, 0.02, 0, false); // grab blocks in park
    TurnMaxTimePID(TestPara, 10, 0.1, true); // turn
    TurnMaxTimePID(TestPara, -10, 0.1, true); // turn
    MoveTimePID(TestPara, 40, 0.2, 0.02, 0, false); // grab blocks in park
    MoveTimePID(TestPara, -30, 0.2, 0.02, 0, false); // grab blocks in park
    wait(250,msec);
    MoveTimePID(TestPara, 40, 0.2, 0.02, 0, false); // grab blocks in park
    
    wait(250,msec);
    MoveTimePID(TestPara, -80, 0.6, 0.2, 0, true); // back out of park
    MoveTimePID(TestPara, 25, 0.8, 0.2, 0, true); // align with park
    Gyro.setHeading(0,degrees);
    MoveEncoderPID(TestPara, 80, 13 , 0.3, 0, true); // back up 
    TurnMaxTimePID(TestPara, 105, 0.4, true); // turns to 4 balls
    RunIndex(100);
    MoveEncoderPID(TestPara, -60, 15.6 , 0.3, 105, true); // grab blue block
    wait(400,msec);
    TurnMaxTimePID(TestPara, 45, 0.3, true); // turns to face middle goal
    MoveTimePID(TestPara, -40, 0.55, 0.2, 45, false); // backs up into middle goal
    RunIndex(0);
    MiddleScore();
    wait(100,msec);
    RunIndex(70);
    MoveTimePID(TestPara, -10, 0.3, 0.2, 45, false); // score
    RunIndex(60);
    MoveTimePID(TestPara, -10, 0.5, 0.2, 45, false); // score
    RunIndex(50);
    MoveTimePID(TestPara, -10, 0.3, 0.2, 45, false); // score
    RunIndex(30);
    MoveTimePID(TestPara, -10, 1.5, 0.2, 45, false); // score
    NeutralScore();
  


    //------------------------------------------------------------------------------------long goal 1
    MiddleScore();
    Scrapper.set(true);
    wait(100,msec);
    RunIndex(100);
    MoveEncoderPID(TestPara, -80, 42, 0.3, 45,true); // drives to long goal
    TurnMaxTimePID(TestPara, 0, 0.2, true); // turns to matchloader
    NeutralScore();
    MoveTimePID(TestPara, 40, 1.4 , 0.3, 0,false); // move into matchloader
    MoveTimePID(TestPara, 20, 1.1, 0.2, 0,false); // mactchload 
    
    MoveEncoderPID(TestPara, 80, 2, 0.4, 0,false); // move back
    Scrapper.set(false);
    TurnMaxTimePID(TestPara, 135, 0.4, true); // turns to face goal
    MoveEncoderPID(TestPara, -80, 13, 0.4, 135,true); // moves to beside goal
    TurnMaxTimePID(pp, 176, 0.3, true); // turns to face other side of the field
    Scrapper.set(true);
    RunIndex(41);
    MoveEncoderPID(TestPara, -90, 45, 0.5, 176, false); // goes to other side of the field 
    MoveEncoderPID(TestPara, -50, 9, 0.2, 180, true); // slows down
    TurnMaxTimePID(TestPara, 225, 0.2, true); // turns to aglin with goal
    MoveEncoderPID(TestPara, -80, 11, 0.2, 225, true); // move to goal and matcjoader

    // -------------- SECOND QUARTER
    TurnMaxTimePID(TestPara, 180, 0.5, true); // turns to have back face long goal
    MoveTimePID(TestPara, -70, 1, 0.1, 180,true); // move to long goal

    // -------------- SECOND QUARTER
    HighScore();
    wait(100,msec);
    RunIndex(100);
    MoveTimePID(TestPara, -25, 1.95, 0.4, 180,true); // score
    NeutralScore();
    Scrapper.set(true);
    MoveTimePID(TestPara, 43, 1.5 , 0.3, 180,false); // move forward into matchloader
    MoveTimePID(TestPara, 20, 1.3, 0.2, 180,false); // mactchload

    MoveTimePID(TestPara, -75, 0.7, 0.1, 180,true); // move to long goal
    HighScore();
    MoveTimePID(TestPara, -25, 2, 0.4, 180,true); // score
    Scrapper.set(false);
    RunIndex(0);
    MoveEncoderPID(TestPara, -90, 7, 0.1, -180, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 10, 0.1, -140, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 23, 0.1, -120, true); // curve towards park
    NeutralScore();
    RunIndex(100);
    MoveTimePID(TestPara, 70, 2.6, 0.1, -95,false); // pickup blocks

    //Scrapper.set(true);

    //MoveTimePID(TestPara, 50, 1.5, 0.1, 87,false); // pickup blocks
    //MoveTimePID(TestPara, 60, 0.5, 0.1, 90,false); // pickup blocks
    MoveEncoderPID(TestPara, 90, 4, 0.1, -90, false); // back up
    MoveEncoderPID(TestPara, -90, 6, 0.1, -50, false); // curve around matchloader
    MoveTimePID(TestPara, 90, 1, 0.1, -90,false); // move into wall and reset
    Gyro.setHeading(-90,degrees);
    wait(250,msec);
    //MoveEncoderPID(TestPara, 90, 4, 0.1, -90, false); // back up
    MoveEncoderPID(TestPara, 90, 3.5, 0.1,  -90, true); // turn into goal
    MoveEncoderPID(TestPara, 90, 15, 0.1,  -180, false); // turn into goal
    HighScore();
    MoveTimePID(TestPara, -20, 1, 0.1, -180,false); // score
    Scrapper.set(true);
    NeutralScore();
    RunIndex(100);
    MoveTimePID(TestPara, 43, 1.3 , 0.4, 180,false); // move into matchloader
    MoveTimePID(TestPara, 20, 1.2, 0.4, 180,false); // mactchload 
    MoveEncoderPID(TestPara, 80, 2, 0.4, 180,false); // move back
    Scrapper.set(false);
    TurnMaxTimePID(TestPara, -45, 0.6, true); // turns to face goal
    MoveEncoderPID(TestPara, -80, 10, 0.4, -45,true); // moves to beside goal
    TurnMaxTimePID(pp, -4, 0.4, true); // turns to face other side of the field
    Scrapper.set(true);
    RunIndex(41);
    MoveEncoderPID(TestPara, -90, 45, 0.5, -4, false); // goes to other side of the field 
    MoveEncoderPID(TestPara, -50, 9, 0.2, 0, true); // slows down
    TurnMaxTimePID(TestPara, 45, 0.2, true); // turns to aglin with goal
    MoveEncoderPID(TestPara, -80, 11, 0.2, 45, true); // move to goal and matcjoader

    // -------------- SECOND QUARTER
    TurnMaxTimePID(TestPara, 0, 0.5, true); // turns to have back face long goal
    MoveTimePID(TestPara, -80, 0.5, 0.1, 0,true); // move to long goal
    HighScore();
    RunIndex(100);
    MoveTimePID(TestPara, -25, 1.95, 0.4, 0,true); // score
    NeutralScore();
    Scrapper.set(true);
    MoveTimePID(TestPara, 43, 1.3 , 0.4, 0,false); // move forward into matchloader
    MoveTimePID(TestPara, 20, 1.3, 0.4, 0,false); // mactchload

    MoveTimePID(TestPara, -75, 0.7, 0.1, 0,true); // move to long goal
    HighScore();
    MoveTimePID(TestPara, -25, 2, 0.4, 0,true); // score
    Scrapper.set(false);
    RunIndex(0);

    MoveEncoderPID(TestPara, -90, 7, 0.1, 0, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 10, 0.1, 40, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 23, 0.1, 60, true); // curve towards park
    RunIndex(100);
    MoveTimePID(TestPara, 80, 1.2, 0.1, 88,false); // park

*/
}