#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);


void skills() { // NEGATIVE TURNS TO THE LEFT
    // declare initial conditions
    PIDDataSet TestPara={1.5,0.1,0.12};

    RunRoller(100);
    // MoveTimePID(TestPara, 80, 1.2, 0.3, 0,false); // drives through park
    // MoveTimePID(TestPara, 45, 2.3, 0.3, 5,false); // slows down 
    // MoveEncoderPID(TestPara, 60, 7, 0.3, 10, true); // backs up a bit so we have space to turn
    // wait(100,msec);
    // TurnMaxTimePID(TestPara, -45, 0.3, true); // turns
    // wait(500,msec);
    // MoveEncoderPID(TestPara, -90, 15.65, 0.3, -45, true); // goes to between matchload and long goal

    MoveEncoderPID(TestPara, -90, 21, 0.5, 0, true); // goes to between the matchload and long goal
    wait(150,msec);
    TurnMaxTimePID(TestPara, 90, 0.7, true); // turns so back faces long goal
    // MoveTimePID(TestPara, -80, 0.9, 0.4, 90,true); // score on long goal
    // RunTopRoller(100);
    // MoveTimePID(TestPara, -10, 2.1, 0.3, 90, true); // pushes into long goal a bit
    // wait(50,msec);
    // RunTopRoller(0);
    Scrapper.set(true);
    wait(100,msec);
    MoveTimePID(TestPara, 50, 1.4 , 0.4, 90,false); // move forward into matchloader
    MoveTimePID(TestPara, 20, 1.1, 0.4, 90,false); // mactchload

    MoveEncoderPID(TestPara, 50, 11, 0.2, 90, true); // moves away from matchloader
    Scrapper.set(false);
    // MoveTimePID(TestPara, -60, 1.2, 0.6, 90,true); // move to long goal
    // wait(50,msec);
    // RunTopRoller(100);
    // MoveTimePID(TestPara, -10, 1.95, 0.4, 90,true); // score
    // Scrapper.set(false);
    // MoveEncoderPID(TestPara, -40, 4, 0.2, 90, true); // moves away from long goal
    RunTopRoller(0);
    RunRoller(0);
    TurnMaxTimePID(TestPara, 0, 0.4, true); // turns to face wall
    MoveTimePID(TestPara, 40, 1.5, 0.4, 0, true); // aligns against wall
    wait(100,msec);
    Gyro.setHeading(0, degrees);
    wait(100,msec);
    MoveEncoderPID(TestPara, 40, 2, 0.4, 0, true); // backs off from wall
    TurnMaxTimePID(TestPara, -90, 0.6, true); // turns to face other side of the field
    MoveEncoderPID(TestPara, -100, 62, 0.4, -90, false); // goes to other side of the field
    MoveEncoderPID(TestPara, -50, 9, 0.2, -90, true); // slows down
    wait(100,msec);

    // second quarter
    TurnMaxTimePID(TestPara, 180, 0.4, true); // turns to face blue park
    MoveEncoderPID(TestPara, -70, 5.5, 0.3, 180, true); // drives forward to between long goal and matchload tube
    TurnMaxTimePID(TestPara, -90, 0.4, true); // turns to have back face long goal

    MoveTimePID(TestPara, -60, 1.2, 0.6, -90,true); // move to long goal
    wait(50,msec);
    RunRoller(100);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.95, 0.4, -90,true); // score
    RunTopRoller(0);
    RunRoller(0);

    Scrapper.set(true);
    RunRoller(100);
    MoveTimePID(TestPara, 50, 1.2 , 0.4, -90,false); // move forward into matchloader
    MoveTimePID(TestPara, 20, 1.3, 0.4, -90,false); // mactchload

    MoveTimePID(TestPara, -60, 1.2, 0.6, -90,true); // move to long goal
    wait(50,msec);
    RunTopRoller(100);
    Scrapper.set(false);
    MoveTimePID(TestPara, -10, 2, 0.4, -90,true); // score
    RunTopRoller(0);
    MoveEncoderPID(TestPara, -80, 7, 0.3, -90, true); // moves away from long goal
    TurnMaxTimePID(TestPara, -135, 0.4, true); // turns to face blue park again
    
    // curvy bit and 3 quarter
    MoveEncoderPID(TestPara, -100, 8, 0.3, -135, false); // drives towards the wall
    MoveEncoderPID(TestPara, -100, 6, 0.3, -155, false); // drives towareds blue park
    MoveEncoderPID(TestPara, -100, 4, 0.3, -165, false); // curves a bit
    MoveEncoderPID(TestPara, 80, 1.9, 0.3, 175, false); // backs up a bit so we have a better angle
    TurnMaxTimePID(TestPara, -179, 0.35, false); // turns for more accuracy
    MoveTimePID(TestPara, 50, 2.3, 0.3, -179, false); // goes into blue park
    MoveTimePID(TestPara, 40, 2.1, 0.2, -174, false); // slows down
    wait(100,msec);
    MoveEncoderPID(TestPara, 70, 9.5, 0.3, -163, true); // backs up a bit so we have space to turn
    TurnMaxTimePID(TestPara, 140, 0.5, true);

    wait(200,msec);
    MoveEncoderPID(TestPara, -100, 19.5, 0.3, 140, false); // goes to between long goal and matchload
    MoveEncoderPID(TestPara, -40, 3, 0.3, 140, false); // slows down
    wait(50,msec);
    TurnMaxTimePID(TestPara, -90, 0.7, false); // turns so back faces long goal

    MoveTimePID(TestPara, -60, 1.2, 0.6, -90,true); // move to long goal
    wait(50,msec);
    RunRoller(100);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.95, 0.4, -90,true); // score
    RunTopRoller(0);
    Scrapper.set(true);
    MoveTimePID(TestPara, 50, 1.2 , 0.4, -90,false); // move forward into matchloader
    MoveTimePID(TestPara, 20, 1.3, 0.4, -90,false); // mactchload

    MoveTimePID(TestPara, -60, 1.2, 0.6, -90,true); // move to long goal
    wait(50,msec);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.95, 0.4, -90,true); // score
    Scrapper.set(false);
    MoveEncoderPID(TestPara, -40, 4, 0.2, -90, true); // moves away from long goal
    RunTopRoller(0);
    RunRoller(0);

    // final corner
    TurnMaxTimePID(TestPara, 180, 0.4, true); // turns to face wall
    MoveTimePID(TestPara, 40, 1.5, 0.4, 180, true); // aligns against wall
    wait(100,msec);
    Gyro.setHeading(180, degrees);
    wait(100,msec);
    MoveEncoderPID(TestPara, 40, 2, 0.4, 180, true); // backs off from wall
    TurnMaxTimePID(TestPara, -90, 0.6, true); // turns to face other side of the field
    MoveEncoderPID(TestPara, -100, 62, 0.4, 90, false); // goes to other side of the field
    MoveEncoderPID(TestPara, -50, 9, 0.2, 90, true); // slows down
    wait(100,msec);
    TurnMaxTimePID(TestPara, 0, 0.4, true); // turns to face red park



    MoveEncoderPID(TestPara, -70, 5.1, 0.3, 0, true); // drives forward to between long goal and matchload tube
    TurnMaxTimePID(TestPara, 90, 0.4, true); // turns to have back face long goal

    Scrapper.set(true);
    RunRoller(100);
    MoveTimePID(TestPara, 50, 1.2 , 0.4, 90,false); // move forward into matchloader
    MoveTimePID(TestPara, 20, 1.3, 0.4, 90,false); // mactchload

    MoveTimePID(TestPara, -60, 1.2, 0.6, 90,true); // move to long goal
    wait(50,msec);
    RunTopRoller(100);
    Scrapper.set(false);
    MoveTimePID(TestPara, -10, 2, 0.4, 90,true); // score
    RunTopRoller(0);
    MoveEncoderPID(TestPara, -80, 7, 0.3, 90, true); // moves away from long goal
    TurnMaxTimePID(TestPara, 135, 0.4, true); // turns to face red park again
    
    // 
    MoveEncoderPID(TestPara, -100, 8, 0.3, 135, false); // drives towards the wall
    MoveEncoderPID(TestPara, -100, 6, 0.3, 155, false); // drives towareds blue park
    MoveEncoderPID(TestPara, -100, 4, 0.3, 165, false); // curves a bit
    MoveEncoderPID(TestPara, 80, 1.9, 0.3, -175, false); // backs up a bit so we have a better angle
    TurnMaxTimePID(TestPara, 179, 0.35, false); // turns for more accuracy
    MoveTimePID(TestPara, 50, 2.5, 0.3, 179, false); // goes into blue park

    int screenheading = Gyro.heading(degrees);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}