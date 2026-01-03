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

    RunIndex(70);
    MoveTimePID(TestPara, 55, 0.8, 0.2, 0, false); // grab blocks in park
    TurnMaxTimePID(TestPara, 40, 0.1, false);
    TurnMaxTimePID(TestPara, -40, 0.1, false);
    wait(100,msec);
    MoveTimePID(TestPara, 70, 0.86, 0.02, 0, false); // grab blocks in park
    TurnMaxTimePID(TestPara, 40, 0.1, false);
    TurnMaxTimePID(TestPara, -40, 0.1, false);
    TurnMaxTimePID(TestPara, 0, 0.1, false);
    MoveTimePID(TestPara, 70, 0.1, 0.02, 0, false); // grab blocks in park
    // RunIndex(0); // stops intake to twitch it
    // wait(50,msec);
    // RunIndex(70); // starts rerunning intake
    wait(250,msec);
    MoveTimePID(TestPara, -50, 0.8, 0.2, 0, true); // back out of park
    RunIndex(0);
    MoveTimePID(TestPara, 20, 1, 0.2, 0, true); // aglin with park
    Gyro.setHeading(0,degrees);
    MoveEncoderPID(TestPara, 80, 13.3 , 0.3, 0, true); // back up 
    TurnMaxTimePID(TestPara, -45, 0.2, true); // turns to 4 balls
    MoveEncoderPID(TestPara, 80, 9.2 , 0.3, -45, true); // back up to aglin with middle goal and blocks
    TurnMaxTimePID(TestPara, 45, 0.3, true); // turns to face 4 balls
    RunIndex(100);
    MoveEncoderPID(TestPara, -50, 2 , 0.3, 45, true); // grabs block
    wait(100,msec);
    MoveTimePID(TestPara, -60, 0.4, 0.2, 45, false); // backs up into middle goal
    MiddleScore();
    RunIndex(50);
    MoveTimePID(TestPara, -10, 0.7, 0.2, 45, false); // score
    RunIndex(70);
    MoveTimePID(TestPara, -10, 2, 0.2, 45, false); // score
  


    RunIndex(100);//------------------------------------------------------------------------------------long goal 1
    MoveEncoderPID(TestPara, -80, 41, 0.4, 45,true); // drives to long goal
    TurnMaxTimePID(TestPara, 0, 0.2, true); // turns to matchloader
    Scrapper.set(true);
    NeutralScore();
    MoveTimePID(TestPara, 45, 1.2 , 0.4, 0,false); // move into matchloader
    MoveTimePID(TestPara, 20, 0.8, 0.4, 0,false); // mactchload 
    MoveEncoderPID(TestPara, 80, 21.8, 0.4, -24,true); // moves to beside goal
    RunIndex(41);
    TurnMaxTimePID(TestPara, 0, 0.3, true); // turns to face other side of the field
    MoveEncoderPID(TestPara, 100, 50, 0.4, 0, false); // goes to other side of the field 
    MoveEncoderPID(TestPara, 50, 9, 0.2, 0, true); // slows down
    wait(100,msec);
    MoveEncoderPID(TestPara, 50, 8.75, 0.2, 62, true); // curve to aglin with long goal

    // -------------- SECOND QUARTER
    TurnMaxTimePID(TestPara, 180, 0.3, true); // turns to have back face long goal

    MoveTimePID(TestPara, -60, 1, 0.6, 180,true); // move to long goal
    HighScore();
    RunIndex(100);
    MoveTimePID(TestPara, -10, 1.95, 0.4, 180,true); // score
    NeutralScore();
    MoveTimePID(TestPara, 50, 1.3 , 0.4, 180,false); // move forward into matchloader
    MoveTimePID(TestPara, 15, 1.3, 0.4, 180,false); // mactchload

    MoveTimePID(TestPara, -60, 1.2, 0.6, 180,true); // move to long goal
    HighScore();
    MoveTimePID(TestPara, -10, 2, 0.4, 180,true); // score
    Scrapper.set(false);
    RunIndex(0);
    MoveEncoderPID(TestPara, -60, 3, 0.2, 180, true); // move forward
    TurnMaxTimePID(TestPara, -90, 0.3, true); // turns to other side of field
    MoveTimePID(TestPara, -50, 1.5, 0.4, -90,true); // align for heading
    Gyro.setHeading(-90,degrees);
    wait(10,msec);
    MoveEncoderPID(TestPara, -90, 85, 0.2, -90, false); // move to other side of field
    MoveEncoderPID(TestPara, -50, 14, 0.2, -90, true); // slows down



    TurnMaxTimePID(TestPara, 180, 0.2, true); // turns to matchloader ---------------------------------------------long goal 2
    Scrapper.set(true);
    NeutralScore();
    RunIndex(100);
    MoveTimePID(TestPara, 45, 1.2 , 0.4, 180,false); // move into matchloader
    MoveTimePID(TestPara, 20, 0.8, 0.4, 180,false); // mactchload 
    MoveEncoderPID(TestPara, 80, 22, 0.4, -204,true); // moves to beside goal
    RunIndex(41);
    TurnMaxTimePID(TestPara, 180, 0.3, true); // turns to face other side of the field
    MoveEncoderPID(TestPara, 100, 50, 0.4, 180, false); // goes to other side of the field 
    MoveEncoderPID(TestPara, 50, 9, 0.2, 180, true); // slows down
    wait(100,msec);
    MoveEncoderPID(TestPara, 50, 8.6, 0.2, 242, true); // curve to aglin with long goal

    // -------------- LAST QUARTER
    TurnMaxTimePID(TestPara, 0, 0.3, true); // turns to have back face long goal

    MoveTimePID(TestPara, -60, 1, 0.6, 0,true); // move to long goal
    HighScore();
    RunIndex(100);
    MoveTimePID(TestPara, -10, 1.95, 0.4, 0,true); // score
    NeutralScore();
    MoveTimePID(TestPara, 50, 1.3 , 0.4, 0,false); // move forward into matchloader
    MoveTimePID(TestPara, 15, 1.3, 0.4, 0,false); // mactchload

    MoveTimePID(TestPara, -60, 1.2, 0.6, 0,true); // move to long goal
    HighScore();
    MoveTimePID(TestPara, -10, 2, 0.4, 0,true); // score
    Scrapper.set(false);
    RunIndex(0);

    MoveEncoderPID(TestPara, -60, 5, 0.2, 0, true); // move forward ----------------------------------------- park
    TurnMaxTimePID(TestPara, 90, 0.3, true); // turns to other side of field
    MoveEncoderPID(TestPara, -80, 34, 0.2, 90, true); // move to park
    TurnMaxTimePID(TestPara, 0, 0.3, true); // turns to park
    MoveTimePID(TestPara, 60, 1.5, 0.2, 0,true); // park













    /*
    RunRoller(100);
    MoveEncoderPID(TestPara, -90, 21, 0.5, 0, true); // goes to between the matchload and long goal
    wait(150,msec);
    TurnMaxTimePID(TestPara, 90, 0.3, true); // turns so back faces long goal
    Scrapper.set(true);
    wait(100,msec);
    MoveTimePID(TestPara, 40, 1.3 , 0.4, 90,false); // move forward into matchloader
    MoveTimePID(TestPara, 15, 1.3, 0.4, 90,false); // mactchload

    MoveEncoderPID(TestPara, 50, 11, 0.2, 90, true); // moves away from matchloader
    Scrapper.set(false);
    RunTopRoller(0);
    RunRoller(0);
    TurnMaxTimePID(TestPara, 0, 0.3, true); // turns to face wall
    MoveTimePID(TestPara, 50, 0.8, 0.4, 0, true); // aligns against wall
    wait(100,msec);
    Gyro.setHeading(0, degrees);
    wait(100,msec);
    MoveEncoderPID(TestPara, 40, 1.9, 0.4, 0, true); // backs off from wall
    TurnMaxTimePID(TestPara, -90, 0.3, true); // turns to face other side of the field
    MoveEncoderPID(TestPara, -100, 62, 0.4, -90, false); // goes to other side of the field
    MoveEncoderPID(TestPara, -50, 9, 0.2, -90, true); // slows down
    wait(100,msec);

    // second quarter
    TurnMaxTimePID(TestPara, 180, 0.3, true); // turns to face blue park
    MoveEncoderPID(TestPara, -70, 5.6, 0.3, 180, true); // drives forward to between long goal and matchload tube
    TurnMaxTimePID(TestPara, -90, 0.3, true); // turns to have back face long goal

    MoveTimePID(TestPara, -60, 1.2, 0.6, -90,true); // move to long goal
    wait(50,msec);
    RunRoller(100);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.95, 0.4, -90,true); // score
    RunTopRoller(0);
    RunRoller(0);

    Scrapper.set(true);
    RunRoller(100);
    MoveTimePID(TestPara, 40, 1.6 , 0.4, -90,false); // move forward into matchloader
    MoveTimePID(TestPara, 15, 1.3, 0.4, -90,false); // mactchload

    MoveTimePID(TestPara, -60, 1.2, 0.6, -90,true); // move to long goal
    wait(50,msec);
    RunTopRoller(100);
    Scrapper.set(false);
    MoveTimePID(TestPara, -10, 2, 0.4, -90,true); // score
    RunTopRoller(0);
    MoveEncoderPID(TestPara, -80, 7, 0.3, -90, true); // moves away from long goal
    TurnMaxTimePID(TestPara, -135, 0.2, true); // turns to face blue park again
    
    // curvy bit and 3 quarter
    MoveEncoderPID(TestPara, -100, 10, 0.3, -135, false); // drives towards the wall
    MoveEncoderPID(TestPara, -100, 6, 0.3, -155, false); // drives towareds blue park
    MoveEncoderPID(TestPara, -100, 4, 0.3, -165, false); // curves a bit
    MoveEncoderPID(TestPara, 80, 1.9, 0.3, 175, false); // backs up a bit so we have a better angle
    TurnMaxTimePID(TestPara, -175, 0.2, false); // turns for more accuracy
    RunRoller(80);
    MoveTimePID(TestPara, 55, 2.0, 0.3, -174, false); // goes into blue park
    MoveTimePID(TestPara, 40, 1.9, 0.2, -174, false); // slows down
    wait(100,msec);
    MoveEncoderPID(TestPara, 70, 9.5, 0.3, -163, true); // backs up a bit so we have space to turn
    TurnMaxTimePID(TestPara, 140, 0.2, true);
    RunRoller(100);

    wait(200,msec);
    MoveEncoderPID(TestPara, -100, 18.8, 0.3, 140, false); // goes to between long goal and matchload
    MoveEncoderPID(TestPara, -40, 3.2, 0.3, 140, false); // slows down
    wait(50,msec);
    TurnMaxTimePID(TestPara, -90, 0.3, false); // turns so back faces long goal

    MoveTimePID(TestPara, -60, 1.2, 0.6, -90,true); // move to long goal
    wait(50,msec);
    RunRoller(100);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.95, 0.4, -90,true); // score
    RunTopRoller(0);
    Scrapper.set(true);
    MoveTimePID(TestPara, 40, 1.6 , 0.4, -90,false); // move forward into matchloader
    MoveTimePID(TestPara, 10, 1.3, 0.4, -90,false); // mactchload

    MoveTimePID(TestPara, -60, 1.2, 0.6, -90,true); // move to long goal
    wait(50,msec);
    RunTopRoller(100);
    MoveTimePID(TestPara, -10, 1.95, 0.4, -90,true); // score
    Scrapper.set(false); 
    MoveEncoderPID(TestPara, -40, 4, 0.2, -90, true); // moves away from long goal
    RunTopRoller(0);
    RunRoller(0);

    // final corner
    TurnMaxTimePID(TestPara, 180, 0.3, true); // turns to face wall
    MoveTimePID(TestPara, 50, 0.8, 0.4, 180, true); // aligns against wall
    wait(100,msec);
    Gyro.setHeading(180, degrees);
    wait(100,msec);
    MoveEncoderPID(TestPara, 40, 1.8, 0.4, 180, true); // backs off from wall
    TurnMaxTimePID(TestPara, 90, 0.3, true); // turns to face other side of the field
    MoveEncoderPID(TestPara, -100, 62, 0.4, 90, false); // goes to other side of the field
    MoveEncoderPID(TestPara, -50, 9, 0.2, 90, true); // slows down
    wait(100,msec);
    TurnMaxTimePID(TestPara, 0, 0.3, true); // turns to face red park



    MoveEncoderPID(TestPara, -70, 5.75, 0.3, 0, true); // drives forward to between long goal and matchload tube
    TurnMaxTimePID(TestPara, 90, 0.3, true); // turns to have back face long goal

    Scrapper.set(true);
    RunRoller(100);
    MoveTimePID(TestPara, 40, 1.6 , 0.4, 90,false); // move forward into matchloader
    MoveTimePID(TestPara, 15, 1.3, 0.4, 90,false); // mactchload

    MoveTimePID(TestPara, -60, 1.2, 0.6, 90,true); // move to long goal
    wait(50,msec);
    RunTopRoller(100);
    Scrapper.set(false);
    MoveTimePID(TestPara, -10, 2, 0.4, 90,true); // score
    RunTopRoller(0);
    MoveEncoderPID(TestPara, -80, 7, 0.3, 90, true); // moves away from long goal
    TurnMaxTimePID(TestPara, 45, 0.2, true); // turns to face red park again
    
    // 
    MoveEncoderPID(TestPara, -100, 8.2, 0.3, 45, false); // drives towards the wall
    MoveEncoderPID(TestPara, -100, 6, 0.3, 35, false); // drives towareds blue park
    MoveEncoderPID(TestPara, -100, 4, 0.3, 25, false); // curves a bit
    MoveEncoderPID(TestPara, 80, 1.9, 0.3, 0, false); // backs up a bit so we have a better angle
    TurnMaxTimePID(TestPara, 5, 0.2, false); // turns for more accuracy
    MoveTimePID(TestPara, 57, 1.5, 0.3, 5, false); // goes into red park 
    MoveTimePID(TestPara, 40, 0.7, 0.3, 5, false); // goes into red park
    MoveTimePID(TestPara, -70, 0.9, 0.3, 5, false); // goes into red park
    */

    int screenheading = Gyro.heading(degrees); 
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(monoL);
    Brain.Screen.setPenColor("#808080");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("HEADING:");
    Brain.Screen.setCursor(4,10);
    Brain.Screen.print(screenheading);
    
}