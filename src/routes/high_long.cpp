#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
#include <iostream>
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void high_long() { // NEGATIVE TURNS TO THE LEFT
    timer stopwatch;
    // declare initial conditions
    PIDDataSet TestPara={1.5,0.1,0.15};
    PIDDataSet TurnPara={1.5,0.1,0.12};
    RunIndex(100);
    MoveEncoderPID(TurnPara, -100, 11, 0.2, -30, false); // goes forward
    // Scrapper.set(true);
    MoveEncoderPID(TurnPara, -100, 18.6, 0.2, -38, false); // curves towards blocks under long
    // Scrapper.set(false);
    MoveEncoderPID(TurnPara, -100, 10.1, 0.2, -85, false); // goes forward toward blocks
    Scrapper.set(true);
    MoveTimePID(TurnPara, 100, 0.15, 0.2, -85, false);
    MoveEncoderPID(TurnPara, 100, 4, 0.2, -65, false); // moves away
    MoveEncoderPID(TurnPara, 100, 16, 0.2, 20, false); // goes back
    MoveEncoderPID(TurnPara, 100, 13.75, 0.2, 65, false); // goes back towards long goal and matchload
    TurnMaxTimePID(TurnPara, -180, 0.25, false); // turns to matchload
    MoveTimePID(TurnPara, 100, 0.36, 0.2, -180, false); // goes into matchload
    MoveTimePID(TurnPara, 50, 0.4, 0.2, -180, false); // slows down
    wait(100,msec);
    MoveTimePID(TurnPara, 100, 0.3, 0.2, -180, false); // rams in again
    // HighScore();
    // wait(20,msec);
    // NeutralScore();
    MoveTimePID(TurnPara, -80, 0.7, 0.2, 176, false); // goes backwards into long goal
    HighScore(); // activates long goal scoring
    MoveTimePID(TurnPara, -40, 1.6, 0.2, 180, false); // pushes into long goal

    // wing code
    MoveEncoderPID(TestPara, -90, 9.2, 0.4, 177, false); // goes away from long goal
    Wings.set(false); // lowers wings
    wait(100,msec);
    NeutralScore(); // stops rolling block violations

    MoveEncoderPID(TestPara, 100, 8.7, 0.4, -159, false); // goes to the side of long goal a bit

    MoveEncoderPID(TestPara, 100, 14, 0.6, 179, false); // backs up to wing
    wait(150,msec);
    Move(-35,0);
    wait(100,msec);
    std::cout<< "time: " <<stopwatch/1000.0<<std::endl;
    /*
    MoveEncoderPID(TestPara, -90, 7, 0.1, 0, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 10, 0.1, 40, false); // curve towards park
    MoveEncoderPID(TestPara, -90, 23, 0.1, 60, true); // curve towards park
    RunIndex(100);
    MoveEncoderPID(TestPara, -80, 35, 0.1, 90, true); // park
    */
    /*
    TurnMaxTimePID(TestPara, -20, 0.1, true); // turns to 3 balls
    RunIndex(100); // activates intake
    MoveEncoderPID(TestPara, -70, 8 , 0.3,-20,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -25, 9 , 0.3,-20,true); // slows down
    MoveEncoderPID(TestPara, -70, 5.9 , 0.3,-20,true); // drives towards line 
    RunIndex(100);
    MoveEncoderPID(TestPara, -60, 11.3 , 0.3,-80,true); // grab balls under long goal
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -60, 2 , 0.3,-80,true); // moves forward a bit more
    wait(250,msec);
    
    TurnMaxTimePID(TestPara, -70, 0.1, true); // turns away
    MoveEncoderPID(TestPara, 70, 15 , 0.3,-70,true); // moves awaay from under long goal
    TurnMaxTimePID(TestPara, -136, 0.3, true); // turns to face between long goal and match load
    MoveEncoderPID(TestPara, -70, 27.6, 0.4, -136,false); // drives to between long goal and match load
    //MoveEncoderPID(TestPara, -60, 39.1, 0.4, -130,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.2, true); // turns so back faces long goal
    MoveTimePID(TestPara, -70, 0.68, 0.5, 180,false); // move to long goal to score
    HighScore();
    RunIndex(100);
    MoveTimePID(TestPara, -10, 1.5, 0.4, 180,false); // slows down to score
    NeutralScore();
    MoveTimePID(TestPara, 50, 1.7 , 0.4, 180,false); // move forward into matchloader
    MoveTimePID(TestPara, 20, 0.1, 0.4, 180,false); // mactchload
    MoveTimePID(TestPara, -60, 1, 0.6, 180,false); // move to long goal
    wait(100,msec);
    HighScore();
    MoveTimePID(TestPara, -10, 10, 0.4, 180,false); // score // (change time value back to 1 later)
    // RunRoller(0);
    // RunTopRoller(0);
    // MoveTimePID(TestPara, 100, 0.36, 0.4, 180,false); // go forwards to prepare ram
    // wait(50,msec);
    // MoveTimePID(TestPara, -100, 1, 0.3, 180,false); // ram
    wait(15000,msec);
    */
}
/*TurnMaxTimePID(TestPara, -18, 0.5, true); // turns to 3 balls
    RunRoller(100); // activates intake
    MoveEncoderPID(TestPara, -50, 10 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    MoveEncoderPID(TestPara, -15, 9 , 0.3,-18,true); // drives and turns towards the 3 blocks near center
    TurnMaxTimePID(TestPara, -50, 0.2, true); // turns to center
    RunRoller(0); 
    Scrapper.set(true);
    MoveEncoderPID(TestPara, -55, 13.5 , 0.3,-50,true); // move to dispurt
    MoveEncoderPID(TestPara, 55, 13.5 , 0.3,-50,true); // move to dispurt
    TurnMaxTimePID(TestPara, -120, 0.4, true); // turns to matchloader
    MoveEncoderPID(TestPara, -60, 20, 0.4, -120,false); // drives to long goal
    TurnMaxTimePID(TestPara, 180, 0.5, true); // turns to matchloader
    RunRoller(100); // activates intake to matchload
    MoveTimePID(TestPara, 55, 1 , 0.4, 180,false); // move into matchloader
    wait(200,msec);
    MoveTimePID(TestPara, -50, 1.5, 0.4, 180,false); // move to long goal
    RunTopRoller(100);
    wait(3000,msec);
    MoveTimePID(TestPara, 50, 1.5, 0.4, 180,false); // move into matchloader again
    Brain.Screen.setFont(monoXL);
    Brain.Screen.setPenColor("#39FF14");
    Brain.Screen.setCursor(3,10);
    Brain.Screen.print("6-7 Blocks Scored");
    wait(15000,msec);*/