#include "screen_gui.hpp"
#include "vex.h"

void drawCurvedRectangle(int xPos, int yPos, int length, int height, int radius);
void drawField(void);

void DisplayAutoSelector(void)
{

  Brain.Screen.clearScreen();

//auton selector
Brain.Screen.setFont(monoM);
Brain.Screen.setFillColor(black);
Brain.Screen.setCursor(1,1);
Brain.Screen.setPenWidth(3);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.drawRectangle(1,25,100,50);

Brain.Screen.setPenColor("#39FF14");
Brain.Screen.drawRectangle(375,25,100,50);

Brain.Screen.setPenColor("#39FF14");
Brain.Screen.drawRectangle(1,100,100,50);

Brain.Screen.setPenColor("#39FF14");
Brain.Screen.drawRectangle(375,100,100,50);

Brain.Screen.setPenColor("#39FF14");
Brain.Screen.drawRectangle(1,175,100,50);

Brain.Screen.setPenColor("#39FF14");
Brain.Screen.drawRectangle(375,175,100,50);

Brain.Screen.setPenColor("#39FF14");
Brain.Screen.drawRectangle(187,5,100,50);

Brain.Screen.setPenColor("#39FF14");
Brain.Screen.drawRectangle(187,175,100,50);
}

void DisplayWords(void)
{
  Brain.Screen.setPenColor("#39FF14");
  Brain.Screen.setFont(monoM);
  Brain.Screen.setCursor(3,3);
  Brain.Screen.print("High Long");

  Brain.Screen.setCursor(7,3);
  Brain.Screen.print("Low Basic");

    Brain.Screen.setCursor(11,3);
  Brain.Screen.print("GS-AWP");

    Brain.Screen.setCursor(3,40);
  Brain.Screen.print("High Basic");

  Brain.Screen.setCursor(7,40);
  Brain.Screen.print("Low Long");

    Brain.Screen.setCursor(11,40);
  Brain.Screen.print("SOLO AWP");

      Brain.Screen.setCursor(11,22);
  Brain.Screen.print("Skills");
}

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
int AutoSelectorVal;

void UpdateDynamic(void)
{
  DisplayAutoSelector();
  DisplayWords();
  Brain.Screen.setFillColor("#39FF14");
Brain.Screen.setPenColor(black);
if(AutoSelectorVal==1){
Brain.Screen.drawRectangle(1,25,100,50);
  Brain.Screen.setCursor(3,3);
  Brain.Screen.print("High Long");

Brain.Screen.setFillColor(black);
Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("HIGH SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("HIGH LONG");
Brain.Screen.setFont(monoM);
  Brain.Screen.setFillColor("#39FF14");

}

if(AutoSelectorVal==2){
Brain.Screen.drawRectangle(375,25,100,50);
    Brain.Screen.setCursor(3,40);
  Brain.Screen.print("High Basic");

Brain.Screen.setFillColor(black);

  Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("HIGH SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("High Basic");
Brain.Screen.setFont(monoM);
  Brain.Screen.setFillColor("#39FF14");
}

if(AutoSelectorVal==3){
  Brain.Screen.drawRectangle(1,100,100,50);
  Brain.Screen.setCursor(7,3);
  Brain.Screen.print("Low Basic");

Brain.Screen.setFillColor(black);

    Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("LOW SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("LOW BASIC......");
Brain.Screen.setFont(monoM);  
  Brain.Screen.setFillColor("#39FF14");
}

if(AutoSelectorVal==4){
Brain.Screen.drawRectangle(375,100,100,50);
  Brain.Screen.setCursor(7,40);
  Brain.Screen.print("LOW LONG");

Brain.Screen.setFillColor(black);

  Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("LOW SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("LOW LONG");
Brain.Screen.setFont(monoM); 
  Brain.Screen.setFillColor("#39FF14");

}

if(AutoSelectorVal==5){
  Brain.Screen.drawRectangle(1,175,100,50);
      Brain.Screen.setCursor(11,3);
  Brain.Screen.print("GS-AWP");

Brain.Screen.setFillColor(black);
    Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("GOAL SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("GS-AWP");
Brain.Screen.setFont(monoM); 
  Brain.Screen.setFillColor("#39FF14");

}

if(AutoSelectorVal==6){
Brain.Screen.drawRectangle(375,175,175,50);
Brain.Screen.setCursor(11,40);
Brain.Screen.print("SOLO AWP");
Brain.Screen.setFillColor(black);
Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("mango");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("SOLO AWP");
Brain.Screen.setFont(monoM); 
  Brain.Screen.setFillColor("#39FF14");

  }

if(AutoSelectorVal==7){
  Brain.Screen.drawRectangle(187,175,100,50);
  Brain.Screen.setCursor(11,22);
  Brain.Screen.print("Skills");

Brain.Screen.setFillColor(black);
      Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("SKILLS");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("SKILLS");
Brain.Screen.setFont(monoM); 
  Brain.Screen.setFillColor("#39FF14");

}

}

void drawCurvedRectangle(int xPos, int yPos, int length, int height, int radius) {
  Brain.Screen.drawRectangle(xPos + radius, yPos, length - 2 * radius, height); // center horizontal
  Brain.Screen.drawRectangle(xPos, yPos + radius, length, height - 2 * radius); // center vertical

  Brain.Screen.drawCircle(xPos + radius, yPos + radius, radius); // top-left
  Brain.Screen.drawCircle(xPos + length - radius, yPos + radius, radius); // top-right
  Brain.Screen.drawCircle(xPos + radius, yPos + height - radius, radius); // bottom-left
  Brain.Screen.drawCircle(xPos + length - radius, yPos + height - radius, radius); // bottom-right
}

void drawField(void)
{
  Brain.Screen.setPenColor("#000000ff");
  Brain.Screen.setFillColor("#808080");
  Brain.Screen.drawRectangle(26,0,107,107); // draws field background

  Brain.Screen.setPenColor("#ff9500ff");
  Brain.Screen.setFillColor("#ff9500ff");
  Brain.Screen.drawRectangle(62,16,36,4); // draws the top long goal
  Brain.Screen.drawRectangle(62,88,36,4); // draws the bottom long goal

  Brain.Screen.drawRectangle(27,17,1,1); // draws top left match load
  Brain.Screen.drawRectangle(27,89,1,1); // draws bottom left match load
  Brain.Screen.drawRectangle(133,17,1,1); // draws top right match load
  Brain.Screen.drawRectangle(133,89,1,1); // draws bottom right match load

  // bottom right is at (159,107);
}
