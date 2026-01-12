#include "screen_gui.hpp"
#include "vex.h"
#include "cstring"

void drawCurvedRectangle(int xPos, int yPos, int length, int height, int radius);
void drawField(void);
void confirmCorner(void);
void drawEmptyRectangle(int x, int y, int width, int height);
void AutoSelection(void);
void drawCurvedBorder(int x, int y, int l, int h, int r, int t, char* col1, char* col2);
void drawSkillsCorner(bool SkillsCorner);
void AutonLogic(void);
void greyScreen(void);

void drawTiltedRectangle(int cx, int cy, double width, double height); // , color c = white
void fillTiltedRectangle(int centerX, int centerY, double width, double height);

// NAMES OF AUTONS GO INSIDE OF HERE

const char* leftAutos[] = {"3+6","9 Block","7 Wing","3+4","4 rush","mangos"}; // MAKE SURE THERE ARE FOUR ITEMS IN THE LIST
// autoselector values 1,2,3,4
// high side

const char* rightAutos[] = {"3+6","9 Block","Solo-AWP"," 7 Wing","3+4","test"}; // SAME FOR THIS ONE

// autoselector values 5,6,7,8 
// low side

// skills is autoselector value 9 btw


void DisplayAutoSelector(void)
{

  Brain.Screen.clearScreen();
  greyScreen();

//auton selector
Brain.Screen.setFont(monoM);
Brain.Screen.setFillColor(black);
Brain.Screen.setCursor(1,1);
Brain.Screen.setPenWidth(3);
Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.drawRectangle(1,25,100,50);

Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.drawRectangle(375,25,100,50);

Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.drawRectangle(1,100,100,50);

Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.drawRectangle(375,100,100,50);

Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.drawRectangle(1,175,100,50);

Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.drawRectangle(375,175,100,50);

Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.drawRectangle(187,25,100,50);

Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.drawRectangle(187,175,100,50);
}

void DisplayWords(void)
{
  Brain.Screen.setPenColor("#FFC0CB");
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
  Brain.Screen.setFillColor("#FFC0CB");
Brain.Screen.setPenColor(black);
if(AutoSelectorVal==1){
Brain.Screen.drawRectangle(1,25,100,50);
  Brain.Screen.setCursor(3,3);
  Brain.Screen.print("High Long");

Brain.Screen.setFillColor(black);
Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("HIGH SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("HIGH LONG");
Brain.Screen.setFont(monoM);
  Brain.Screen.setFillColor("#FFC0CB");

}

if(AutoSelectorVal==2){
Brain.Screen.drawRectangle(375,25,100,50);
    Brain.Screen.setCursor(3,40);
  Brain.Screen.print("High Basic");

Brain.Screen.setFillColor(black);

  Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("HIGH SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("High Basic");
Brain.Screen.setFont(monoM);
  Brain.Screen.setFillColor("#FFC0CB");
}

if(AutoSelectorVal==3){
  Brain.Screen.drawRectangle(1,100,100,50);
  Brain.Screen.setCursor(7,3);
  Brain.Screen.print("Low Basic");

Brain.Screen.setFillColor(black);

    Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("LOW SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("LOW BASIC......");
Brain.Screen.setFont(monoM);  
  Brain.Screen.setFillColor("#FFC0CB");
}

if(AutoSelectorVal==4){
Brain.Screen.drawRectangle(375,100,100,50);
  Brain.Screen.setCursor(7,40);
  Brain.Screen.print("LOW LONG");

Brain.Screen.setFillColor(black);

  Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("LOW SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("LOW LONG");
Brain.Screen.setFont(monoM); 
  Brain.Screen.setFillColor("#FFC0CB");

}

if(AutoSelectorVal==5){
  Brain.Screen.drawRectangle(1,175,100,50);
      Brain.Screen.setCursor(11,3);
  Brain.Screen.print("GS-AWP");

Brain.Screen.setFillColor(black);
    Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("GOAL SIDE");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("GS-AWP");
Brain.Screen.setFont(monoM); 
  Brain.Screen.setFillColor("#FFC0CB");

}

if(AutoSelectorVal==6){
Brain.Screen.drawRectangle(375,175,175,50);
Brain.Screen.setCursor(11,40);
Brain.Screen.print("SOLO AWP");
Brain.Screen.setFillColor(black);
Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#FFC0CB");
Brain.Screen.setCursor(3,10);
Brain.Screen.print("mango");
Brain.Screen.setCursor(4,10);
Brain.Screen.print("SOLO AWP");
Brain.Screen.setFont(monoM); 
  Brain.Screen.setFillColor("#FFC0CB");

  }

if(AutoSelectorVal==7){
  Brain.Screen.drawRectangle(187,175,100,50);
  Brain.Screen.setCursor(11,22);
  Brain.Screen.print("Skills");
  Brain.Screen.setFillColor(black);
  Brain.Screen.setFont(monoXL);
  Brain.Screen.setPenColor("#FFC0CB");
  Brain.Screen.setCursor(3,10);
  Brain.Screen.print("SKILLS");
  Brain.Screen.setCursor(4,10);
  Brain.Screen.print("SKILLS");
  Brain.Screen.setFont(monoM); 
  Brain.Screen.setFillColor("#FFC0CB");
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

void drawCurvedBorder(int x, int y, int l, int h, int r, int t, char* col1, char* col2) {
  Brain.Screen.setPenColor("#FFC0CB");
  Brain.Screen.setFillColor("#FFC0CB");
  drawCurvedRectangle(x,y,l,h,r);
  Brain.Screen.setPenColor("#000000");
  Brain.Screen.setFillColor("#000000");
  drawCurvedRectangle(x+t,y+t,l-(2*t),h-(2*t),r);
}

void drawEmptyRectangle(int x, int y, int width, int height)
{
  Brain.Screen.drawLine(x, y, x + width, y); // top edge
  Brain.Screen.drawLine(x, y + height, x + width, y + height); // bottom edge
  Brain.Screen.drawLine(x, y, x, y + height); // left edge
  Brain.Screen.drawLine(x + width, y, x + width, y + height); // right edge
}

void drawField(void)
{
  Brain.Screen.setPenColor("#000000");
  Brain.Screen.setFillColor("#808080");
  Brain.Screen.drawRectangle(26,0,239,239); // draws field background 26 0 107 107

  Brain.Screen.setPenColor("#FFA500");
  Brain.Screen.setFillColor("#FFA500");
  Brain.Screen.drawRectangle(105,35,82,10); // draws the top long goal
  Brain.Screen.drawRectangle(105,195,82,10); // draws the bottom long goal

  Brain.Screen.drawRectangle(29,38,4,4); // draws top left match load
  Brain.Screen.drawRectangle(29,198,4,4); // draws bottom left match load
  Brain.Screen.drawRectangle(258,38,4,4); // draws top right match load
  Brain.Screen.drawRectangle(258,198,4,4); // draws bottom right match load

  Brain.Screen.setPenColor("#000000"); // seperates the field into 4 sections
  Brain.Screen.drawLine(26,120,266,120);
  Brain.Screen.drawLine(146,0,146,239);

  Brain.Screen.setPenColor("#FF0000");
  Brain.Screen.setFillColor("#FF0000");
  Brain.Screen.drawRectangle(28,131,24,2); // draws red parking
  Brain.Screen.drawRectangle(50,106,2,25);
  Brain.Screen.drawRectangle(28,106,24,2);

  Brain.Screen.setPenColor("#0000FF");
  Brain.Screen.setFillColor("#0000FF");
  Brain.Screen.drawRectangle(240,131,24,2); // draws blue parking
  Brain.Screen.drawRectangle(240,106,2,25);
  Brain.Screen.drawRectangle(240,106,24,2);

  Brain.Screen.setPenColor("#000000ff");
  Brain.Screen.setFillColor("#808080");
  Brain.Screen.setFont(monoM);
  Brain.Screen.setCursor(4,6);
  Brain.Screen.print("Red-L");
  Brain.Screen.setCursor(9,6);
  Brain.Screen.print("Red-R");
  Brain.Screen.setCursor(4,19);
  Brain.Screen.print("Blue-R");
  Brain.Screen.setCursor(9,19);
  Brain.Screen.print("Blue-L");

  // bottom right is at (159,107);
  // 240x480
  drawCurvedBorder(310,50,140,50,10,2,"e","e");
  Brain.Screen.setPenColor("#FFC0CB");
  Brain.Screen.setFillColor("#000000");
  Brain.Screen.setFont(mono30);
  Brain.Screen.setCursor(3,23);
  Brain.Screen.print("Confirm");
  Brain.Screen.setFont(monoM);

  drawSkillsCorner(false);
  
  Brain.Screen.setPenColor("#dadada");
  Brain.Screen.setFillColor("#dadada");
  drawTiltedRectangle(146,120,50,10);
  fillTiltedRectangle(146,120,48,8);

  Brain.Screen.setPenColor("#c8c8c8");
  Brain.Screen.setFillColor("#c8c8c8");
  drawTiltedRectangle(146,120,10,50);
  fillTiltedRectangle(146,120,8,48);

}


int corner = 0;
bool confirmed = false;
bool pressing = false;
int OLDcorner = 0;

void drawSkillsCorner(bool SkillsCorner)
{
  if (SkillsCorner == false) {
  drawCurvedBorder(310,140,140,50,10,2,"e","e");
  Brain.Screen.setPenColor("#FFC0CB");
  Brain.Screen.setFillColor("#000000");
  Brain.Screen.setFont(mono30);
  Brain.Screen.setCursor(6,23);
  Brain.Screen.print("SKILLS");
  Brain.Screen.setFont(monoM);
  }
  if (SkillsCorner == true) {
  Brain.Screen.setPenColor("#FFC0CB");
  Brain.Screen.setFillColor("#FFC0CB");
  drawCurvedRectangle(310,140,140,50,10);
  Brain.Screen.setPenColor("#000000");
  Brain.Screen.setFillColor("#FFC0CB");
  Brain.Screen.setFont(mono30);
  Brain.Screen.setCursor(6,23);
  Brain.Screen.print("SKILLS");
  Brain.Screen.setFont(monoM);
  }
}
void confirmCorner(void)
{
  // Brain.Screen.setPenColor("#FFC0CB");
  // Brain.Screen.setFillColor("#000000");
  // drawCurvedBorder(325,95,100,50,10,2,"e","e");
  // Brain.Screen.drawRectangle(325,95,100,50);

  if (pressing) // just to prevent the previous menu pressing from affecting this menu
  {
    if (Brain.Screen.yPosition() < 120) // detects which corner you pressed
    {
      if (Brain.Screen.xPosition() < 145 && Brain.Screen.xPosition() > 26) corner = 1;
      if (Brain.Screen.xPosition() > 144 && Brain.Screen.xPosition() < 256) corner = 2;
    }
    else
    {
      if (Brain.Screen.xPosition() < 145 && Brain.Screen.xPosition() > 26) corner = 3;
      if (Brain.Screen.xPosition() > 144 && Brain.Screen.xPosition() < 256) corner = 4;
    }
  }

  if (Brain.Screen.xPosition() < 450 && Brain.Screen.xPosition() > 310) // confirm button detection
  {
    if (Brain.Screen.yPosition() < 190 && Brain.Screen.yPosition() > 140) corner = 5;
    else if (Brain.Screen.yPosition() < 100 && Brain.Screen.yPosition() > 50)
    {
      confirmed=true;
    }
  }

  if (OLDcorner != corner){ // makes sure that it the field doesn't constantly update
    Brain.Screen.clearScreen();
    greyScreen();
    drawField(); 
    drawSkillsCorner(false);
    Brain.Screen.setPenColor("#FFC0CB");
    if (corner == 1) drawEmptyRectangle(26,0,120,120); // draws rectangle around selected corner
    else if (corner == 2) drawEmptyRectangle(146,0,120,120);
    else if (corner == 3) drawEmptyRectangle(26,120,120,120);
    else if (corner == 4) drawEmptyRectangle(146,120,120,120);
    else if (corner == 5) drawSkillsCorner(true);
    OLDcorner = corner;
  }

  // Brain.Screen.setPenColor("#FFC0CB");
  // Brain.Screen.setFillColor("#000000");
  // Brain.Screen.setCursor(11,40);
  // Brain.Screen.print(corner);

  pressing = Brain.Screen.pressing();
}

int leftCounter;
int rightCounter;
int option = 0;
int buttonPressed = 0;

bool confirmed2 = false;

void AutoSelectionRefresh(void)
{
  // draws confirm button
  drawCurvedBorder(345,105,100,50,10,2,"e","e");
  Brain.Screen.setPenColor("#FFC0CB");
  Brain.Screen.setFillColor("#000000");
  Brain.Screen.setFont(monoM);
  Brain.Screen.setCursor(7,37);
  Brain.Screen.print("CONFIRM");

  drawCurvedBorder(185,45,100,50,10,2,"e","e"); // draws rectangles for the buttons
  drawCurvedBorder(35,45,100,50,10,2,"e","e");

  drawCurvedBorder(185,105,100,50,10,2,"e","e");
  drawCurvedBorder(35,105,100,50,10,2,"e","e");

  drawCurvedBorder(185,165,100,50,10,2,"e","e");
  drawCurvedBorder(35,165,100,50,10,2,"e","e");

  // displaying stuff on the screen part
  if (corner == 1 || corner == 4)
  {
    option = 1; // just for knowing which auto to run
    // drawCurvedBorder(185,42,100,50,10,2,"e","e"); // draws rectangles for the buttons
    // drawCurvedBorder(35,42,100,50,10,2,"e","e");
    // drawCurvedBorder(185,144,100,50,10,2,"e","e");
    // drawCurvedBorder(35,144,100,50,10,2,"e","e");

    Brain.Screen.setPenColor("#FFC0CB");
    Brain.Screen.setFillColor("#000000");
    Brain.Screen.setFont(monoM);

    leftCounter = sizeof(leftAutos)/sizeof(leftAutos[0]); // counts number of autos

    Brain.Screen.setCursor(4,7); // prints auto names
    Brain.Screen.print(leftAutos[0]);
    Brain.Screen.setCursor(4,21);
    Brain.Screen.print(leftAutos[1]);
    Brain.Screen.setCursor(7,6);
    Brain.Screen.print(leftAutos[2]);
    Brain.Screen.setCursor(7,21);
    Brain.Screen.print(leftAutos[3]);
    Brain.Screen.setCursor(10,7);
    Brain.Screen.print(leftAutos[4]);
    Brain.Screen.setCursor(10,21);
    Brain.Screen.print(leftAutos[5]);
  }

  if (corner == 2 || corner == 3)
  {
    option = 2; // just for deciding which auton
    // drawCurvedBorder(185,42,100,50,10,2,"#FFC0CB","#000000"); // draws rectangles for the border
    // drawCurvedBorder (35,42,100,50,10,2,"#FFC0CB","#000000");
    // drawCurvedBorder(185,144,100,50,10,2,"#FFC0CB","#000000");
    // drawCurvedBorder(35,144,100,50,10,2,"#FFC0CB","#000000");

    Brain.Screen.setPenColor("#FFC0CB");
    Brain.Screen.setFillColor("#000000");
    Brain.Screen.setFont(monoM);

    rightCounter = sizeof(rightAutos)/sizeof(rightAutos[0]);

    Brain.Screen.setCursor(4,7); // inserts words into rectangles
    Brain.Screen.print(rightAutos[0]);
    Brain.Screen.setCursor(4,21);
    Brain.Screen.print(rightAutos[1]);
    Brain.Screen.setCursor(7,6);
    Brain.Screen.print(rightAutos[2]);
    Brain.Screen.setCursor(7,21);
    Brain.Screen.print(rightAutos[3]);
    Brain.Screen.setCursor(10,6);
    Brain.Screen.print(rightAutos[4]);
    Brain.Screen.setCursor(10,21);
    Brain.Screen.print(rightAutos[5]);
  }
}

void AutoSelection(void) // displays auto selection screen
{
  if (Brain.Screen.xPosition() > 345 && Brain.Screen.xPosition() < 445) // detects confirm button
  {
    if (Brain.Screen.yPosition() > 105 && Brain.Screen.yPosition() < 155) confirmed2 = true; // tells loop in main.cpp to exit
  }

  if (pressing && !Brain.Screen.pressing() && corner != 5) // corner 5 is skills so it auto confirms
  {

    if (Brain.Screen.yPosition() > 45 && Brain.Screen.yPosition() < 99) // detects the button that you pressed
    {
      if (Brain.Screen.xPosition() > 35 && Brain.Screen.xPosition() < 135) buttonPressed = 1;
      if (Brain.Screen.xPosition() > 185 && Brain.Screen.xPosition() < 285) buttonPressed = 2;
    }
    else if (Brain.Screen.yPosition() > 105 && Brain.Screen.yPosition() < 155)
    {
      if (Brain.Screen.xPosition() > 35 && Brain.Screen.xPosition() < 135) buttonPressed = 3;
      if (Brain.Screen.xPosition() > 185 && Brain.Screen.xPosition() < 285) buttonPressed = 4;
    }
    else if (Brain.Screen.yPosition() > 165 && Brain.Screen.yPosition() < 215) // change this value
    {
      if (Brain.Screen.xPosition() > 35 && Brain.Screen.xPosition() < 135) buttonPressed = 5;
      if (Brain.Screen.xPosition() > 185 && Brain.Screen.xPosition() < 285) buttonPressed = 6;
    }  
    pressing = false; // prevents this from immediately looping again

    AutoSelectionRefresh();
    Brain.Screen.setPenColor("#FFC0CB"); // creates inverted colours
    Brain.Screen.setFillColor("#FFC0CB");
    switch (buttonPressed) // draws the curved rectangle depending on which position everything is in
    {
      case 1: drawCurvedRectangle(35,45,100,50,10); Brain.Screen.setCursor(4,7); break;
      case 2: drawCurvedRectangle(185,45,100,50,10); Brain.Screen.setCursor(4,21); break;
      case 3: drawCurvedRectangle(35,105,100,50,10); Brain.Screen.setCursor(7,6); break;
      case 4: drawCurvedRectangle(185,105,100,50,10); Brain.Screen.setCursor(7,21); break;
      case 5: drawCurvedRectangle(35,165,100,50,10); Brain.Screen.setCursor(10,6); break;
      case 6: drawCurvedRectangle(185,165,100,50,10); Brain.Screen.setCursor(10,21); break;
      default: break;
    }
    if (corner == 1 || corner == 4)
    {
      Brain.Screen.setPenColor("#000000");
      if (buttonPressed > 0) Brain.Screen.print(leftAutos[buttonPressed-1]); // 
    }
    if (corner == 2 || corner == 3)
    {
      Brain.Screen.setPenColor("#000000");
      if (buttonPressed > 0) Brain.Screen.print(rightAutos[buttonPressed-1]);
    }
  }

  else if (corner == 5 || corner == 0) confirmed2 = true; // automatically confirmed for skills
  pressing = Brain.Screen.pressing(); // detects button pressing
}

void AutonLogic(void) // shows confirmation and which auton is being run
{
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(black);
  Brain.Screen.setFont(monoXL);
  Brain.Screen.setPenColor("#FFC0CB");
  Brain.Screen.setCursor(5,5);
  Brain.Screen.print("AUTO CONFIRMED"); // shows confirmation message
  Brain.Screen.setCursor(2,5);

  if (corner == 5) // skills is corner 5
  {
    // Brain.Screen.setCursor(4,10);
    Brain.Screen.print("SKILLS");
    AutoSelectorVal = 13;
  }
  else if (option == 1 && buttonPressed > 0) // left side autos
  {
    switch (buttonPressed) // may want to switch buttonPressed-1 with actual values
    {
      case 1: Brain.Screen.print(leftAutos[buttonPressed-1]); AutoSelectorVal = 1; break;
      case 2: Brain.Screen.print(leftAutos[buttonPressed-1]); AutoSelectorVal = 2; break;
      case 3: Brain.Screen.print(leftAutos[buttonPressed-1]); AutoSelectorVal = 3; break;
      case 4: Brain.Screen.print(leftAutos[buttonPressed-1]); AutoSelectorVal = 4; break;
      case 5: Brain.Screen.print(leftAutos[buttonPressed-1]); AutoSelectorVal = 5; break;
      case 6: Brain.Screen.print(leftAutos[buttonPressed-1]); AutoSelectorVal = 6; break;
    }
  }
  else if (option == 2 && buttonPressed > 0) // left side autos
  {
    switch (buttonPressed) // may want to switch buttonPressed-1 with actual values
    {
      case 1: Brain.Screen.print(rightAutos[buttonPressed-1]); AutoSelectorVal = 7; break;
      case 2: Brain.Screen.print(rightAutos[buttonPressed-1]); AutoSelectorVal = 8; break;
      case 3: Brain.Screen.print(rightAutos[buttonPressed-1]); AutoSelectorVal = 9; break;
      case 4: Brain.Screen.print(rightAutos[buttonPressed-1]); AutoSelectorVal = 10; break;
      case 5: Brain.Screen.print(rightAutos[buttonPressed-1]); AutoSelectorVal = 11; break;
      case 6: Brain.Screen.print(rightAutos[buttonPressed-1]); AutoSelectorVal = 12; break;
    }
  }
  
  Brain.Screen.setCursor(3,5); 
  Brain.Screen.setPenColor("#FFC0CB");
  if (option == 1 && leftAutos[buttonPressed-1][0] != '\0') Brain.Screen.print("HIGH/LEFT SIDE"); // shows where to place bot 
  else if (option == 2 && rightAutos[buttonPressed-1][0] != '\0') Brain.Screen.print("LOW/RIGHT SIDE"); // shows where to place bot
  else if (corner == 5) Brain.Screen.print("mango"); 
}

void greyScreen(void) // was meant to change the background to grey but it looked ugly
{
  // #4F4F4F
  // Brain.Screen.setPenColor("#4F4F4F"); 
  // Brain.Screen.setFillColor("#4F4F4F");
  // Brain.Screen.drawRectangle(0,0,480,240);
}


void drawTiltedRectangle(int cx, int cy, double width, double height) 
{
  double angleRad = 45.0 * M_PI / 180.0;  // Fixed 45° rotation
  double cosA = cos(angleRad);
  double sinA = sin(angleRad);

  // Half dimensions for centering
  double hw = width / 2.0;
  double hh = height / 2.0;

  // Define rectangle corners relative to center
  double corners[4][2] = {
    {-hw, -hh}, // top-left
    { hw, -hh}, // top-right
    { hw,  hh}, // bottom-right
    {-hw,  hh}  // bottom-left
  };

  // Rotated and translated corner positions
  int X[4];
  int Y[4];

  for (int i = 0; i < 4; i++) {
    double xRot = corners[i][0] * cosA - corners[i][1] * sinA;
    double yRot = corners[i][0] * sinA + corners[i][1] * cosA;

    X[i] = round(cx + xRot);
    Y[i] = round(cy + yRot);
  }

  // Draw rectangle edges
  for (int i = 0; i < 4; i++) {
    int next = (i + 1) % 4;
    Brain.Screen.drawLine(X[i], Y[i], X[next], Y[next]);
  }
}

void fillTiltedRectangle(int centerX, int centerY, double width, double height) 
{
    double angleRad = 45.0 * M_PI / 180.0;
    double cosA = cos(angleRad);
    double sinA = sin(angleRad);

    // Half dimensions for centering
    double hw = width / 2.0;
    double hh = height / 2.0;

    // Loop over the "height" of the rectangle in small steps
    for (double h = -hh; h <= hh; h += 1.0) {
        // Left and right x positions before rotation
        double xLeft = -hw;
        double xRight = hw;

        double yPos = h;

        // Rotate the left point
        int Xs = round(centerX + xLeft * cosA - yPos * sinA);
        int Ys = round(centerY + xLeft * sinA + yPos * cosA);

        // Rotate the right point
        int Xe = round(centerX + xRight * cosA - yPos * sinA);
        int Ye = round(centerY + xRight * sinA + yPos * cosA);

        // Draw the horizontal "scanline"
        Brain.Screen.drawLine(Xs, Ys, Xe, Ye);
    }
}



  // Brain.Screen.setCursor(10,40);
  // Brain.Screen.print(confirmed2);
  // Brain.Screen.setCursor(11,40); // debugging purposes
  // Brain.Screen.print(Brain.Screen.xPosition());
  // Brain.Screen.setCursor(12,40);
  // Brain.Screen.print(Brain.Screen.yPosition());

  // ---------------------------------------------------------------------------------------------
  // button detection part -----------------------------------------------------------------------