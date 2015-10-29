/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools
#include "arlodrive.h"
#define SPEED (100)
int main()                                    // Main function
{
    // Add startup code here.
    drive_speed(0, 0);                     // Start servos/encoders cog
    drive_setMaxSpeed(100);
    drive_setRampStep(3);              // Set ramping speed

 
    while(1)
    {
        print("Left/Right servos forward\n");
        drive_speed(SPEED, SPEED);
        pause(5000);
        drive_speed(0, 0);
        pause(500);
        print("Left/Right servos forward\n");
        drive_speed(-SPEED, -SPEED);
        pause(5000);
        drive_speed(0, 0);
        pause(500);
    }  
}
