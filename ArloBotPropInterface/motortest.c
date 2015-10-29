/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools
#include "servo.h"
#include "arlodrive.h"



void encodersLeds();

int main()                                    // Main function
{
    int ii;
    
    cog_run(encodersLeds, 128);
    servo_speed(12, 0);
    servo_speed(13, 0);
    
    for (ii = 0; ii < 10; ++ii)
    {
        pause(1000);
        print("Loop %d\n", ii);
        
        print("Left servo forward\n");
        servo_speed(12, 500);
        pause(5000);
        servo_speed(12, 0);
        pause(200);
        print("Left servo backward\n");
        servo_speed(12, -500);
        pause(5000);
        servo_speed(12, 0);
        pause(200);
        
        print("Right servo forward\n");
        servo_speed(13, -500);
        pause(5000);
        servo_speed(13, 0);
        pause(200);
        print("Right servo backward\n");
        servo_speed(13, 500);
        pause(5000);
        servo_speed(13, 0);       
        pause(200);
   }       
}

void encodersLeds()
{
    set_direction(26, 1);
    set_direction(27, 1);
    while(1)
    {
        set_output(26, input(14));
        set_output(27, input(15));        
    }        
}    
