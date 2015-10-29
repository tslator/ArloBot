#include "simpletools.h"
#include "arlobotconfig.h"
#include "motiondetect.h"

#define THRESHOLD 15
#define MAX_SAMPLES 5

static int hit_counter;

void MD_Init()
{
    hit_counter = 0;
}

uint8_t MD_Detected()
{    
    uint8_t motion_detected = 0;
#ifdef hasPIR
    int ii;
    
    for (int ii = 0; ii < MAX_SAMPLES; ii++) 
    { 
        // 5 x 200ms pause = 1000 between updates
        
        int state = input(PIR_PIN); // Check sensor (1) motion, (0) no motion
        
        // Count positive hits and make a call:
        if (state == 0) 
        {
            hit_counter = 0;
        } 
        else 
        {
            hit_counter++; // Increment on each positive hit
        }
        
        if (hit_counter > THRESHOLD) 
        {
            motion_detected = 1;
        } 
        else 
        {
            motion_detected = 0;
        }
        
        pause(200); // Pause 1/5 second before repeat
    }
#endif
    return motion_detected;
}