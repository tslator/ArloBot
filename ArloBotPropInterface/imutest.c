/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include <simpletools.h>                      // Include simple tools
#include <math.h>
//#include <fdserial.h>
#include "imu2.h"
#include "utils.h"
#include "i2cbus.h"

static IMU_STATE imu_state;
static uint32_t last_time;
//static fdserial* term;

float CalcHeading(IMU_STATE* state)
{
    //Direction (y>0) = 90 - [arcTAN(x/y)]*180/PI
    //Direction (y<0) = 270 - [arcTAN(x/y)]*180/PI
    //Direction (y=0, x<0) = 180.0
    //Direction (y=0, x>0) = 0.0
    
    if (state->mag_y == 0.0)
    {
        if (state->mag_x < 0)
        {
            return 180.0;
        }            
        if (state->mag_x > 0)
        {
            return 0.0;
        }            
    }
    
    if (state->mag_y > 0)
    {
        return 90 - atan(state->mag_x/state->mag_y)*180/PI;
    }
    
    if (state->mag_y < 0)
    {
        return 270 - atan(state->mag_x/state->mag_y)*180/PI;
    }
    
}    

int main()                                    // Main function
{
    float heading;
    //simpleterm_close(); // Close simplex serial terminal
    //term = fdserial_open(31, 30, 0, 115200); // Open Full Duplex serial connection
    
    last_time = millis();
    InitI2C();
    InitImu();
    
    while(1)
    {
        if ( (millis() - last_time) > 1000)
        {
            last_time = millis();
            ReadImu(&imu_state);
            
            heading = CalcHeading(&imu_state);
            print("X: %.3f, Y: %.3f, Z: %.3f, H: %.3f\n", imu_state.mag_x, imu_state.mag_y, imu_state.mag_z, heading);
        }            
            
    }  
}
