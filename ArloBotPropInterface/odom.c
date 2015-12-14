#include <simpletools.h>
#include "arlodrive.h"
#include "utils.h"
#include "odom.h"

static ODOM_STATE OdomState;
static int odom_stack[128];
extern float dist_per_count;
extern float track_width;
uint8_t odom_enabled;

static void UpdateOdometry()
{
    static int ticksLeft;
    static int ticksRight;
    static int ticksLeftOld;
    static int ticksRightOld;
    static int speedLeft;
    static int speedRight;    
    
    ticksLeftOld = ticksLeft;
    ticksRightOld = ticksRight;
    drive_getTicks(&ticksLeft, &ticksRight);
    drive_getSpeedCalc(&speedLeft, &speedRight);

    int deltaTicksLeft      = ticksLeft - ticksLeftOld;
    int deltaTicksRight     = ticksRight - ticksRightOld;
    float deltaDistance    = 0.5f * (double) (deltaTicksLeft + deltaTicksRight) * dist_per_count;
    float deltaX           = deltaDistance * (double) cos(OdomState.heading);
    float deltaY           = deltaDistance * (double) sin(OdomState.heading);
    float radians_per_count = dist_per_count / track_width;
    float deltaHeading     = (double) (deltaTicksRight - deltaTicksLeft) * radians_per_count;

    OdomState.x_dist += deltaX;
    OdomState.y_dist += deltaY;
    OdomState.heading += deltaHeading;
    
    // limit heading to -Pi <= heading < Pi
    if (OdomState.heading > PI) 
    {
        OdomState.heading -= 2.0 * PI;
    } 
    else 
    {
        if (OdomState.heading <= -PI) 
        {
            OdomState.heading += 2.0 * PI;
        }
    }
    
    // http://webdelcire.com/wordpress/archives/527
    OdomState.linear_speed = ((speedRight * dist_per_count) + (speedLeft * dist_per_count)) / 2;
    OdomState.angular_speed = ((speedRight * dist_per_count) - (speedLeft * dist_per_count)) / track_width;    
}

static void BroadcastOdometry(void *par)
{
    static uint32_t last_time = 0;

    while (1) 
    {
        
        if ((millis() - last_time) > 100)
        {
            last_time = millis();
            if (odom_enabled)
            {
                UpdateOdometry();
            }
        }
    }
}

void GetOdomState(ODOM_STATE* state)
{
    memcpy(state, &OdomState, sizeof(OdomState));
}

void OdomStart()
{   
    odom_enabled = 0;
    memset(&OdomState, 0, sizeof(OdomState));
    
    cogstart(&BroadcastOdometry, NULL, odom_stack, sizeof(odom_stack));
}