#include <simpletools.h>
#include "arlodrive.h"
#include "utils.h"
#include "odom.h"

extern float dist_per_count;
extern float track_width;

static ODOM_STATE OdomState;
static int odom_stack[40 + 20];

static int ticksLeft;
static int ticksRight;
static int ticksLeftOld;
static int ticksRightOld;
static int speedLeft;
static int speedRight;    

#define DELTA_TICKS_LEFT (ticksLeft - ticksLeftOld)
#define DELTA_TICKS_RIGHT (ticksRight - ticksRightOld)
#define DELTA_DISTANCE(delta_left, delta_right) (0.5f * (float) (delta_left + delta_right) * dist_per_count)
#define DELTA_X(heading, delta_dist) (delta_dist * (float) cos(heading))
#define DELTA_Y(heading, delta_dist) (delta_dist * (float) sin(heading))
#define DELTA_HEADING(delta_left, delta_right) ((float) (delta_right - delta_left) * (dist_per_count / track_width))

static void CalcOdometry()
{
    
    ticksLeftOld = ticksLeft;
    ticksRightOld = ticksRight;
    drive_getTicks(&ticksLeft, &ticksRight);
    drive_getSpeedCalc(&speedLeft, &speedRight);

    OdomState.x_dist += DELTA_X(OdomState.heading, DELTA_DISTANCE(DELTA_TICKS_LEFT, DELTA_TICKS_RIGHT));
    OdomState.y_dist += DELTA_Y(OdomState.heading, DELTA_DISTANCE(DELTA_TICKS_LEFT, DELTA_TICKS_RIGHT));
    OdomState.heading += DELTA_HEADING(DELTA_TICKS_LEFT, DELTA_TICKS_RIGHT);
    
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
    while (1) 
    {
        CalcOdometry();
    }
}

void GetOdomState(ODOM_STATE* state)
{
    memcpy(state, &OdomState, sizeof(OdomState));
}

void OdomStart()
{   
    memset(&OdomState, 0, sizeof(OdomState));
    
    cogstart(&BroadcastOdometry, NULL, odom_stack, sizeof(odom_stack));
}