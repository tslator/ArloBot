/**************************************************************************************************
 The purpose of this module is to calculate and report odometry readings
**************************************************************************************************/

/**************************************************************************************************
  Includes
**************************************************************************************************/
#include "arlodrive.h"
#include "odom.h"
#include "message.h"

/**************************************************************************************************
  Variables
**************************************************************************************************/
extern float DistancePerCount;
extern float TrackWidth;

// For Odometry
static int ticksLeft;
static int ticksRight;
static int ticksLeftOld;
static int ticksRightOld;
static int speedLeft;
static int speedRight;

static float Heading;
static float X;
static float Y;

static int fstack[256];

/**************************************************************************************************
  Functions
**************************************************************************************************/
// Note: We don't need these
void getTicks();
void displayTicks();
void BroadcastOdometry(void *par); // Use a cog to broadcast Odometry to ROS continuously

// This will be moved to a new module, e.g. utilities or time etc
extern uint32_t millis();


/**************************************************************************************************
  Retrieves left and right ticks and calculates left and right wheel speed
**************************************************************************************************/
// Note: Should be able to make this static
void getTicks(void) 
{
    ticksLeftOld = ticksLeft;
    ticksRightOld = ticksRight;
    drive_getTicks(&ticksLeft, &ticksRight);
    drive_getSpeedCalc(&speedLeft, &speedRight);
}

/**************************************************************************************************
  Calculates the odometry values for the Odometry message and sends the message
**************************************************************************************************/
void displayTicks(void) 
{
    int deltaTicksLeft      = ticksLeft - ticksLeftOld;
    int deltaTicksRight     = ticksRight - ticksRightOld;
    double deltaDistance    = 0.5f * (double) (deltaTicksLeft + deltaTicksRight) * DistancePerCount;
    double deltaX           = deltaDistance * (double) cos(Heading);
    double deltaY           = deltaDistance * (double) sin(Heading);
    // Note: Why are we calculating RadiansPerCount everytime when the value can only change if DistancePerCount or TrackWidth change
    // which can only change if there is a change in the default values (at compile time) or there is a configuration change
    double RadiansPerCount  = DistancePerCount / TrackWidth;
    double deltaHeading     = (double) (deltaTicksRight - deltaTicksLeft) * RadiansPerCount;

    X += deltaX;
    Y += deltaY;
    Heading += deltaHeading;
    
    // limit heading to -Pi <= heading < Pi
    if (Heading > PI) 
    {
        Heading -= 2.0 * PI;
    } 
    else 
    {
        if (Heading <= -PI) 
        {
            Heading += 2.0 * PI;
        }
    }
    
    // http://webdelcire.com/wordpress/archives/527
    double V     = ((speedRight * DistancePerCount) + (speedLeft * DistancePerCount)) / 2;
    double Omega = ((speedRight * DistancePerCount) - (speedLeft * DistancePerCount)) / TrackWidth;

    // Odometry for ROS
       
    STATUS_MSG msg;
    memset(&msg, 0, sizeof(msg));
    msg.type = STATUS_MSG_TYPE_ODOM;
    msg.data.odom.x_dist = X;
    msg.data.odom.y_dist = Y;
    msg.data.odom.heading = Heading;
#ifdef GYRO_SUPPORT
    msg.data.odom.gyro_heading = GYRO_GetHeading();
#endif
    msg.data.odom.linear_speed = V;
    msg.data.odom.angular_speed = Omega;
    MSG_SendStatusMessage(&msg);
}

/* Some of the code below came from Dr. Rainer Hessmer's robot.pde
   The rest was heavily inspired/copied from here:
http://forums.parallax.com/showthread.php/154963-measuring-speed-of-the-ActivityBot?p=1260800&viewfull=1#post1260800
*/

/**************************************************************************************************
  Main COG function
  This function runs in a forever loop
  Every 100ms it calculates new Odometry values and sends a message
**************************************************************************************************/
void BroadcastOdometry(void *par)
{
    static uint32_t last_time = 0;

    while (1) 
    {
        if ((millis() - last_time) > 100)
        {
            last_time = millis();
            getTicks();
            displayTicks();
        }
    }
}

/**************************************************************************************************
  Initializes internal module variables
**************************************************************************************************/
void ODOM_Init()
{
    ticksLeft       = 0;
    ticksRight      = 0;
    ticksLeftOld    = 0;
    ticksRightOld   = 0;
    speedLeft       = 0;
    speedRight      = 0;

    Heading = 0.0;
    X       = 0.0;
    Y       = 0.0;
}

/**************************************************************************************************
  Configures internal module components
**************************************************************************************************/
void ODOM_Config()
{
}

/**************************************************************************************************
  Kicks off the COG function
**************************************************************************************************/
void ODOM_Start()
{
    cogstart(&BroadcastOdometry, NULL, fstack, sizeof fstack);
}
/*************************************************************************************************
EOF
*/
