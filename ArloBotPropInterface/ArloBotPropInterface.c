/**************************************************************************************************
 This is the main module for the Propeller firmware
**************************************************************************************************/


/**************************************************************************************************
 Includes
**************************************************************************************************/
#include "simpletools.h"
#include "fdserial.h"
/*
http://forums.parallax.com/showthread.php/154274-The-quot-Artist-quot-robot?p=1277133&viewfull=1#post1277133
"The most impressive is that we can use the same code as the ActivityBot examples, replacing only the library’s name. So everyone who has a big-size Robot, similar to The Artist (like Arlo, Eddie or any other) must only change the library “abdrive.h” with the “arlodrive.h”. So we can take all the advantages form the pre-written code for the ActivityBot!'
http://www.parallax.com/news/2013-12-23/run-activitybot-c-code-arlo
http://forums.parallax.com/showthread.php/152636-Run-ActivityBot-C-Code-in-the-Arlo!?p=1230592&posted=1#post1230592
*/
#include "arlodrive.h"
#include "message.h"
#include "odom.h"
#include "motiondetect.h"
#include "infrared.h"
#include "ultrasonic.h"


/**************************************************************************************************
 Macros
**************************************************************************************************/
#define DEFAULT_MAX_SPEED (100)
#define DEFAULT_TRACK_WIDTH (0.403000)
#define DEFAULT_DIST_PER_COUNT (0.006760)
#define MAX_NO_SERIAL_COMMS_TIME (1000)  // 1 second
#define INIT_LOOP_WAIT_TIME (1000)       // 1 second

#define OP_STATE_STATUS 0
#define OP_STATE_STATUS_TIME (1000)
#define IR_SENSOR_STATUS 1
#define IR_SENSOR_STATUS_TIME (1000)
#define US_SENSOR_STATUS 2
#define US_SENSOR_STATUS_TIME (1000)

/**************************************************************************************************
 Types
**************************************************************************************************/
typedef struct _Operational_State
{
    // These variables track startup and running states
    uint8_t motion_detected;
    uint8_t drive_geometry_received;
    uint8_t op_state_received;
    uint32_t serial_timeout;
    uint8_t moving;

    // These variables hold op state configuration received from ROS
    uint8_t max_forward_speed;
    uint8_t max_reverse_speed;
    uint8_t last_x;
    uint8_t last_y;
    uint8_t last_heading;
    
    // These variables convey op state to ROS
    uint8_t safe_to_proceed;
    uint8_t safe_to_recede;
    uint8_t escaping;
    uint8_t min_distance_sensor;
    uint8_t left_motor_power;
    uint8_t right_motor_power;
    uint8_t cliff_detected;
    uint8_t floor_obstacle_detected;
    uint8_t ignore_proximity;
    uint8_t ignore_cliff_sensors;
    uint8_t ignore_ir_sensors;
    uint8_t ignore_floor_sensors;
    uint8_t ac_power;
} OP_STATE;

/* Consider what type of data structure could be used to hold IR/US sensor groupings */

/**************************************************************************************************
 Variables
**************************************************************************************************/
// Shared variables - These variables are shared with the Odometry cog
float volatile DistancePerCount;
float volatile TrackWidth;

static float CommandedVelocity;
static float CommandedAngularVelocity;
static OP_STATE OpState;


/**************************************************************************************************
 Functions
**************************************************************************************************/
/**************************************************************************************************
 Returns the number of millis since the propeller started up
 Note: This function needs to be moved to a separate module like utilities or time etc.
 Note: Also, I don't think function handles rollover of CNT, but I think that is more than 24 hours
 and it is unlikely that the robot will be running for 24 hours straight
**************************************************************************************************/
uint32_t millis()
{
  return (CNT / (CLKFREQ / 1000));
}

/**************************************************************************************************
 Wrapper for pause primitive
 Note: This function needs to be moved to a separate module like utilities or time etc.
**************************************************************************************************/
static void Wait(uint32_t timeout)
{
    pause(timeout);
}

/**************************************************************************************************
  Invokes the parsing routine on incoming ACTION messages
**************************************************************************************************/
void ParseActionMessages()
{
    ACTION_MSG a_msg;
    
    MSG_ParseActionMessage(&a_msg);
    switch (a_msg.type)
    {
        case ACTION_MSG_TYPE_MOVE:
        {
            CommandedVelocity = a_msg.data.move.linear_velocity;
            CommandedAngularVelocity = a_msg.data.move.angular_velocity;
            
            // Note: Move commands come via serial communications.  It is necessary to track how long
            //       between serial commands in order to keep the robot from running unchecked
            //       Every time a serial command is received, the timeout is reset
            OpState.serial_timeout = millis();
            
            break;
        }
        
        default:
            // Unknown message type
            //assert();
            break;
    }
}

/**************************************************************************************************
  Invokes the parsing routine on incoming CONFIG messages
**************************************************************************************************/
static void ParseConfigMessages()
{
    CONFIG_MSG c_msg;
    
    MSG_ParseConfigMessage(&c_msg);
    switch (c_msg.type)
    {
        case CONFIG_MSG_TYPE_DRIVE_GEOMETRY:
        {
            // What other validation is neccessary on these values?
            // One thing that has occurred, when the values for track width and distance per count are 0.0
            // the resulting odometry calculations end up being reported as NANs.
            // At the very least, these values need to be greater than 0.0 (to within a certain tolerance)
            // These values do have a default and maybe the default should remain in effect unless the 
            // overrides are "legitimate".  Food for thought.
            if (c_msg.data.drive_geo.track_width > 0.0)
            {
                TrackWidth = c_msg.data.drive_geo.track_width;
            }
            if (c_msg.data.drive_geo.dist_per_count > 0.0)
            {
                DistancePerCount = c_msg.data.drive_geo.dist_per_count;
            }
            OpState.drive_geometry_received = 1;
            break;
        }
        
        case CONFIG_MSG_TYPE_OP_STATE:
        {
            // Note: Consider combining the binary parameters into a bitmap
            
            OpState.ignore_proximity = c_msg.data.op_state.ignore_proximity;
            OpState.ignore_cliff_sensors = c_msg.data.op_state.ignore_cliff_sensors;
            OpState.ignore_ir_sensors = c_msg.data.op_state.ignore_ir_sensors;
            OpState.ignore_floor_sensors = c_msg.data.op_state.ignore_floor_sensors;
            OpState.ac_power = c_msg.data.op_state.ac_power;
            OpState.last_x = c_msg.data.op_state.last_x;
            OpState.last_y = c_msg.data.op_state.last_y;
            OpState.last_heading = c_msg.data.op_state.last_heading;
            OpState.op_state_received = 1;
            break;
        }
        
        default:
            // Unknown message type
            //assert()
            break;
    }
}

/**************************************************************************************************
  Tracks time for each type of STATUS message and return True when the message should be and False
  otherwise
**************************************************************************************************/
static uint8_t TimeForStatusMessage(uint8_t status_type)
{
    static uint32_t op_state_last_time = 0;
    static uint32_t ir_sensor_last_time = 0;
    static uint32_t us_sensor_last_time = 0;
    
    switch (status_type)
    {
        case OP_STATE_STATUS:
            if ((millis() - op_state_last_time) > OP_STATE_STATUS_TIME)
            {
                op_state_last_time = millis();
                return 1;
            }                
            break;
            
        case IR_SENSOR_STATUS:
            if ((millis() - ir_sensor_last_time) > IR_SENSOR_STATUS_TIME)
            {
                ir_sensor_last_time = millis();
                return 1;
            }                
            break;
            
        case US_SENSOR_STATUS:
            if ((millis() - us_sensor_last_time) > US_SENSOR_STATUS_TIME)
            {
                us_sensor_last_time = millis();
                return 1;
            }                
            break;
            
        default:
            return 0;
    }
    
    return 0;
}

/**************************************************************************************************
  Processes incoming messages when they are available
**************************************************************************************************/
static void ProcessIncomingMessages()
{
    if (MSG_Available())
    {
        ParseActionMessages();
        ParseConfigMessages();
    }
}

/**************************************************************************************************
  Sends STATUS messages at the appropriately scheduled time
**************************************************************************************************/
static void SendStatusMessages()
{
    STATUS_MSG msg;
    
    if (TimeForStatusMessage(OP_STATE_STATUS))
    {
        memset(&msg, 0, sizeof(msg));
        
        msg.type = STATUS_MSG_TYPE_OP_STATE;
        msg.data.op_state.drive_geometry_received = OpState.drive_geometry_received;
        msg.data.op_state.op_state_received       = OpState.op_state_received;
        msg.data.op_state.motion_detected         = OpState.motion_detected;
        msg.data.op_state.safe_to_proceed         = OpState.safe_to_proceed;
        msg.data.op_state.safe_to_recede          = OpState.safe_to_recede;
        msg.data.op_state.escaping                = OpState.escaping;
        msg.data.op_state.max_forward_speed       = OpState.max_forward_speed;
        msg.data.op_state.max_reverse_speed       = OpState.max_reverse_speed;
        msg.data.op_state.min_distance_sensor     = OpState.min_distance_sensor;
        msg.data.op_state.left_motor_power        = OpState.left_motor_power;
        msg.data.op_state.right_motor_power       = OpState.right_motor_power;
        msg.data.op_state.cliff_detected          = OpState.cliff_detected;
        msg.data.op_state.floor_obstacle_detected = OpState.floor_obstacle_detected;
        MSG_SendStatusMessage(&msg);
    }
    
#ifdef IR_SENSORS_SUPPORT
    if (TimeForStatusMessage(IR_SENSOR_STATUS))
    {
        float distances[MAX_NUM_IR_SENSORS];
        uint8_t num_used;
        uint8_t ii;
        
        memset(&distances, 0, sizeof(distances));
        memset(&msg, 0, sizeof(msg));
        
        msg.type = STATUS_MSG_TYPE_IR_SENSOR;
        IR_Distance(distances, &num_used);
        for (ii = 0; ii < num_used; ++ii)
        {
            msg.data.ir.sensors[ii] = distances[ii];
        }
    }
#endif

#ifdef US_SENSORS_SUPPORT
    if (TimeForStatusMessage(US_SENSOR_STATUS))
    {
        float distances[MAX_NUM_US_SENSORS];
        uint8_t num_used;
        uint8_t ii;
        
        memset(&distances, 0, sizeof(distances));
        memset(&msg, 0, sizeof(msg));
        
        msg.type = STATUS_MSG_TYPE_US_SENSOR;
        US_Distance(distances, &num_used);
        for (ii = 0; ii < num_used; ++ii)
        {
            msg.data.us.sensors[ii] = distances[ii];
        }
    }
#endif
}

/**************************************************************************************************
  Checks various environment information, sensors, states, etc to determine if continued operation
  is possible
**************************************************************************************************/
static void CheckEnvironment()
{
    
    OpState.safe_to_proceed = 1;
    OpState.safe_to_recede = 1;
        
    // Don't let the robot move unchecked.  At the minimum the robot can only move for MAX_NO_SERIAL_COMMS_TIME seconds
    // If the robot is moving (non-zero speed on left or right wheels) and there has not been serial communication for 
    // MAX_NO_SERIAL_COMMS_TIME then zero out the velocities
    if ( OpState.moving && 
         ((millis() - OpState.serial_timeout) > MAX_NO_SERIAL_COMMS_TIME) )
    {
        CommandedVelocity = 0.0;
        CommandedAngularVelocity = 0.0;
    }
    
    // Ultimately we will monitor the motor voltages and report them, for now, just fake it so ROS is happy
    OpState.left_motor_power = 4.69;
    OpState.right_motor_power = 4.69;
    
}

/**************************************************************************************************
  Calculate the speeds for the left and right wheels based on the given linear and angular velocities
  Ref: 
**************************************************************************************************/
static void CalculateLeftRightSpeed(float* left, float* right)
// Prevent saturation at max wheel speed when a compound command is sent.
/* Without this, if your max speed is 50, and ROS asks us to set one wheel
   at 50 and the other at 100, we will end up with both at 50
   changing a turn into a straight line!
   This is because arlodrive.c just cuts off each wheel at the set max speed
   with no regard for the expected left to right wheel speed ratio.
   Here is the code from arlodrive.c:
   int abd_speedLimit = 100;
   static int encoderFeedback = 1;

   void drive_setMaxSpeed(int maxTicksPerSec) {
   abd_speedLimit = maxTicksPerSec;
   }
   ...
   void set_drive_speed(int left, int right) {
   if(encoderFeedback) {
   if(left > abd_speedLimit) left = abd_speedLimit;
   if(left < -abd_speedLimit) left = -abd_speedLimit;
   if(right > abd_speedLimit) right = abd_speedLimit;
   if(right < -abd_speedLimit) right = -abd_speedLimit;
   }}
   ...

   So clearly we need to "normalize" the speed so that if one number is truncated,
   the other is brought down the same amount, in order to accomplish the same turn
   ratio at a slower speed!
   Especially if the max speed is variable based on parameters within this code,
   such as proximity to walls, etc.
   */
{
    float angular_velocity_offset;
    
    angular_velocity_offset = CommandedAngularVelocity * (TrackWidth * 0.5);
    
    if (CommandedVelocity > 0) 
    {
        // Use max forward speed for rotate in place.
        if ( (OpState.max_forward_speed * DistancePerCount) - fabs(angular_velocity_offset) < CommandedVelocity)
        {
            CommandedVelocity = (OpState.max_forward_speed * DistancePerCount) - fabs(angular_velocity_offset);
        }        
    } 
    else if (CommandedVelocity < 0)
    { 
        // Use max reverse speed for reverse movement.

        // In theory ROS never requests a negative angular velocity, only teleop
        if ( -((OpState.max_reverse_speed * DistancePerCount) - fabs(angular_velocity_offset)) > CommandedVelocity)
        {
            CommandedVelocity = -((OpState.max_reverse_speed * DistancePerCount) - fabs(angular_velocity_offset));
        }            
    }
    
    *left = (CommandedVelocity - angular_velocity_offset)/DistancePerCount;
    *right = (CommandedVelocity + angular_velocity_offset)/DistancePerCount;
    
}

/**************************************************************************************************
  Applies the left and right wheel speeds to the motors.  The wrapper allows for any additional
  safety checks or overrides.  Also, provides calculation for the 'moving' state variable.  
**************************************************************************************************/
static void ApplyLeftRightSpeed(float left_speed, float right_speed)
{
    // Use the left/right speed to determine if the robot is moving
    OpState.moving = 0;
    if (left_speed > 0 || right_speed > 0 || left_speed < 0 || right_speed < 0)
    {
        OpState.moving = 1;
    }
          
    drive_speed(left_speed, right_speed);
}

/**************************************************************************************************
  Initializes internal module variables and also initializes external modules via their Init entry
  points.
**************************************************************************************************/
static void Init()
{
    TrackWidth = DEFAULT_TRACK_WIDTH;
    DistancePerCount = DEFAULT_DIST_PER_COUNT;
    
    CommandedVelocity = 0.0;
    CommandedAngularVelocity = 0.0;
    
    memset(&OpState, 0, sizeof(OpState));
    OpState.max_forward_speed = DEFAULT_MAX_SPEED;
    OpState.max_reverse_speed = DEFAULT_MAX_SPEED;
    
    OpState.left_motor_power = 4.69;
    OpState.right_motor_power = 4.69;
    
    MSG_Init();
    ODOM_Init();
}

/**************************************************************************************************
  Configures internal module variables and components.
**************************************************************************************************/
static void Config()
{
    // Initialize the drive speed, set the maximum speed, and set ramp step
    drive_speed(0, 0);
    drive_setMaxSpeed(DEFAULT_MAX_SPEED);    
    // Note: Chris has a question as to whether the ramping needed to be adjusted.  This needs to be addressed.
    drive_setRampStep(3);
    
    MSG_Config();
    ODOM_Config();
}

/**************************************************************************************************
  Wrapper around motion detection.  The only application at this point is to use a PIR sensor and 
  let the robot monitor motion to decide whether it should turn on.  I'm keeping this here for
  now to preserve Chris' original intent, but there is no actual implementation -- the MD module
  is empty
**************************************************************************************************/
static uint8_t MotionDetected()
{
    OpState.motion_detected = MD_Detected();
    return OpState.motion_detected;
}

/**************************************************************************************************
  Provide control over configuration of the robot.  On start up, the Propeller firmware requires
  two pieces of information: drive geometry and operational state.  When both have been received
  the Propeller firmware can proceed to the main loop and do robot stuff
**************************************************************************************************/
static uint8_t OpStateReceived()
{
    if (OpState.drive_geometry_received && 
        OpState.op_state_received)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**************************************************************************************************
  Kicks of the Odometry processing and enters the main control loop
**************************************************************************************************/
static void ControlLoop()
{
    float left_speed;
    float right_speed;
    
    ODOM_Start();

    while (1)
    {
        ProcessIncomingMessages();
        CheckEnvironment();
        CalculateLeftRightSpeed(&left_speed, &right_speed);
        ApplyLeftRightSpeed(left_speed, right_speed);
        SendStatusMessages();
    }
}

/**************************************************************************************************
  Performs initialization and configuration, enters the Operational State detection loop, and 
  subsequently continues on to the control loop
**************************************************************************************************/
int main()
{
    Init();
    Config();

    MSG_Start();
    
    // If motion is detected then notify ROS to transmit the Operational State and startup the Robot
    // If the Operational State is transmitted independent of motion, then startup the Robot
    while (!MotionDetected() && !OpStateReceived())
    {
        ProcessIncomingMessages();
        SendStatusMessages();
        Wait(INIT_LOOP_WAIT_TIME);
    };
    
    ControlLoop();
}
/*************************************************************************************************
EOF
*/
