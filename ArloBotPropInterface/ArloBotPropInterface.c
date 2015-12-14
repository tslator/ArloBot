#include <simpletools.h>
#include <fdserial.h>
#include "arlodrive.h"
#include "i2cbus.h"
#include "utils.h"
#include "message.h"
#include "sensors.h"
#include "odom.h"
#include "safety.h"



#define DEFAULT_MAX_SPEED (100)
#define DEFAULT_TRACK_WIDTH (0.403000)
#define DEFAULT_DIST_PER_COUNT (0.006760)
#define MAX_NO_SERIAL_COMMS_TIME (1000)
#define INIT_LOOP_WAIT_TIME (1000)

#define OP_STATE_STATUS_TIME (1000)
#define ODOM_STATE_STATUS_TIME (1000)
#define AIR_SENSOR_STATUS_TIME (1000)
#define DIR_SENSOR_STATUS_TIME (1000)
#define US_SENSOR_STATUS_TIME (100)

//-------------------------------------------------------------------------------------------------
//  Messaging
//-------------------------------------------------------------------------------------------------
const char COMMA[2] = ",";

// Message Classes
#define ACTION_MSG_CLASS ('a')
#define STATUS_MSG_CLASS ('s')
#define CONFIG_MSG_CLASS ('c')

// Message Types
#define ACTION_MSG_TYPE_MOVE ('m')
#define STATUS_MSG_TYPE_ODOM ('o')
#define STATUS_MSG_TYPE_OP_STATE ('p')
#define STATUS_MSG_TYPE_AIR_SENSOR ('i') // Analog IR Sensor
#define STATUS_MSG_TYPE_DIR_SENSOR ('g') // Digital IR Sensor
#define STATUS_MSG_TYPE_US_SENSOR ('u')
#define CONFIG_MSG_TYPE_DRIVE_GEOMETRY ('d')
#define CONFIG_MSG_TYPE_OP_STATE ('p')

#define USVALFMT "%.1f"
#define US_SENSOR_FORMAT "%c:%c:"USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT","USVALFMT"\n"

#define DIR_SENSOR_FORMAT   "%c:%c:%d,%d,%d,%d,%d,%d\n"

#define OPSVALFMT "%.1f"
#define OP_STATE_FORMAT "%c:%c:%d,%d,%d,%d,%d,%d,%d,%d,%d,"OPSVALFMT","OPSVALFMT",%d,%d\n"

#define AIRVALFMT "%.1f"
#define AIR_SENSOR_FORMAT "%c:%c:"AIRVALFMT","AIRVALFMT","AIRVALFMT","AIRVALFMT","AIRVALFMT","AIRVALFMT","AIRVALFMT","AIRVALFMT"\n"

#define ODOMVALFORMAT "%.1f"
#define ODOM_FORMAT "%c:%c:"ODOMVALFORMAT","ODOMVALFORMAT","ODOMVALFORMAT","ODOMVALFORMAT","ODOMVALFORMAT","ODOMVALFORMAT"\n"

static char out_buffer[MAX_BUFFER_DATA];
static char in_buffer[MAX_BUFFER_DATA];

//-------------------------------------------------------------------------------------------------
//  Odometry
//-------------------------------------------------------------------------------------------------
static ODOM_STATE OdomState;

extern uint8_t odom_enabled;
#define ENABLE_ODOM()     (odom_enabled = 1)
#define IS_ODOM_ENABLED() (odom_enabled == 1)

//-------------------------------------------------------------------------------------------------
//  Sensors
//-------------------------------------------------------------------------------------------------
static IMU_STATE ImuState;
static SENSOR_STATE SensorState;
static SAFETY_STATE SafetyState;

static uint8_t sensors_enabled;
#define ENABLE_SENSORS()  (sensors_enabled = 1)
#define IS_SENSORS_ENABLED() (sensors_enabled == 1)



//-------------------------------------------------------------------------------------------------
//  Application
//-------------------------------------------------------------------------------------------------
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
    float left_motor_power;
    float right_motor_power;
    uint8_t ac_power;
} OP_STATE;

// Shared variables - These variables are shared with the Odometry cog
float volatile dist_per_count = DEFAULT_TRACK_WIDTH;
float volatile track_width = DEFAULT_DIST_PER_COUNT;

static float commanded_linear_velocity;
static float commanded_angular_velocity;
static float left_speed;
static float right_speed;
static OP_STATE OpState;

//-------------------------------------------------------------------------------------------------
//  Messaging
//-------------------------------------------------------------------------------------------------
static void SendStatusMessage(char type)
{
    uint8_t send_message = 0;
    
    switch (type)
    {
        case STATUS_MSG_TYPE_ODOM:
            sprint(out_buffer, ODOM_FORMAT,
                   STATUS_MSG_CLASS,
                   STATUS_MSG_TYPE_ODOM,
                   OdomState.x_dist, 
                   OdomState.y_dist, 
                   OdomState.heading, 
                   ImuState.heading, 
                   OdomState.linear_speed, 
                   OdomState.angular_speed);
                   
            send_message = 1;
            break;
           
        case STATUS_MSG_TYPE_AIR_SENSOR:
            // Send sensor information
            // It maybe useful to send both raw sensor data, e.g. all the ultrasonic sensors, all the ir sensors, etc
            // and also send the grouping of sensors, e.g. FrontCollisionDetect (which might be a combination of ultrasonic and ir sesnors), 
            // RearCollisionDetect, etc.
            sprint(out_buffer, AIR_SENSOR_FORMAT,
                   STATUS_MSG_CLASS,
                   STATUS_MSG_TYPE_AIR_SENSOR,
                   SensorState.analog_ir[0],
                   SensorState.analog_ir[1],
                   SensorState.analog_ir[2],
                   SensorState.analog_ir[3],
                   SensorState.analog_ir[4],
                   SensorState.analog_ir[5],
                   SensorState.analog_ir[6],
                   SensorState.analog_ir[7]);
            send_message = 1;
            break;
            
        case STATUS_MSG_TYPE_DIR_SENSOR:
            // Send sensor information
            // It maybe useful to send both raw sensor data, e.g. all the ultrasonic sensors, all the ir sensors, etc
            // and also send the grouping of sensors, e.g. FrontCollisionDetect (which might be a combination of ultrasonic and ir sesnors), 
            // RearCollisionDetect, etc.
            sprint(out_buffer, DIR_SENSOR_FORMAT,
                   STATUS_MSG_CLASS,
                   STATUS_MSG_TYPE_DIR_SENSOR,
                   SensorState.digital_ir[0],
                   SensorState.digital_ir[1],
                   SensorState.digital_ir[2],
                   SensorState.digital_ir[3],
                   SensorState.digital_ir[4],
                   SensorState.digital_ir[5]);
            send_message = 1;
            break;
            
        case STATUS_MSG_TYPE_US_SENSOR:
            // Send sensor information
            // It maybe useful to send both raw sensor data, e.g. all the ultrasonic sensors, all the ir sensors, etc
            // and also send the grouping of sensors, e.g. FrontCollisionDetect (which might be a combination of ultrasonic and ir sesnors), 
            // RearCollisionDetect, etc.
            sprint(out_buffer, US_SENSOR_FORMAT,
                   STATUS_MSG_CLASS,
                   STATUS_MSG_TYPE_US_SENSOR,
                   SensorState.ultrasonic[0],
                   SensorState.ultrasonic[1],
                   SensorState.ultrasonic[2],
                   SensorState.ultrasonic[3],
                   SensorState.ultrasonic[4],
                   SensorState.ultrasonic[5],
                   SensorState.ultrasonic[6],
                   SensorState.ultrasonic[7],
                   SensorState.ultrasonic[8],
                   SensorState.ultrasonic[9],
                   SensorState.ultrasonic[10],
                   SensorState.ultrasonic[11],
                   SensorState.ultrasonic[12],
                   SensorState.ultrasonic[13],
                   SensorState.ultrasonic[14],
                   SensorState.ultrasonic[15]
                   );
            send_message = 1;
            break;
          
        case  STATUS_MSG_TYPE_OP_STATE:
            // Consider using a bitmap for the binary detectors
            sprint(out_buffer, OP_STATE_FORMAT,
                   STATUS_MSG_CLASS,
                   CONFIG_MSG_TYPE_OP_STATE,
                   OpState.drive_geometry_received,
                   OpState.op_state_received,
                   SensorState.motion_detected,
                   SafetyState.safe_to_proceed,
                   SafetyState.safe_to_recede,
                   SafetyState.escaping,
                   OpState.max_forward_speed,
                   OpState.max_reverse_speed,
                   SafetyState.min_distance_sensor,
                   OpState.left_motor_power,
                   OpState.right_motor_power,
                   SafetyState.cliff_detected,
                   SafetyState.floor_obstacle_detected);
            send_message = 1;
            break;
        
        default:
            // Unknown message type
            break;
    }
    
    if (send_message)
    {
        SendMessage(out_buffer);
        //print(out_buffer);
    }
}

static uint8_t TimeForStatusMessage(char type)
{
    static uint32_t op_state_last_time   = 0;
    static uint32_t odom_state_last_time = 0;
    static uint32_t air_sensor_last_time = 0;
    static uint32_t dir_sensor_last_time = 0;
    static uint32_t us_sensor_last_time  = 0;
    
    switch (type)
    {
        case STATUS_MSG_TYPE_OP_STATE:
            if ((millis() - op_state_last_time) > OP_STATE_STATUS_TIME)
            {
                op_state_last_time = millis();
                return 1;
            }                
            break;
            
        case STATUS_MSG_TYPE_ODOM:
            if ((millis() - odom_state_last_time) > ODOM_STATE_STATUS_TIME)
            {
                odom_state_last_time = millis();
                return IS_ODOM_ENABLED();
            }
            break;
            
        case STATUS_MSG_TYPE_AIR_SENSOR:
            if ((millis() - air_sensor_last_time) > AIR_SENSOR_STATUS_TIME)
            {
                air_sensor_last_time = millis();
                return IS_SENSORS_ENABLED();
            }                
            break;
            
        case STATUS_MSG_TYPE_DIR_SENSOR:
            if ((millis() - dir_sensor_last_time) > DIR_SENSOR_STATUS_TIME)
            {
                dir_sensor_last_time = millis();
                return IS_SENSORS_ENABLED();
            }                
            break;
            
        case STATUS_MSG_TYPE_US_SENSOR:
            if ((millis() - us_sensor_last_time) > US_SENSOR_STATUS_TIME)
            {
                us_sensor_last_time = millis();
                return IS_SENSORS_ENABLED();
            }                
            break;
            
        default:
            return 0;
    }
    
    return 0;
}


static void SendStatusMessages()
{
    if (TimeForStatusMessage(STATUS_MSG_TYPE_OP_STATE))
    {
        SendStatusMessage(STATUS_MSG_TYPE_OP_STATE);
    }
    
    if (TimeForStatusMessage(STATUS_MSG_TYPE_ODOM))
    {
        SendStatusMessage(STATUS_MSG_TYPE_ODOM);
    }
    
    if (TimeForStatusMessage(STATUS_MSG_TYPE_AIR_SENSOR))
    {
        SendStatusMessage(STATUS_MSG_TYPE_AIR_SENSOR);
    }    
    
    if (TimeForStatusMessage(STATUS_MSG_TYPE_US_SENSOR))
    {
        SendStatusMessage(STATUS_MSG_TYPE_US_SENSOR);
    }
    
    if (TimeForStatusMessage(STATUS_MSG_TYPE_DIR_SENSOR))
    {
        SendStatusMessage(STATUS_MSG_TYPE_DIR_SENSOR);
    }
}

static void ParseMoveMessage(char *buffer)
{
    char *unconverted;
    char *value_token;
    
    /* parse up the data into the values array */
    value_token = strtok(buffer, COMMA);
    commanded_linear_velocity = strtod(value_token, &unconverted);
    value_token = strtok(NULL, COMMA);
    commanded_angular_velocity = strtod(value_token, &unconverted);
}

static void ParseDriveGeometry(char *buffer)
{
    char *unconverted;
    char *value_token;
    float new_track_width;
    float new_dist_per_count;
    
    /* parse up the data into the values array */
    /* c:o:%f,%f */
    value_token = strtok(buffer, COMMA);
    new_track_width = strtod(value_token, &unconverted);
    value_token = strtok(NULL, COMMA);
    new_dist_per_count = strtod(value_token, &unconverted);

    if (new_track_width > 0.0)
    {
        track_width = new_track_width;
    }
    if (new_dist_per_count > 0.0)
    {
        dist_per_count = new_dist_per_count;
    }

    OpState.drive_geometry_received = 1;
}

static void ParseOpState(char *buffer)
{
    char *unconverted;
    char *value_token;
    
    /* parse up the data into the values array */
    /* c:p:%d,%d,%d,%d,%d,%f,%f,%f */
    value_token = strtok(buffer, COMMA);
    SafetyState.ignore_proximity = (uint8_t)strtol(value_token, &unconverted, 10);
    value_token = strtok(NULL, COMMA);
    SafetyState.ignore_cliff_sensors = (uint8_t)strtol(value_token, &unconverted, 10);
    value_token = strtok(NULL, COMMA);
    SafetyState.ignore_dist_sensors = (uint8_t)strtol(value_token, &unconverted, 10);
    value_token = strtok(NULL, COMMA);
    SafetyState.ignore_floor_sensors = (uint8_t)strtol(value_token, &unconverted, 10);
    value_token = strtok(NULL, COMMA);
    OpState.ac_power = (uint8_t)strtol(value_token, &unconverted, 10);
    value_token = strtok(NULL, COMMA);
    OpState.last_x = strtod(value_token, &unconverted);
    value_token = strtok(NULL, COMMA);
    OpState.last_y = strtod(value_token, &unconverted);
    value_token = strtok(NULL, COMMA);
    OpState.last_heading = strtod(value_token, &unconverted);
    
    OpState.op_state_received = 1;
}    

static void ProcessIncomingMessages()
{
    if (AcquireMessage(in_buffer))
    {
        char msg_class = in_buffer[0];
        char msg_type = in_buffer[2];
        
        switch (msg_class)
        {
            case ACTION_MSG_CLASS:
            {
                switch (msg_type)
                {
                    case ACTION_MSG_TYPE_MOVE:
                        ParseMoveMessage(&in_buffer[4]);
                        
                        // Note: Move commands come via serial communications.  It is necessary to track how long
                        //       between serial commands in order to keep the robot from running unchecked
                        //       Every time a serial command is received, the timeout is reset
                        OpState.serial_timeout = millis();
                        break;
                }
            }
            break;
            
            case CONFIG_MSG_CLASS:
            {
                switch (msg_type)
                {
                    case CONFIG_MSG_TYPE_DRIVE_GEOMETRY:
                        ParseDriveGeometry(&in_buffer[4]);
                        break;
                        
                    case CONFIG_MSG_TYPE_OP_STATE:
                        ParseOpState(&in_buffer[4]);
                        break;
                }                    
            }
            break;                               
        }
    }
}


//-------------------------------------------------------------------------------------------------
//  Application
//-------------------------------------------------------------------------------------------------
static uint8_t MotionDetected()
{
    return SensorState.motion_detected;    
}

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

static void Init()
{
    InitI2C();
    
    memset(&OpState, 0, sizeof(OpState));
    memset(&OdomState, 0, sizeof(OdomState));
    memset(&SensorState, 0, sizeof(SensorState));
    memset(&ImuState, 0, sizeof(ImuState));
    memset(&SafetyState, 0, sizeof(SafetyState));
    
    OpState.max_forward_speed = DEFAULT_MAX_SPEED;
    OpState.max_reverse_speed = DEFAULT_MAX_SPEED;
    
    OpState.left_motor_power = 0;
    OpState.right_motor_power = 0;    
}

static void Config()
{
    // Initialize the drive speed, set the maximum speed, and set ramp step
    drive_speed(0, 0);
    drive_setMaxSpeed(DEFAULT_MAX_SPEED);    
    // Note: Chris has a question as to whether the ramping needed to be adjusted.  This needs to be addressed.
    drive_setRampStep(3);
}

static void Start()
{
    MessageStart();
    OdomStart();
    SensorsStart();
}

static float CalcHeading(IMU_STATE* state)
{
    return 0;
}

static void CheckEnvironment()
{
    GetSensorState(&SensorState);
    UpdateSafety(&SensorState, &SafetyState);
    GetImuState(&ImuState);
    GetOdomState(&OdomState);
    
        
    // Note: There may be a time when the Imu heading will be used, i.e., assigned to the gyro_heading field of OdomState
    // That could be done here.  First, get ImuState, then OdomState and then assign.

    // Check the safety state to ensure the robot doesn't run into anything
    if ( (commanded_linear_velocity > 0 && !SafetyState.safe_to_proceed) || (commanded_linear_velocity < 0 && !SafetyState.safe_to_recede))
    {
        commanded_linear_velocity = 0.0;
        commanded_angular_velocity = 0.0;
    }        

    // Don't let the robot move unchecked.  At the maximum the robot can move for MAX_NO_SERIAL_COMMS_TIME seconds
    // If the robot is moving (non-zero speed on left or right wheels) and there has not been serial communication for 
    // MAX_NO_SERIAL_COMMS_TIME then zero out the velocities
    if ( OpState.moving && 
         ((millis() - OpState.serial_timeout) > MAX_NO_SERIAL_COMMS_TIME) )
    {
        commanded_linear_velocity = 0.0;
        commanded_angular_velocity = 0.0;
    }
    
    // Ultimately we will monitor the motor voltages and report them, for now, just fake it so ROS is happy
    OpState.left_motor_power = 4.69;
    OpState.right_motor_power = 4.69;        
}

static void UpdateMotorSpeed()
{
    float angular_velocity_offset;
    
    angular_velocity_offset = commanded_angular_velocity * (track_width * 0.5);
    
    if (commanded_linear_velocity > 0) 
    {
        // Use max forward speed for rotate in place.
        if ( (OpState.max_forward_speed * dist_per_count) - fabs(angular_velocity_offset) < commanded_linear_velocity)
        {
            commanded_linear_velocity = (OpState.max_forward_speed * dist_per_count) - fabs(angular_velocity_offset);
        }        
    } 
    else if (commanded_linear_velocity < 0)
    { 
        // Use max reverse speed for reverse movement.

        // In theory ROS never requests a negative angular velocity, only teleop
        if ( -((OpState.max_reverse_speed * dist_per_count) - fabs(angular_velocity_offset)) > commanded_linear_velocity)
        {
            commanded_linear_velocity = -((OpState.max_reverse_speed * dist_per_count) - fabs(angular_velocity_offset));
        }            
    }
    
    left_speed = (commanded_linear_velocity - angular_velocity_offset)/dist_per_count;
    right_speed = (commanded_linear_velocity + angular_velocity_offset)/dist_per_count;
    
    // Use the left/right speed to determine if the robot is moving
    OpState.moving = 0;
    if (left_speed > 0 || right_speed > 0 || left_speed < 0 || right_speed < 0)
    {
        OpState.moving = 1;
    }
          
    drive_speed(left_speed, right_speed);
}

int main()
{
    Init();
    Config();
    Start();
    /*
    while (1)
    {
        ProcessIncomingMessages();
        CheckEnvironment();
        
        if (!MotionDetected() && !OpStateReceived())
        {
            SendStatusMessages();
            pause(INIT_LOOP_WAIT_TIME);
            continue;
        }
        
        if (!IsOdomEnabled())
        {
            OdomEnable();
        }
        if (!IsSensorsEnabled())
        {
            SensorsEnable();
        }
        
        UpdateMotorSpeed();
        SendStatusMessages();        
    }
    */
        
    while (!MotionDetected() && !OpStateReceived())
    {
        ProcessIncomingMessages();
        CheckEnvironment();
        SendStatusMessages();
        pause(INIT_LOOP_WAIT_TIME);
    };

    ENABLE_ODOM();
    ENABLE_SENSORS();
    
    while (1)
    {
        ProcessIncomingMessages();
        CheckEnvironment();
        UpdateMotorSpeed();
        SendStatusMessages();
    }
    
    
}
