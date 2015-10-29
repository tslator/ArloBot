/**************************************************************************************************
 The purpose of this module is to handle incoming and outgoing messages
**************************************************************************************************/
#ifndef MESSAGE_H
#define MESSAGE_H

/**************************************************************************************************
 Includes
**************************************************************************************************/
#include "fdserial.h"
#include "arlobotconfig.h"

/**************************************************************************************************
 Macros
**************************************************************************************************/
#define ACTION_MSG_CLASS    (0)
#define STATUS_MSG_CLASS    (1)
#define CONFIG_MSG_CLASS    (2)


/**************************************************************************************************
  The following are defines and type definitions for action messages.  Action
  messages are messages received from ROS on which the Propeller software must
  take action, e.g. move
**************************************************************************************************/

/**************************************************************************************************
 Constants
**************************************************************************************************/
#define ACTION_MSG_TYPE_UNKNOWN (0)
#define ACTION_MSG_TYPE_MOVE    (1)  // ROS -> Propeller, Format: a:m:<linear velocity>,<angular velocity>

/**************************************************************************************************
 Types
**************************************************************************************************/
typedef struct _Move_Message
{
    float linear_velocity;
    float angular_velocity;
} MOVE_MSG;

typedef union _Action_Data
{
    MOVE_MSG move;
} ACTION_DATA;

typedef struct _Action_Message
{
    uint8_t type;
    ACTION_DATA data;
} ACTION_MSG;

/**************************************************************************************************
  The following defines and type definitions for Config messages.  Config messages
  are messages sent from ROS to Propeller to configure various settings, e.g.
  drive geometry and operational state.
**************************************************************************************************/

/**************************************************************************************************
 Constants
**************************************************************************************************/
#define CONFIG_MSG_TYPE_UNKNOWN        (0)
#define CONFIG_MSG_TYPE_DRIVE_GEOMETRY (1) // ROS -> Propeller, Format: c:d:<track width>,<distance per count>
#define CONFIG_MSG_TYPE_OP_STATE       (2) // ROS -> Propeller, Format: c:o:<data>

/**************************************************************************************************
 Types
**************************************************************************************************/
typedef struct _Drive_Geometry_Data
{
    float track_width;
    float dist_per_count;
} DRIVE_GEOMETRY_DATA;

typedef struct _Config_Operational_State_Data
{
    uint8_t ignore_proximity;
    uint8_t ignore_cliff_sensors;
    uint8_t ignore_ir_sensors;
    uint8_t ignore_floor_sensors;
    uint8_t ac_power;
    uint8_t last_x;
    uint8_t last_y;
    uint8_t last_heading;
} CONFIG_OP_STATE_DATA;

typedef union _Config_Data
{
    DRIVE_GEOMETRY_DATA drive_geo;
    CONFIG_OP_STATE_DATA op_state;
} CONFIG_DATA;

typedef struct _Config_Message
{
    uint8_t type;
    CONFIG_DATA data;
} CONFIG_MSG;


/**************************************************************************************************
  The following defines and type definitions for Status messages.  Status messages
  are messages sent from Propeller to ROS to convey various status information, e.g.,
  odometry data, sensor data, operational state.
**************************************************************************************************/

/**************************************************************************************************
  Constants
**************************************************************************************************/
#define STATUS_MSG_TYPE_UNKNOWN   (0)
#define STATUS_MSG_TYPE_ODOM      (1)  // Propeller -> ROS, Format: s:o:<data>
#define STATUS_MSG_TYPE_OP_STATE  (2)  // Propeller -> ROS, Format: s:p:<data>
#define STATUS_MSG_TYPE_US_SENSOR (3)  // Propeller -> ROS, Format: s:u:<data>
#define STATUS_MSG_TYPE_IR_SENSOR (4)  // Propeller -> ROS, Format: s:i:<data>

/**************************************************************************************************
  Types
**************************************************************************************************/
typedef struct _Odom_Data
{
    float x_dist;
    float y_dist;
    float heading;
    /* Chris sent the gyro heading as part of the Odometry message
       He also sent the IR and Ping sensor data as well.  I'm more of the mind
       to separate that information into different messages.  But, since I dont
       have a gyro, I'll leave a placeholder here for now.
     */
#ifdef GYRO_SUPPORT
    float gyro_heading;
#endif
    float linear_speed;
    float angular_speed;
} ODOM_DATA;

typedef struct _Status_Operational_State_Data
{
    uint8_t drive_geometry_received;
    uint8_t op_state_received;
    uint8_t motion_detected;
    uint8_t safe_to_proceed;
    uint8_t safe_to_recede;
    uint8_t escaping;
    uint8_t max_forward_speed;
    uint8_t max_reverse_speed;
    uint8_t min_distance_sensor;
    float   left_motor_power;
    float   right_motor_power;
    uint8_t cliff_detected;
    uint8_t floor_obstacle_detected;
} STATUS_OP_STATE_DATA;

typedef struct _Infrared_Sensor_Data
{
    uint8_t sensors[MAX_NUM_IR_SENSORS];
} IR_SENSOR_DATA;

typedef struct _Ultrasonic_Sensor_Data
{
    uint8_t sensors[MAX_NUM_US_SENSORS];
} US_SENSOR_DATA;

typedef union _Status_Data
{
    ODOM_DATA odom;
    STATUS_OP_STATE_DATA op_state;
    IR_SENSOR_DATA ir;
    US_SENSOR_DATA us;
} STATUS_DATA;

typedef struct _Status_Message
{
    uint8_t type;
    STATUS_DATA data;
} STATUS_MSG;


/**************************************************************************************************
 * The following defines and type definitions for Debug messages.  Debug messages
 * are messages sent from Propeller to ROS to convey debug information.
**************************************************************************************************/

/**************************************************************************************************
  Constants
**************************************************************************************************/
#define DEBUG_MSG_TYPE_UNKNOWN  (0)
#define DEBUG_MSG_TYPE_INFO     (1)

/**************************************************************************************************
  Functions
**************************************************************************************************/

/* Initializes the module */
void MSG_Init();

/* Configures the module */
void MSG_Config();

void MSG_Start();

/* Checks for available messages */
int MSG_Available();

/* Parses the various messages
   Note: Data buffer is managed internally
 */
void MSG_ParseActionMessage(ACTION_MSG* msg);
void MSG_ParseConfigMessage(CONFIG_MSG* msg);

/* Sends status and debug messages
   Note: Data buffer is managed internally
 */
int MSG_SendStatusMessage(STATUS_MSG* msg);
int MSG_SendDebugMessage(uint8_t level, char* msg);

#endif
/*************************************************************************************************
EOF
*/
