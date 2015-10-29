/**************************************************************************************************
 The purpose of this module is to handle incoming and outgoing messages
**************************************************************************************************/


/**************************************************************************************************
 Includes
**************************************************************************************************/
#include "simpletools.h"
#include "message.h"

/**************************************************************************************************
  Constants
**************************************************************************************************/
const char COLON[2] = ":";
const char COMMA[2] = ",";

/**************************************************************************************************
  Types
**************************************************************************************************/
typedef struct _Message_Buffer
{
    uint8_t is_full;
    char    data[100];
    uint8_t count;
} MESSAGE_BUFFER;


/**************************************************************************************************
  Macros
**************************************************************************************************/
#define PEEK_INCOMING(index)    (incoming_msg.data[index])
#define IS_ACTION_MSG(type)     (type == 'a')
#define IS_CONFIG_MSG(type)     (type == 'c')
#define PEND_ON_BUFFER(buffer)  while(buffer.is_full)   \
                                {                       \
                                    ;                   \
                                }

/**************************************************************************************************
  Variables
**************************************************************************************************/
static volatile MESSAGE_BUFFER incoming_msg;
static volatile MESSAGE_BUFFER outgoing_msg;
static fdserial *msg_serial;
static int fstack[256]; // If things get weird make this number bigger!

/**************************************************************************************************
  Functions
**************************************************************************************************/

/**************************************************************************************************
  Main COG function
  This function runs in a forever loop.  
  It checks for incoming messages, reads the bytes from the serial port, and copies them into the
  incoming message buffer
  It also checks for messages in the outgoing message buffer and writes them to the serial port
**************************************************************************************************/
static void HandleSerial(void *par)
{
    while (1)
    {
        // Check for incoming message, read and store it, signal the message is ready
        if (!incoming_msg.is_full)
        { 
            memset(incoming_msg.data, 0, sizeof(incoming_msg.data));
            if (fdserial_rxReady(msg_serial) != 0)
            {
                int count = 0;
                
                while (count < sizeof(incoming_msg.data)) 
                {
                    incoming_msg.data[count] = readChar(msg_serial);
                    if (incoming_msg.data[count] == '\r' || incoming_msg.data[count] == '\n')
                    {
                        int ii;
#ifdef LOW_LEVEL_DEBUG // Use this only for debugging problems with message handling
                        dprint(msg_serial, "message received (%d): ", count);
                        for (ii = 0; ii < count; ++ii)
                        {
                            dprint(msg_serial, "%c", incoming_msg.data[ii]);
                        }
                        dprint(msg_serial, "\n");
#endif                        
                        incoming_msg.count = count;
                        incoming_msg.is_full = 1;
                        break;
                    }    
                    count++;
                }
            }
        }
        else
        {
            // This means we aren't processing incomming messages fast enough
            // Could implment an overflow counter that could be reported back as status
        }
        
        // Check for outgoing message, send, and clear flag
        if (outgoing_msg.is_full)
        {
            dprint(msg_serial, "%s", outgoing_msg.data);
            outgoing_msg.is_full = 0;
       }
    }        
}
                                
/**************************************************************************************************
  Copies an incoming message into the buffer, if the message buffer is full (a message is ready)
**************************************************************************************************/
static void AcquireMessage(char *message)
{
    if (incoming_msg.is_full)
    {
        memcpy(message, incoming_msg.data, incoming_msg.count);
        incoming_msg.is_full = 0;
    }
    else
    {
        // This is an error.  We would only parse if a message was available
        //assert(incoming_msg.is_full == 0);
    }
}


/**************************************************************************************************
  Returns True when a message is available for reading; otherwise False
**************************************************************************************************/
int MSG_Available()
{
    return incoming_msg.is_full;
}

/**************************************************************************************************
  Parses an incoming message into the ACTION message structure based on the actual message type
**************************************************************************************************/
void MSG_ParseActionMessage(ACTION_MSG* msg)
{
    char message[sizeof(incoming_msg.data)];
    char *token;
    uint8_t is_action_msg = 0;
    uint8_t is_move_msg = 0;
    
    memset(msg, 0, sizeof(0));

    // Note: Check the message class before attempting to parse - only parse ACTION messages
    
    //if (!IS_ACTION_MSG(PEEK_INCOMING(0)))
    if (! (incoming_msg.data[0] == 'a') )
    {
        return;
    }

    AcquireMessage(message);
    token = strtok(message, COLON);
    
    while (token != NULL)
    {
        switch (*token)
        {
            case 'a':
                is_action_msg = 1;
                break;
                
            case 'm':  // this is a move message
                if (is_action_msg)
                {
                    is_move_msg = 1;
                    msg->type = ACTION_MSG_TYPE_MOVE;
                }
                break;
                                
            default:
                /* The message received was not an action message */
                break;
        }
        
        token = strtok(NULL, COLON);        
        
        if (is_move_msg)
        {
            char *unconverted;
            char *value_token;
            
            /* parse up the data into the values array */
            value_token = strtok(token, COMMA);
            msg->data.move.linear_velocity = strtod(value_token, &unconverted);
            value_token = strtok(NULL, COMMA);
            msg->data.move.angular_velocity = strtod(value_token, &unconverted);
            break;
        }
    }
}                

/**************************************************************************************************
  Parses an incoming message into the CONFIG message structure based on the actual message type
**************************************************************************************************/
void MSG_ParseConfigMessage(CONFIG_MSG* msg)
{
    char message[sizeof(incoming_msg.data)];
    char *token;
    uint8_t is_config_msg = 0;
    uint8_t is_drive_geo_msg = 0;
    uint8_t is_op_state_msg = 0;
    
    memset(msg, 0, sizeof(0));
    
    // Note: Check the message class before attempting to parse - only parse CONFIG messages
    
    //if (!IS_CONFIG_MSG(PEEK_INCOMING(0)))
    if (! (incoming_msg.data[0] == 'c') )
    {
        return;
    }
    
    AcquireMessage(message);
    
    token = strtok(message, COLON);
    
    while (token != NULL)
    {
        switch (*token)
        {
            case 'c':  // this is a config message
                is_config_msg = 1;
                break;
                
            case 'd': // this is a drive geometry message
                if (is_config_msg) 
                {
                    is_drive_geo_msg = 1;
                    msg->type = CONFIG_MSG_TYPE_DRIVE_GEOMETRY;
                }
                break;
                
            case 'p': // this is an operational state message
                if (is_config_msg)
                {
                    is_op_state_msg = 1;
                    msg->type = CONFIG_MSG_TYPE_OP_STATE;
                }
                break;
                
            default:
                /* The message received was not an action message */
                return;
        }
        
        token = strtok(NULL, COLON);
        
        if (is_drive_geo_msg)
        {
            char *unconverted;
            char *value_token;
            
            /* parse up the data into the values array */
            /* c:o:%f,%f */
            value_token = strtok(token, COMMA);
            msg->data.drive_geo.track_width = strtod(value_token, &unconverted);
            value_token = strtok(NULL, COMMA);
            msg->data.drive_geo.dist_per_count = strtod(value_token, &unconverted);
            break;
        }        
        
        if (is_op_state_msg)
        {
            char *unconverted;
            char *value_token;
            
            /* parse up the data into the values array */
            /* c:p:%d,%d,%d,%d,%d,%f,%f,%f */
            value_token = strtok(token, COMMA);
            msg->data.op_state.ignore_proximity = (uint8_t)strtol(value_token, &unconverted, 10);
            value_token = strtok(NULL, COMMA);
            msg->data.op_state.ignore_cliff_sensors = (uint8_t)strtol(value_token, &unconverted, 10);
            value_token = strtok(NULL, COMMA);
            msg->data.op_state.ignore_ir_sensors = (uint8_t)strtol(value_token, &unconverted, 10);
            value_token = strtok(NULL, COMMA);
            msg->data.op_state.ignore_floor_sensors = (uint8_t)strtol(value_token, &unconverted, 10);
            value_token = strtok(NULL, COMMA);
            msg->data.op_state.ac_power = (uint8_t)strtol(value_token, &unconverted, 10);
            value_token = strtok(NULL, COMMA);
            msg->data.op_state.last_x = strtod(value_token, &unconverted);
            value_token = strtok(NULL, COMMA);
            msg->data.op_state.last_y = strtod(value_token, &unconverted);
            value_token = strtok(NULL, COMMA);
            msg->data.op_state.last_heading = strtod(value_token, &unconverted);
            break;
        }

    }
}

/**************************************************************************************************
  Copy a STATUS message into the outgoing message buffer
**************************************************************************************************/
int MSG_SendStatusMessage(STATUS_MSG* msg)
{
    char buffer[sizeof(outgoing_msg.data)];
    
    memset(buffer, 0, sizeof(buffer));
    
    switch (msg->type)
    {
        case STATUS_MSG_TYPE_ODOM:
#ifdef GYRO_SUPPORT
            sprint(buffer, "s:o:%.3f,%.3f,%.3f,%.3f,%.3f\n",
#else
            sprint(buffer, "s:o:%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n", 
#endif
                   msg->data.odom.x_dist, 
                   msg->data.odom.y_dist, 
                   msg->data.odom.heading, 
#ifdef GYRO_SUPPORT
                   msg->data.odom.gyro_heading, 
#endif
                   msg->data.odom.linear_speed, 
                   msg->data.odom.angular_speed);
            break;
            
        case STATUS_MSG_TYPE_IR_SENSOR:
            // Send sensor information
            // It maybe useful to send both raw sensor data, e.g. all the ultrasonic sensors, all the ir sensors, etc
            // and also send the grouping of sensors, e.g. FrontCollisionDetect (which might be a combination of ultrasonic and ir sesnors), 
            // RearCollisionDetect, etc.
            break;
            
        case STATUS_MSG_TYPE_US_SENSOR:
            // Send sensor information
            // It maybe useful to send both raw sensor data, e.g. all the ultrasonic sensors, all the ir sensors, etc
            // and also send the grouping of sensors, e.g. FrontCollisionDetect (which might be a combination of ultrasonic and ir sesnors), 
            // RearCollisionDetect, etc.
            break;
            
        case  STATUS_MSG_TYPE_OP_STATE:
            // Consider using a bitmap for the binary detectors
            sprint(buffer, "s:p:%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%d,%d\n",
                   msg->data.op_state.drive_geometry_received,
                   msg->data.op_state.op_state_received,
                   msg->data.op_state.motion_detected,
                   msg->data.op_state.safe_to_proceed,
                   msg->data.op_state.safe_to_recede,
                   msg->data.op_state.escaping,
                   msg->data.op_state.max_forward_speed,
                   msg->data.op_state.max_reverse_speed,
                   msg->data.op_state.min_distance_sensor,
                   msg->data.op_state.left_motor_power,
                   msg->data.op_state.right_motor_power,
                   msg->data.op_state.cliff_detected,
                   msg->data.op_state.floor_obstacle_detected);
            break;
            
        default:
            // Unknown message type
            break;
    }
    
    // Note: Wait for the buffer to go empty before adding a new message
    
    PEND_ON_BUFFER(outgoing_msg);
    memset(outgoing_msg.data, 0, sizeof(outgoing_msg.data));
    memcpy(outgoing_msg.data, buffer, strlen(buffer));
    outgoing_msg.is_full = 1;
    
    return 0;
}

/**************************************************************************************************
  Copy a DEBUG message into the outgoing message buffer
**************************************************************************************************/
int MSG_SendDebugMessage(uint8_t level, char* msg)
{
    char buffer[sizeof(outgoing_msg.data)];
    
    memset(buffer, 0, sizeof(buffer));
    
    switch (level)
    {
        case DEBUG_MSG_TYPE_INFO:
            sprint(buffer, "d:%d:%s\n", level, msg);
            break;
            
        default:
            break;
    }
    
    PEND_ON_BUFFER(outgoing_msg);
    memset(outgoing_msg.data, 0, sizeof(outgoing_msg.data));
    memcpy(outgoing_msg.data, buffer, strlen(buffer));
    outgoing_msg.is_full = 1;
}


/**************************************************************************************************
  Initializes internal module variables
**************************************************************************************************/
void MSG_Init()
{
    memset(&incoming_msg, 0, sizeof(incoming_msg));
    memset(&outgoing_msg, 0, sizeof(outgoing_msg));
}

/**************************************************************************************************
  Configures the serial port
**************************************************************************************************/
void MSG_Config()
{   
    simpleterm_close();
    msg_serial = fdserial_open(31, 30, 0, 115200);
}

/**************************************************************************************************
  Kicks off the HandleSerial function on the next available COG
**************************************************************************************************/
void MSG_Start()
{
    cogstart(&HandleSerial, NULL, fstack, sizeof(fstack));    
}
/*************************************************************************************************
EOF
*/