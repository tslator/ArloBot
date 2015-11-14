#include "arlobotconfig.h"
#include "infrared.h"


#define MCP3208_SENSOR

#if defined (MCP3208_SENSOR)
#include "newmcp3208.h"
#define IR_SENSOR_INIT MCP3208_Init
#define IR_SENSOR_ADD MCP3208_Add
#define IR_SENSOR_PING MCP3208_Ping
#define IR_SENSOR_DISTANCE MCP3208_Distance
#elif defined (OTHER_SENSOR)
#endif

static int8_t sensors[MAX_NUM_IR_SENSORS];
static uint8_t num_used_sensors;

void IR_Init()
{
    uint8_t ii;
    
    IR_SENSOR_INIT();
    
    for (ii = 0; ii < MAX_NUM_IR_SENSORS; ++ii)
    {
        sensors[ii] = -1;
    }
}

void IR_Config(uint8_t num_sensors, uint8_t start_channel, uint8_t in_cm)
{
    uint8_t ii;
    
    num_used_sensors = num_sensors;
    
    for (ii = 0; ii < num_sensors; ++ii)
    {
        sensors[ii] = IR_SENSOR_ADD(start_channel + ii, in_cm);
    }
}

void IR_Ping()
{
    uint8_t ii;
    
    for (ii = 0; ii < num_used_sensors; ++ii)
    {
        IR_SENSOR_PING(sensors[ii]);
    }
}

void IR_Distance(float* distances, uint8_t* num_used)
{
    uint8_t ii;
    
    for (ii = 0; ii < num_used_sensors; ++ii)
    {
        distances[ii] = IR_SENSOR_DISTANCE(sensors[ii]);
    }
    
    *num_used = num_used_sensors;
}
