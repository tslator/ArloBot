/**************************************************************************************************
 The purpose of this module is to provide access to Ultrasonic sensor data
**************************************************************************************************/


/**************************************************************************************************
  Includes
**************************************************************************************************/
#include "arlobotconfig.h"
#include "ultrasonic.h"



#define ULTRA_SONIC_SENSOR_HCSR04
//#define ULTRA_SONIC_SENSOR_PING

#define SUPPORT_SENSOR_MUX

#ifdef ULTRA_SONIC_SENSOR_HCSR04
#define TRIG_PIN 7
#define ECHO_PIN 8
#endif

#ifdef ULTRA_SONIC_SENSOR_PING
// Provide the PING-specific defines here (nothing since we're not using PING sensors)
#define SIG_PIN 7
#endif

#ifdef SUPPORT_SENSOR_MUX
#include "sensormux.h"
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/
#if defined (ULTRA_SONIC_SENSOR_HCSR04)
#include "HCSR04.h"
#define US_SENSOR_INIT     HCSR04_Init
#define US_SENSOR_ADD      HCSR04_Add
#define US_SENSOR_PING     HCSR04_Ping
#define US_SENSOR_DISTANCE HCSR04_Distance
#elif defined (OTHER_SENSOR)
#endif

/**************************************************************************************************
  Types
**************************************************************************************************/
typedef struct _Sensor_Type
{
    int8_t handle;
    int8_t addr;
} SENSOR_TYPE;


/**************************************************************************************************
  Variables
**************************************************************************************************/
static SENSOR_TYPE sensors[MAX_NUM_US_SENSORS];
static uint8_t num_used_sensors;

/**************************************************************************************************
  Functions
**************************************************************************************************/

/**************************************************************************************************
  Initializes internal module variables
**************************************************************************************************/
void US_Init()
{
    uint8_t ii;
    
    /* The ultrasonic sensors are connected via two muxes, one for the trig pin and one for the echo pin
       The muxes need to be initialized and configured in order to use the ultrasonic sensors
     */
    
    SENMUX_Init();
    US_SENSOR_INIT();
    
    for (ii = 0; ii < MAX_NUM_US_SENSORS; ++ii)
    {
        sensors[ii].addr = -1;
        sensors[ii].handle = -1;
    }
}

/* start_offset provides a way to allocate sensors in a block, e.g. 4 sensors with offset 0, 2 sensors with offset 4, 6 sensors with offset 6 
 
    Sensors 0 .. 3 - these were allocated with first call
    Sensors 4 .. 5 - these were allocated with the second call
    Sensors 6 .. 11 - these were allocated with the last call
    
    This could be used as a way to group sensors into higher level sensor blocks, e.g., front upper deck, rear upper deck, front lower deck, rear lower deck, etc
    This mapping information would have to come from higher up in the stack though
*/
void US_Config(uint8_t num_sensors, uint8_t start_offset, uint8_t in_cm)
{
    uint8_t ii;
    
    num_used_sensors = num_sensors;
    
    for (ii = 0; ii < num_sensors; ++ii)
    {        
        sensors[ii].addr = start_offset + ii;
        sensors[ii].handle = US_SENSOR_ADD(TRIG_PIN, ECHO_PIN, in_cm);
    }
}

void US_Ping()
{
    uint8_t ii;
    
    for (ii = 0; ii < num_used_sensors; ++ii)
    {
        SENMUX_Select(sensors[ii].addr);
        US_SENSOR_PING(sensors[ii].handle);
    }
}

void US_Distance(float* distances, uint8_t* num_used)
{
    uint8_t ii;
    
    for (ii = 0; ii < num_used_sensors; ++ii)
    {
        distances[ii] = US_SENSOR_DISTANCE(sensors[ii].handle);
    }
    
    *num_used = num_used_sensors;
}