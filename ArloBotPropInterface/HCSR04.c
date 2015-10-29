
#include "simpletools.h"
#include "arlobotconfig.h"
#include "const.h"
#include "ultrasonic.h"
#include "HCSR04.h"

typedef struct _HCSR04_Instance 
{
    int8_t  trig_pin;
    int8_t  echo_pin;
    float   units;
    uint32_t  pulse;
    float   distance;
    uint8_t used;
} HCSR04_INSTANCE;

static HCSR04_INSTANCE hcsr04_array[MAX_NUM_US_SENSORS];

static void HCSR04_MeasurePulse(uint8_t handle) 
{
    uint32_t pulse_start = 0;
    uint32_t pulse_end = 0;
    
    /* The HCSR04 requires a 10us pulse on the trigger pin to start
       the measurement
    */
    low(hcsr04_array[handle].trig_pin);
    usleep(2);
    high(hcsr04_array[handle].trig_pin); 
    usleep(10); 
    low(hcsr04_array[handle].trig_pin);

    /* Measure the pulse by capturing when the pin goes high and then low again
     */
    
    while (get_state(hcsr04_array[handle].echo_pin) == LOW) {
        pulse_start = CNT;
    }
    
    while (get_state(hcsr04_array[handle].echo_pin) == HIGH) {
        pulse_end = CNT;
    }
    
    hcsr04_array[handle].pulse = pulse_end - pulse_start;
}


void HCSR04_Init()
{
    uint8_t ii;
    
    for (ii = 0; ii < MAX_NUM_US_SENSORS; ++ii)
    {
        hcsr04_array[ii].trig_pin = -1;
        hcsr04_array[ii].echo_pin = -1;
        hcsr04_array[ii].units = 0.0;
        hcsr04_array[ii].pulse = 0;
        hcsr04_array[ii].distance = 0.0;
        hcsr04_array[ii].used = 0;
    }
}

int8_t HCSR04_Add(int8_t trig_pin, int8_t echo_pin, uint8_t in_cm)
{
    int8_t ii;
    
    for (ii = 0; ii < MAX_NUM_US_SENSORS; ++ii)
    {
        if (!hcsr04_array[ii].used)
        {
            hcsr04_array[ii].used = 1;
            hcsr04_array[ii].trig_pin = trig_pin;
            set_direction(trig_pin, 1);
            hcsr04_array[ii].echo_pin = echo_pin;
            set_direction(echo_pin, 0);
            hcsr04_array[ii].units = in_cm ? SPEED_OF_SOUND_CM_PER_SEC : SPEED_OF_SOUND_INCH_PER_SEC;
            
            return ii;
        }
    }
    
    // Warn if an attempt is made to add more sensors than are supported
    return -1;
}

void HCSR04_Ping(uint8_t handle)
{
    HCSR04_MeasurePulse(handle);
}

float HCSR04_Distance(uint8_t handle)
{
    return hcsr04_array[handle].pulse / hcsr04_array[handle].units;
}