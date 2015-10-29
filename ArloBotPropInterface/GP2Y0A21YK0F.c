

#include "simpletools.h"
#include "GP2Y0A21YK0F.h"

#if defined (MCP3208_ADC)
#include "newmcp3208.h"
#define ADC_READ(channel)    readAdc(channel, MCP3208_DINOUT_PIN, MCP3208_CLK_PIN, MCP3208_CS_PIN)
#define IR_REF_VOLTAGE MCP3208_REFERENCE_VOLTAGE
#define ADC_RESOLUTION 4096.0
#elif defined (PROP_ADC)
#define ADC_READ
#endif

typedef struct _GP2Y0A21YK0F_Instance
{
    int8_t channel;
    uint8_t used;
} GP2Y0A21YK0F_INSTANCE;

static GP2Y0A21YK0F_INSTANCE gp2y0a21yk0f_array[NUM_INFRARED_SENSORS];


//https://www.tindie.com/products/upgradeindustries/sharp-10-80cm-infrared-distance-sensor-gp2y0a21yk0f

/*
From 8-bit ADC to Distance in Centimeters:

distance = 4 * 12343.85 * (8bit reading)^-1.15

From 10-bit ADC to Distance in Centimeters:

distance = 12343.85 * (10bit reading)^-1.15

From Voltage to Distance in Centimeters:

distance (cm) = 27.86 (voltage reading)^-1.15
*/
float calc_dist_in_cm_1(uint16_t voltage)
{
    int dist;
    
    dist = 27.86 * pow(voltage, -1.15); // https://www.tindie.com/products/upgradeindustries/sharp-10-80cm-infrared-distance-sensor-gp2y0a21yk0f/    
    
    return dist;
}


//http://home.roboticlab.eu/en/examples/sensor/ir_distance

/*
 1 / (d + k) = a * ADC + b

where

    d - distance in centimeters.
    k - corrective constant (fund using tial-and-error method)
    ADC - digitalized value of voltage.
    a - linear member (value is determined by the trend line equation)
    b - free member(value is determined by the trend line equation)

     Distance d can be expressed from the formula:

d = (1 / (a * ADC + B)) - k

Now it is basically possible to calculate the distance by using this formula, but this requires floating-point calculations, since while dividing fractions will occur. Because the microcontroller operates using integers, the formula must be simplified and converted to larger ratios. Then when dividing the quotient with a linear-member it will look as follows:

d = (1 / a) / (ADC + B / a) - k

When introducing the corrective constant to the formula and also the linear-member and the free-member from the trend-line equation, the formula for calculating the distance will be:

d = 5461 / (ADC - 17) - 2

This formula is computable with 16-bit integers and completely suitable to AVR. Before calculating, must be ensured that the value of the ADC is over 17, otherwise dividing with 0 or negative distance may occur. 
*/
static float calc_dist_in_cm_2(uint16_t voltage)
{
    #define CONST_A 5461.0
    #define CONST_B -17.0
    #define CONST_K 2.0
    
    if (voltage + CONST_B <= 0)
    {
        return -1;
    }
 
    return CONST_A / (voltage + CONST_B) - CONST_K;
}


#if defined (USE_METHOD_1))
#define CALC_DIST_IN_CM calc_dist_in_cm_1
#elif defined (USE_METHOD_2)
#define CALC_DIST_IN_CM calc_dist_in_cm_2
#endif

static float measure_voltage(uint8_t handle)
{
    int adc_reading;

    adc_reading = ADC_READ(sensor[handle].channel);
    sensor[handle].voltage = (adc_reading + 1) * IR_REF_VOLTAGE / ADC_RESOLUTION;
}

void GP2Y0A21YK0F_Init()
{
    uint8_t ii;
    
    for (ii = 0; ii < NUM_INFRARED_SENSORS; ++ii)
    {
        gp2y0a21yk0f_array[ii].channel = -1;
        gp2y0a21yk0f_array[ii].used = 0;
    }
}

uint8_t GP2Y0A21YK0F_Add(uint8_t channel, uint8_t is_cm)
{
    uint8_t ii;
    
    for (ii = 0; ii < NUM_INFRARED_SENSORS; ++ii)
    {
        if (!gp2y0a21yk0f_array[ii].used)
        {
            gp2y0a21yk0f_array[ii].used = 1;
            gp2y0a21yk0f_array[ii].channel = channel;
        }
    }
}

void GP2Y0A21YK0F_Ping(uint8_t handle)
{
    measure_voltage(handle);
}


float GP2Y0A21YK0F_Distance(uint8_t handle)
{
    return CALC_DIST_IN_CM(gp2y0a21yk0f_array[ii].voltage);
}

#endif