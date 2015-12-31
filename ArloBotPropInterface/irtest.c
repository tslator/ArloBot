/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include <simpletools.h>
#include <math.h>
#include <adcDCpropab.h>
#include "hwconfig.h"
#include "i2cbus.h"

#define AD7828_ADDR_0 0x90
#define AD7828_ADDR_1 0x92

static float calc_dist(float voltage)
{
  return 27.86 * pow(voltage, -1.15);
}

static void init_adc()
{
#ifdef USE_AB_ADC
  adc_init(PROP_A2D_CS, PROP_A2D_SCL, PROP_A2D_DO, PROP_A2D_DI);
#else
  InitI2C();
#endif
}

static float get_adc_volts(uint8_t channel)
{
#ifdef USE_AB_ADC
    return adc_volts(0);
#else
    // Two AD7828 are supported:
    //     Channels 0 - 7 are assigned to AD7828_ADDR_0
    //     Channels 8 - 15 are assigned to AD7828_ADDR_1
    uint8_t addr = channel < (NUM_ANALOG_IR_SENSORS/2) ? AD7828_ADDR_0 : AD7828_ADDR_1;
    uint16_t value = I2C_ReadADC(addr, addr + 1, channel % NUM_ANALOG_IR_SENSORS);
    float volts = ((value + 1) * 5.0) / 4096;
    
    return volts;
#endif
}        

int main()
{
  init_adc();
  
  while(1)
  {
    float value = get_adc_volts(0);
    float dist = calc_dist(value);
    print("value: %f, dist: %f\n", value, dist);
    pause(100);
  }  
}
