/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"
#include "hwconfig.h"
#include "i2cbus.h"

#define IR_REF_VOLTAGE 2.5
#define ADC_RESOLUTION 4096

static uint16_t I2CReadADC(uint8_t addr, uint8_t channel)
{
    const uint8_t CHANNEL[8] = {0x8C,0xCC,0x9C,0xDC,0xAC,0xEC,0xBC,0xFC};

    uint8_t a2d_val_high;
    uint8_t a2d_val_low;
    
    print("Calling I2C_ByteWrite ...\n");
    // Note: Passing 0 means no register
    I2C_ByteWrite(addr + AD7827_WRITE_MODIFIER, 0, CHANNEL[channel]);
    print("I2C_ByteWrite returned\n");
    pause(100);
    I2C_ByteRead(addr + AD7827_READ_MODIFIER, 0, &a2d_val_high);    
    I2C_ByteRead(addr + AD7827_READ_MODIFIER, 0, &a2d_val_low);
    
    return (a2d_val_high << 8) + a2d_val_low;
}

static float ReadAnalogIr(uint8_t index)
{
    uint16_t result;
#ifdef USE_SPI_ADC
    result = SPIReadADC(index);
#else
    // Two AD7828 are supported:
    //     Channels 0 - 7 are assigned to AD7828_ADDR_0
    //     Channels 8 - 15 are assigned to AD7828_ADDR_1
    uint8_t addr = index < (NUM_ANALOG_IR_SENSORS/2) ? AD7828_ADDR_0 : AD7828_ADDR_1;
    result = I2CReadADC(addr, index % NUM_ANALOG_IR_SENSORS);
#endif
    
    float voltage = ((result + 1) * IR_REF_VOLTAGE * 100) / ADC_RESOLUTION;
    
    //http://home.roboticlab.eu/en/examples/sensor/ir_distance
    #define CONST_A 5461.0
    #define CONST_B -17.0
    #define CONST_K 2.0
        
    if (voltage + CONST_B <= 0)
    {
        return -1;
    }
 
    return CONST_A / (voltage + CONST_B) - CONST_K;
}    


int main()
{

  InitI2C();
 
  while(1)
  {
    uint16_t value = I2CReadADC(0x90, 0);
    print("%d\n", value);
  }  
}
