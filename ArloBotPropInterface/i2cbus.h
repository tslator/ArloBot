#ifndef I2CBUS_H
#define I2CBUS_H

#include "hwconfig.h"

typedef enum
{
    ACC_I2C_ADDR = 0x1D,
    MAG_I2C_ADDR = 0x1E,
    AD7828_ADDR  = 0x90
} I2C_ADDR_t;


void InitI2C();
void I2C_ByteRead(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t* data);
void I2C_ByteWrite(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t data);
uint8_t I2CBusy(uint8_t address);


#endif