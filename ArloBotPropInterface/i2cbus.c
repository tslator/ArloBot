#include "simpletools.h"
#include "i2cbus.h"

static i2c* i2c_bus;
static int lock;

void InitI2C()
{
    lock = locknew();
    lockclr(lock);
    i2c_bus = i2c_newbus(I2C_SCL, I2C_SDA, 0);
}

void I2C_ByteRead(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t* data)
{
    uint8_t num_bytes;
    uint8_t size = 0;
    
    if (reg > 0)
    {
        size = sizeof(reg);
    }
    
    lockset(lock);
    while (i2c_busy(i2c_bus, slaveAddress))
    {
        ;
    }    
    num_bytes = i2c_in(i2c_bus, slaveAddress, reg, size, data, 1);
    lockclr(lock);
    //print("\t\tRead addr %02x, reg %02x, data %02x, total %d\n", slaveAddress, reg, *data, num_bytes);
}

void I2C_ByteWrite(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t data)
{
    uint8_t num_bytes;
    uint8_t size = 0;
    
    if (reg > 0)
    {
        size = sizeof(reg);
    }

    lockset(lock);
    num_bytes = i2c_out(i2c_bus, slaveAddress, reg, size, &data, sizeof(data));
    while (i2c_busy(i2c_bus, slaveAddress))
    {
        ;
    }    
    lockclr(lock);
    //print("\t\tWrote - addr %02x, reg %02x, data %02x, total %d\n", slaveAddress, reg, data, num_bytes);
}

uint8_t I2CBusy(uint8_t address)
{
    uint8_t result;
    
    lockset(lock);
    result = i2c_busy(i2c_bus, address);
    lockclr(lock);
    return result;
}
