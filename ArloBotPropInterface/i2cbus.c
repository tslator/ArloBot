#include "simpletools.h"
#include "i2cbus.h"

static i2c volatile i2c_bus;
static int lock;

static int i2c_startpoll(i2c* i2c_bus, uint8_t addr)
{
    int ack = i2c_poll(i2c_bus, addr);
    while (ack)
    {
        ack = i2c_poll(i2c_bus, addr);
    }
    return ack;
}    


void InitI2C()
{
    lock = locknew();
    lockclr(lock);
    i2c_open(&i2c_bus, I2C_SCL, I2C_SDA, 0);
}

uint16_t I2C_ReadADC(uint8_t write_addr, uint8_t read_addr, uint8_t channel)
{
    const uint8_t CHANNEL[8] = {0x84,0xC4,0x94,0xD4,0xA4,0xE4,0xB4,0xF4};

    uint8_t a2d_val_high;
    uint8_t a2d_val_low;
    
    int ack;
    
    lockset(lock);
    ack = i2c_startpoll(&i2c_bus, write_addr);    
    ack = i2c_writeByte(&i2c_bus, CHANNEL[0]);
    ack = i2c_startpoll(&i2c_bus, read_addr);    
    a2d_val_high = i2c_readByte(&i2c_bus, 0);
    a2d_val_low = i2c_readByte(&i2c_bus, 1);
    i2c_stop(&i2c_bus);
    lockclr(lock);
        
    return (((uint16_t) a2d_val_high) << 8) | (uint16_t) a2d_val_low;
}

//  Note: There is an inherent conflict with the i2c bus approach above and what is done below.  The ADC 
//  I2C bus had to use low level calls because the higher level calls didn't work.  The calls below were
//  translated from Arduino for the IMU.  At this point, we're not using the IMU so it doesn't matter, but
//  if that changes then either we will need to rewrite the below routines in terms of the low level i2c
//  primitives or figure out how to make the different bus approach play nicely together.  Just a note
//  to remind you of the details.

uint8_t I2C_Ready(uint8_t addr)
{
#if 0
    print("i2c_ready enter\n");
    while (i2c_busy(i2c_bus, addr))
    {
        ;
    }    
    print("i2c_ready exit\n");
#endif
}

void I2C_ByteRead(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t* data)
{
#if 0
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
    print("\t\tRead addr %02x, reg %02x, data %02x, total %d\n", slaveAddress, reg, *data, num_bytes);
#endif
}

void I2C_ByteWrite(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t data)
{
#if 0    
    uint8_t num_bytes;
    uint8_t size = 0;
    
    if (reg > 0)
    {
        size = sizeof(reg);
    }

    lockset(lock);
    if (I2C_Ready(slaveAddress))
    {
        num_bytes = i2c_out(i2c_bus, slaveAddress, reg, size, &data, sizeof(data));
    }    
    lockclr(lock);
    print("\t\tWrote - addr %02x, reg %02x, data %02x, total %d\n", slaveAddress, reg, data, num_bytes);
#endif
}

uint8_t I2CBusy(uint8_t address)
{
#if 0
    uint8_t result;
    
    lockset(lock);
    result = i2c_busy(i2c_bus, address);
    lockclr(lock);
    return result;
#endif
}
