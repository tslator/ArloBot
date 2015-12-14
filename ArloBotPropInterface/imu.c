#include <simpletools.h>
#include "imu.h"
#include "lm303c.h"
#include "hwconfig.h"
#include "i2cbus.h"


#define SENSITIVITY_ACC   0.06103515625   // LSB/mg
#define SENSITIVITY_MAG   0.00048828125   // LSB/Ga


static void MAG_ReadReg(MAG_REG_t reg, uint8_t* data)
{
    I2C_ByteRead(MAG_I2C_ADDR, reg, data);
}

static uint8_t MAG_WriteReg(MAG_REG_t reg, uint8_t data)
{
    I2C_ByteWrite(MAG_I2C_ADDR, reg, data);
}

static void ACC_ReadReg(ACC_REG_t reg, uint8_t* data)
{
    I2C_ByteRead(ACC_I2C_ADDR, reg, data);
}

static uint8_t ACC_WriteReg(ACC_REG_t reg, uint8_t data)
{
    I2C_ByteWrite(ACC_I2C_ADDR, reg, data);
}

void InitImu()
{
    uint8_t value;
    uint8_t test_value;
    
    print("Initializing IMU ... \n");
    
    while (I2CBusy(MAG_I2C_ADDR))
    {
        ;
    }
    print("\tMagnetometer is ready\n");
    while (I2CBusy(ACC_I2C_ADDR))
    {
        ;
    }
    print("\tAccelerometer is ready\n");

    print("\tConfigure Magnetometer\n");
    
    MAG_ReadReg(0x0F, &value);
    //print("MAG Who Am I: %02x\n", value);
    
    
    //successes += MAG_SetODR(modr);
    MAG_ReadReg(MAG_CTRL_REG1, &value);
    value &= ~(MAG_DO_80_Hz | 0x03);
    value |= MAG_DO_40_Hz;
    MAG_WriteReg(MAG_CTRL_REG1, value);
    //MAG_ReadReg(MAG_CTRL_REG1, &test_value);
    //print("\t\t%02x == %02x -> %s\n", value, test_value, value == test_value ? "TRUE" : "FALSE");


    // Initialize magnetic field full scale
    //successes += MAG_SetFullScale(mfs);
    MAG_ReadReg(MAG_CTRL_REG2, &value);
    value &= ~MAG_FS_16_Ga; //mask
    value |= MAG_FS_16_Ga;
    MAG_WriteReg(MAG_CTRL_REG2, value);
    
    // Enabling block data updating
    //successes += MAG_BlockDataUpdate(mbu);
    MAG_ReadReg(MAG_CTRL_REG5, &value);
    //print("MAG_CTRL_REG5 (%02x), %02x\n", MAG_CTRL_REG5, value);
    value &= ~MAG_BDU_ENABLE; //mask
    value |= MAG_BDU_ENABLE;
    //print("MAG_CTRL_REG5 (%02x), %02x\n", MAG_CTRL_REG5, value);
    MAG_WriteReg(MAG_CTRL_REG5, value);
    
    // Initialize magnetometer X/Y axes ouput data rate
    //successes += MAG_XY_AxOperativeMode(mxyodr);
    MAG_ReadReg(MAG_CTRL_REG1, &value);
    value &= ~MAG_OMXY_ULTRA_HIGH_PERFORMANCE; //mask
    value |= MAG_OMXY_HIGH_PERFORMANCE;
    MAG_WriteReg(MAG_CTRL_REG1, value);
  
    // Initialize magnetometer Z axis performance mode
    //successes += MAG_Z_AxOperativeMode(mzodr);
    MAG_ReadReg(MAG_CTRL_REG4, &value);
    value &= ~MAG_OMZ_ULTRA_HIGH_PERFORMANCE; //mask
    value |= MAG_OMZ_HIGH_PERFORMANCE;
    MAG_WriteReg(MAG_CTRL_REG4, value);
    
    // Initialize magnetometer run mode.
    //successes += MAG_SetMode(mm);
    MAG_ReadReg(MAG_CTRL_REG3, &value);
    value &= ~MAG_MD_POWER_DOWN_2;
    value |= MAG_MD_CONTINUOUS;
    MAG_WriteReg(MAG_CTRL_REG3, value);
    //print("\tComplete\n");

    
    //print("\tConfigure Accelerometer\n");
    
    ////////// Initialize Accelerometer //////////
    // Initialize acceleration full scale
    //successes += ACC_SetFullScale(afs);
    ACC_ReadReg(ACC_CTRL4, &value);
    value &= ~ACC_FS_8g;
    value |= ACC_FS_2g;
    ACC_WriteReg(ACC_CTRL4, value);
    
    
    // Enable block data updating
    //successes += ACC_BlockDataUpdate(abu);
    ACC_ReadReg(ACC_CTRL1, &value);
    value &= ~ACC_BDU_ENABLE;
    value |= ACC_BDU_ENABLE;
    ACC_WriteReg(ACC_CTRL1, value);
    
    // Enable X, Y, and Z accelerometer axes
    //successes += ACC_EnableAxis(aea);
    ACC_ReadReg(ACC_CTRL1, &value);
    value &= ~0x07;
    value |= ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE;
    ACC_WriteReg(ACC_CTRL1, value);
        
    // Initialize accelerometer output data rate
    //successes += ACC_SetODR(aodr);
    ACC_ReadReg(ACC_CTRL1, &value);
    value &= ~ACC_ODR_MASK;
    value |= ACC_ODR_100_Hz;
    ACC_WriteReg(ACC_CTRL1, value);

    //print("\tComplete\n");

    //print("Enable Temperature Sensor\n");
    // Enable temperature sensor
    MAG_ReadReg(MAG_CTRL_REG1, &value);
    value &= ~MAG_TEMP_EN_ENABLE; //mask
    value |= MAG_TEMP_EN_ENABLE;
    MAG_WriteReg(MAG_CTRL_REG1, value);
    
    //print("\tComplete\n");
    //print("Complete!\n");
}

static void CalcHeading(IMU_STATE* state)
{
    state->heading = 0;
}

void ReadImu(IMU_STATE* state)
{
    int16_t temp_value;
    uint8_t valueH;
    uint8_t valueL;
    
    ACC_ReadReg(ACC_OUT_X_H, &valueH);
    ACC_ReadReg(ACC_OUT_X_L, &valueL);
    temp_value = (valueH << 8) | valueL;
    state->accel_x = temp_value * SENSITIVITY_ACC;
    
    ACC_ReadReg(ACC_OUT_Y_H, &valueH);
    ACC_ReadReg(ACC_OUT_Y_L, &valueL);
    temp_value = (valueH << 8) | valueL;
    state->accel_y = temp_value * SENSITIVITY_ACC;
    
    ACC_ReadReg(ACC_OUT_Z_H, &valueH);
    ACC_ReadReg(ACC_OUT_Z_L, &valueL);
    temp_value = (valueH << 8) | valueL;
    state->accel_z = temp_value * SENSITIVITY_ACC;
    
    MAG_ReadReg(MAG_OUTX_L, &valueL);
    MAG_ReadReg(MAG_OUTX_H, &valueH);
    temp_value = (valueH << 8) | valueL;
    state->mag_x = temp_value * SENSITIVITY_MAG;

    MAG_ReadReg(MAG_OUTY_L, &valueL);
    MAG_ReadReg(MAG_OUTY_H, &valueH);
    temp_value = (valueH << 8) | valueL;
    state->mag_y = temp_value * SENSITIVITY_MAG;

    MAG_ReadReg(MAG_OUTZ_L, &valueL);
    MAG_ReadReg(MAG_OUTZ_H, &valueH);
    temp_value = (valueH << 8) | valueL;
    state->mag_z = temp_value * SENSITIVITY_MAG;
    
    MAG_ReadReg(MAG_TEMP_OUT_L, &valueL);
    MAG_ReadReg(MAG_TEMP_OUT_H, &valueH);
    // Note: tempurature values is represented as a 2's complement value
    temp_value = (valueH << 8) | valueL;
    // 8 digits/˚C
    // Reads 0 @ 25˚C
    state->temp_c = (temp_value/8.0) + 25.0;
    state->temp_f = (state->temp_c * 9.0 / 5.0) + 32.0;
    
    CalcHeading(state);
}
