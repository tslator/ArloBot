#ifndef SENSORS_H
#define SENSORS_H

#include "hwconfig.h"
#include "imu.h"

typedef struct _Sensor_State
{
    uint8_t motion_detected;
    uint8_t digital_ir[NUM_DIGITAL_IR_SENSORS];
    float   analog_ir[NUM_ANALOG_IR_SENSORS];
    float   ultrasonic[NUM_ULTRASONIC_SENSORS];
} SENSOR_STATE;

void SensorsStart();
void GetSensorState(SENSOR_STATE* state);
void GetImuState(IMU_STATE* imu);




#endif