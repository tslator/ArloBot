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
    float   left_motor_voltage;
    float   right_motor_voltage;
    float   left_motor_current;
    float   right_motor_current;
} SENSOR_STATE;

void SensorsStart();
void GetSensorState(SENSOR_STATE* state);
#if 0
void GetImuState(IMU_STATE* imu);
#endif




#endif