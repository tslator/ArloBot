#ifndef IMU_H
#define IMU_H

typedef struct _Imu_State
{
    float accel_x;
    float accel_y;
    float accel_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float temp_f;
    float temp_c;
    float heading;
} IMU_STATE;


void InitImu();
void ReadImu(IMU_STATE* state);


#endif