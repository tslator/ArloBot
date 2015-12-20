#ifndef ODOM_H
#define ODOM_H


typedef struct _Odometry_State
{
    float x_dist;
    float y_dist;
    float heading;
    float gyro_heading;
    float linear_speed;
    float angular_speed;
} ODOM_STATE;

void OdomStart();
void GetOdomState(ODOM_STATE* state);

#endif