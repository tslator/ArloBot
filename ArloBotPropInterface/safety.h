#ifndef SAFETY_H
#define SAFTEY_H

#include "sensors.h"


typedef struct _Safety_State
{
    uint8_t safe_to_proceed;
    uint8_t safe_to_recede;
    uint8_t escaping;
    float min_distance_sensor;
    uint8_t cliff_detected;
    uint8_t floor_obstacle_detected;
    uint8_t ignore_proximity;
    uint8_t ignore_cliff_sensors;
    uint8_t ignore_dist_sensors;
    uint8_t ignore_floor_sensors;
} SAFETY_STATE;

void UpdateSafety(SENSOR_STATE* sensor_state, SAFETY_STATE* safety_state);

#endif