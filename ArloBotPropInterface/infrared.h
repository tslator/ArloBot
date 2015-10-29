#ifndef INFRARED_H
#define INFRARED_H

#include "simpletools.h"


void IR_Init();
void IR_Config(uint8_t num_sensors, uint8_t start_channel, uint8_t in_cm);
void IR_Ping();
void IR_Distance(float* distances, uint8_t* num_used);


#endif