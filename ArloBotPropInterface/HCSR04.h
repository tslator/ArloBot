#ifndef HCSR04_H
#define HCSR04_H

#include "simpletools.h"

void HCSR04_Init();
int8_t HCSR04_Add(int8_t trig_pin, int8_t echo_pin, uint8_t in_cm);
void HCSR04_Ping(uint8_t handle);
float HCSR04_Distance(uint8_t handle);

#endif