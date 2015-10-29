#ifdef GP2Y0A21YK0F_H
#define GP2Y0A21YK0F_H


#include "simpletools.h"

void GP2Y0A21YK0F_Init();
uint8_t GP2Y0A21YK0F_Add(uint8_t channel, uint8_t is_cm);
void GP2Y0A21YK0F_Ping(uint8_t handle);
float GP2Y0A21YK0F_Distance(uint8_t handle);


#endif