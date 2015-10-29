#ifndef NEWMCP3208_H
#define NEWMCP3208_H

#include "simpletools.h"

int readADC(int channel, int dinout, int clk, int cs);

void MCP3208_Init();
int8_t MCP3208_Add(uint8_t addr, uint8_t in_cm);
void MCP3208_Ping(uint8_t addr);
float MCP3208_Distance(uint8_t addr);


#endif
