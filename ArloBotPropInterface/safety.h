#ifndef SAFETY_H
#define SAFETY_H

// For "Safety Override" Cog
volatile int safeToProceed = 0;
volatile int safeToRecede = 0;
volatile int cliff = 0;
volatile int floorO = 0;
volatile int Escaping = 0;
volatile int minDistanceSensor = 0;
volatile int ignoreProximity = 0;
volatile int ignoreCliffSensors = 0;
volatile int ignoreFloorSensors = 0;
volatile int ignoreIRSensors = 0;
volatile int pluggedIn = 0;


void SAFETY_Init();
void SAFETY_Start();

#endif