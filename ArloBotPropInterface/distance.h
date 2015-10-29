#ifndef DISTANCE_H
#define DISTANCE_H

volatile float us_distances[NUM_ULTRASONIC_SENSORS];
volatile float ir_distances[NUM_INFRARED_SENSORS];


DS_Init();
DS_Start();

#endif
