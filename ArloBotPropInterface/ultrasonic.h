/**************************************************************************************************
 The purpose of this module is to provide access to Ultrasonic sensor data
**************************************************************************************************/
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

/**************************************************************************************************
 Includes
**************************************************************************************************/
#include "simpletools.h"

/* Initialize the module */
void US_Init();

/* Configure the module */
void US_Config(uint8_t num_sensors, uint8_t start_offset, uint8_t in_cm);

void US_Ping();

/* The array passed in need to be an array of floats with size MAX_NUM_US_SENSORS */
void US_Distance(float* distances, uint8_t* num_used);
        

#endif
/**************************************************************************************************
EOF
*/

