#ifndef UTILS_H
#define UTILS_H

#include "simpletools.h"


#ifndef TRUE
#define TRUE (1==1)
#endif
#ifndef FALSE
#define FALSE (!TRUE)
#endif

/**************************************************************************************************
 Returns the number of millis since the propeller started up
 Note: This function needs to be moved to a separate module like utilities or time etc.
 Note: Also, I don't think function handles rollover of CNT, but I think that is more than 24 hours
 and it is unlikely that the robot will be running for 24 hours straight
**************************************************************************************************/
uint32_t millis();

#endif