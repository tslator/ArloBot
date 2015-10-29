//#include "per_robot_settings_for_propeller_c_code.h"


//#define GYRO_SUPPORT
//#define IR_SENSORS_SUPPORT
//#define US_SENSORS_SUPPORT 

#define MAX_NUM_US_SENSORS (16)
#define MAX_NUM_IR_SENSORS (8)

#ifdef IR_SENSORS_SUPPORT
#include "infrared.h"
#endif

#ifdef US_SENSORS_SUPPORT
#include "ultrasonic.h"
#endif


#ifdef hasMotorPowerMonitorCircuit
#include "powermonitor.h"
#endif

// Global Storage for PING & IR Sensor Data:
#ifdef hasFloorObstacleSensors
int floorArray[NUMBER_OF_FLOOR_SENSORS] = {0};
#endif

