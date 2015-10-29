#include "safety.h"
#include "distance.h"

static int blockedSensor[NUMBER_OF_PING_SENSORS] = {0};
static int i;
static int blockedF = 0
static int blockedR = 0
static int foundCliff = 0
static int floorObstacle = 0
static int pleaseEscape = 0
static int minDistance = 255
static int minRDistance = 255
static int newSpeedLimit = 100;

extern int speedLeft, speedRight, throttleStatus = 0;


// Use a cog to squelch incoming commands and perform safety procedures like halting, backing off, avoiding cliffs, calling for help, etc.
// This can use proximity sensors to detect obstacles (including people) and cliffs
// This can use the gyro to detect tipping
// This can use the gyro to detect significant heading errors due to slipping wheels when an obstacle is encountered or high centered
static int safetyOverrideStack[128]; // If things get weird make this number bigger!


typedef struct
{
    int found;
    int* ignore;
    uint8_t first;
    uint8_t last;
    
} FIND_A_GOOD_NAME_TYPE;

void CS_Evalulate()
{
#ifdef hasCliffSensors
    foundCliff = 0;
    if (ignoreCliffSensors == 0) {
        // Check Cliff Sensors first
        for (i = FIRST_CLIFF_SENSOR; i < FIRST_CLIFF_SENSOR + NUMBER_OF_CLIFF_SENSORS; i++) {
            if (ir_distances[i] > FLOOR_DISTANCE) {
                safeToProceed = 0; // Prevent main thread from setting any drive_speed
                // Stop robot if it is currently moving forward and not escaping
                // TODO: Can we "chase" the robot off of a cliff, because the rear sensor
                // would put us into an "Escaping == 1" situation, but we would be moving forward
                // OVER the cliff instead of back, and thus really need to stop now?!
                if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
                    drive_speed(0, 0);
                }
                // Use this to give the "all clear" later if it never gets set
                blockedF = 1;
                // Use this to clear the 'cliff' variable later if this never gets set.
                foundCliff = 1;
                // Set the global 'cliff' variable so we can see this in ROS.
                cliff = 1;
                blockedSensor[2] = 1; // Pretend this is the front sensor, since it needs to back up NOW!
                pleaseEscape = 1;
            }
        }
    }
    // Clear the global 'cliff' variable if no cliff was seen.
    if (foundCliff == 0) {
        cliff = 0;
    }
#endif
}

void FOS_Evaulate()
{
#ifdef hasFloorObstacleSensors
    floorObstacle = 0;
    if (ignoreFloorSensors == 0) {
      for (i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) {
        if (floorArray[i] == 0) {
          safeToProceed = 0; // Prevent main thread from setting any drive_speed
          // Stop robot if it is currently moving forward and not escaping
          // TODO: Can we "chase" the robot off of a cliff, because the rear sensor
          // would put us into an "Escaping == 1" situation, but we would be moving forward
          // OVER the cliff instead of back, and thus really need to stop now?!
          if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
            drive_speed(0, 0);
          }
          // Use this to give the "all clear" later if it never gets set
          blockedF = 1;
          // Use this to clear the 'floorO' variable later if this never gets set.
          floorObstacle = 1;
          // Set the global 'floorO' variable so we can see this in ROS.
          floorO = 1;
          blockedSensor[2] = 1; // Pretend this is the front sensor, since it needs to back up NOW!
          pleaseEscape = 1;
          }
      }
    }
    // Clear the global 'floorO' variable if no floor obstacle was seen.
    if (floorObstacle == 0) {
        floorO = 0;
    }
#endif
}

void FPS_Evaulate()
{
#ifdef hasFrontPingSensors
    // Walk Front PING Sensors to find blocked paths and halt immediately
    for (i = FIRST_FRONT_PING_SENSOR_NUMBER; i < HOW_MANY_FRONT_PING_SENSORS + FIRST_FRONT_PING_SENSOR_NUMBER; i++) {
        // PING Sensors
        if (us_distances[i] < startSlowDownDistance[i]) {
            if (us_distances[i] <= haltDistance[i] + 1) { // Halt just before.
                safeToProceed = 0; // Prevent main thread from setting any drive_speed
                // Stop robot if it is currently moving forward and not escaping
                if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
                    drive_speed(0, 0);
                }
                blockedF = 1; // Use this to give the "all clear" later if it never gets set
                blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                // Escape just after, to try make a buffer to avoid back and forthing.
                if (us_distances[i] < haltDistance[i]) {
                    pleaseEscape = 1;
                }
            }
            // For speed restriction:
            if (us_distances[i] < minDistance) {
                minDistance = us_distances[i];
                minDistanceSensor = i;
            }
        }
    }
#endif
}

void FUDS_Evaulate()
{
#ifdef hasFrontUpperDeckSensors
    // Walk Upper Deck Sensors
    for (i = FIRST_FRONT_UPPER_SENSOR_NUMBER; i < HOW_MANY_FRONT_UPPER_SENSORS + FIRST_FRONT_UPPER_SENSOR_NUMBER; i++) {
        // PING Sensors
        if (us_distances[i] < startSlowDownDistance[i]) {
            // Halt just before.
            if (us_distances[i] <= haltDistance[i] + 1) {
                // Prevent main thread from setting any drive_speed
                safeToProceed = 0;
                // Stop robot if it is currently moving forward and not escaping
                if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
                    drive_speed(0, 0);
                }
                 // Use this to give the "all clear" later if it never gets set
                blockedF = 1;
                 // Keep track of which sensors are blocked for intelligent escape sequences.
                blockedSensor[i] = 1;
                if (us_distances[i] < haltDistance[i]) {
                    // Escape just after, to try make a buffer to avoid back and forthing.
                    pleaseEscape = 1;
                }
            }
            // For speed restriction:
            if (us_distances[i] < minDistance) {
                minDistance = us_distances[i];
                minDistanceSensor = i;
            }
        }
    }
#endif
}

void RPS_Evaulate()
{
#ifdef hasRearPingSensors
    // Walk REAR Sensor Array to find blocked paths and halt immediately
    for (i = FIRST_REAR_PING_SENSOR_NUMBER; i < FIRST_REAR_PING_SENSOR_NUMBER + HOW_MANY_REAR_PING_SENSORS; i++) {
        if (us_distances[i] < startSlowDownDistance[i]) {
            if (us_distances[i] <= haltDistance[i] + 1) { // Halt just before.
                safeToRecede = 0; // Prevent main thread from setting any drive_speed
                // Stop robot if it is currently moving forward and not escaping
                if ((Escaping == 0) && ((speedLeft + speedRight) < 0)) {
                    drive_speed(0, 0);
                }
                blockedR = 1; // Use this to give the "all clear" later if it never gets set
                blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                if (us_distances[i] < haltDistance[i]) // Escape just after, to try make a buffer to avoid back and forthing.
                    pleaseEscape = 1;
            }
            // For speed restriction:
            if (us_distances[i] < minRDistance) {
                minRDistance = us_distances[i];
                minDistanceSensor = i;
            }
        }
    }
#endif
}

void RUDS_Evaulate()
{
#ifdef hasRearUpperDeckSensors
    for (i = FIRST_REAR_UPPER_SENSOR_NUMBER; i < FIRST_REAR_UPPER_SENSOR_NUMBER + HOW_MANY_REAR_UPPER_SENSORS; i++) { // Only use the rear sensors
        // PING Sensors
        if (us_distances[i] < startSlowDownDistance[i]) {
            if (us_distances[i] <= haltDistance[i] + 1) { // Halt just before.
                safeToRecede = 0; // Prevent main thread from setting any drive_speed
                // Stop robot if it is currently moving forward and not escaping
                if ((Escaping == 0) && ((speedLeft + speedRight) < 0)) {
                    drive_speed(0, 0);
                }
                blockedR = 1; // Use this to give the "all clear" later if it never gets set
                blockedSensor[i] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                if (us_distances[i] < haltDistance[i]) // Escape just after, to try make a buffer to avoid back and forthing.
                    pleaseEscape = 1;
            }
            // For speed restriction:
            if (us_distances[i] < minRDistance) {
                minRDistance = us_distances[i];
                minDistanceSensor = i;
            }
        }
    }
#endif
}

void FIRS_Evaulate()
{
#ifdef hasFrontIRSensors
    // Walk front IR Sensors
    for (i = FIRST_FRONT_IR_SENSOR_NUMBER; i < HOW_MANY_FRONT_IR_SENSORS + FIRST_FRONT_IR_SENSOR_NUMBER; i++) {
        if (ir_distances[i] < IRstartSlowDownDistance[i])  {
            if (ir_distances[i] <= haltDistance[i] + 1) {
                // Prevent main thread from setting any drive_speed
                safeToProceed = 0;
                // Stop robot if it is currently moving forward and not escaping
                if ((Escaping == 0) && ((speedLeft + speedRight) > 0)) {
                    drive_speed(0, 0);
                }
                // Use this to give the "all clear" later if it never gets set
                blockedF = 1;
                // Keep track of which sensors are blocked for intelligent escape sequences.
                blockedSensor[i] = 1;
                if (ir_distances[i] < haltDistance[i]) {
                    pleaseEscape = 1;
                }
            }
            // For speed restriction:
            if (ir_distances[i] < minDistance) {
                minDistance = ir_distances[i];
                minDistanceSensor = i;
            }
        }
    }
#endif
}

void RIRS_Evaulate()
{
#ifdef hasRearIRSensors
    for (i = FIRST_REAR_IR_SENSOR_NUMBER; i < FIRST_REAR_IR_SENSOR_NUMBER + HOW_MANY_REAR_IR_SENSORS; i++) {
        #ifdef RENAME_REAR_IR_SENSOR
        int sensorFakeIndex = RENAME_REAR_IR_SENSOR;
        #else
        int sensorFakeIndex = i;
        #endif
        if (ir_distances[i] < IRstartSlowDownDistance[i]) {
            if (ir_distances[i] <= haltDistance[sensorFakeIndex] + 1) {
                safeToRecede = 0; // Prevent main thread from setting any drive_speed
                // Stop robot if it is currently moving forward and not escaping
                if ((Escaping == 0) && ((speedLeft + speedRight) < 0)) {
                    drive_speed(0, 0);
                }
                blockedR = 1; // Use this to give the "all clear" later if it never gets set
                blockedSensor[sensorFakeIndex] = 1; // Keep track of which sensors are blocked for intelligent escape sequences.
                if (ir_distances[i] < haltDistance[sensorFakeIndex])
                    pleaseEscape = 1;
            }
            // For speed restriction:
            if (ir_distances[i] < minRDistance) {
                minRDistance = ir_distances[i];
                minDistanceSensor = i;
            }
        }
    }    
#endif
}

/* TESTS:
   1. Make sure output sensor readings to ROS are near real time.
   2. Make sure "escape" operations are fast and accurate.
   */
static void safetyOverride(void *par) {
    
    int throttleRamp = 0;
    
    
    while (1) {
        if (ignoreProximity == 0) {
            // Reset blockedSensor array to all zeros.
            memset(blockedSensor, 0, sizeof(blockedSensor));
            blockedF = 0;
            blockedR = 0;
            pleaseEscape = 0;
            minDistance = 255;

            CS_Evalulate();
            FOS_Evaulate();
            FPS_Evaulate();
            FUDS_Evaulate();
            RPS_Evaulate();
            RUDS_Evaulate();

            if (ignoreIRSensors == 0) {
                FIRS_Evaulate();
                RIRS_Evaulate();
            }

            // Reduce Speed Limit when we are close to an obstruction
            /* EXPLANATION minDistance won't be set unless a given sensor is closer than its particular startSlowDownDistance value, so we won't be slowing down if sensor 0 is 40, only if it is under 10 */
            if (minDistance < MAX_DISTANCE) {
                // Set based on percentage of range
                // TODO: Is this a good method?
                newSpeedLimit = (minDistance - haltDistance[minDistanceSensor]) * (100 / (MAX_DISTANCE - haltDistance[minDistanceSensor]));
                // Limit maximum and minimum speed.
                if (newSpeedLimit < MINIMUM_SPEED) {
                    newSpeedLimit = MINIMUM_SPEED;
                } else if (newSpeedLimit > 100) {
                    newSpeedLimit = 100;
                }
                // Ramp and limit affect of random hits
                if (newSpeedLimit > abd_speedLimit) {
                    if (throttleRamp == THROTTLE_STOP) {
                        abd_speedLimit = abd_speedLimit + 1;
                    }
                } else if (newSpeedLimit < abd_speedLimit) {
                    if (throttleRamp == THROTTLE_STOP) {
                        abd_speedLimit = abd_speedLimit - 1;
                    }
                }
            } else {
                // Ramp return to full if all obstacles are clear
                if (abd_speedLimit < 100) {
                    if (throttleRamp == THROTTLE_STOP) // Slow ramping down
                        abd_speedLimit = abd_speedLimit + 1;
                }
            }

            // Same for REVERSE Speed Limit
            if (minRDistance < MAX_DISTANCE) {
                // Set based on percentage of range
                // TODO: Is this a good method?
                newSpeedLimit = (minRDistance - haltDistance[minDistanceSensor]) * (100 / (MAX_DISTANCE - haltDistance[minDistanceSensor]));
                // Limit maximum and minimum speed.
                if (newSpeedLimit < MINIMUM_SPEED) {
                    newSpeedLimit = MINIMUM_SPEED;
                } else if (newSpeedLimit > 100) {
                    newSpeedLimit = 100;
                }
                // Ramp and limit affect of random hits
                if (newSpeedLimit > abdR_speedLimit) {
                    if (throttleRamp == THROTTLE_STOP) {
                        abdR_speedLimit = abdR_speedLimit + 1;
                    }
                } else if (newSpeedLimit < abdR_speedLimit) {
                    if (throttleRamp == THROTTLE_STOP) {
                        abdR_speedLimit = abdR_speedLimit - 1;
                    }
                }
            } else {
                // Ramp return to full if all obstacles are clear
                if (abdR_speedLimit < 100) {
                    if (throttleRamp == THROTTLE_STOP) // Slow ramping down
                        abdR_speedLimit = abdR_speedLimit + 1;
                }
            }

            // Clear forward and backward individually now.
            if (blockedF == 0) {
                safeToProceed = 1;
            }
            if (blockedR == 0) {
                safeToRecede = 1;
            }
            // If NO sensors are blocked, give the all clear!
            if (blockedF == 0 && blockedR == 0) {
                if (Escaping == 1) {// If it WAS escaping before stop it before releasing it
                    drive_speed(0, 0); // return to stopped before giving control back to main thread
                }
                Escaping = 0; // Have fun!
            } else {
                if (pleaseEscape == 1 && pluggedIn == 0) {
                    // If it is plugged in, don't escape!
                    Escaping = 1; // This will stop main thread from driving the motors.
                    /* At this point we are blocked, so it is OK to take over control
                       of the robot (safeToProceed == 0, so the main thread won't do anything),
                       and it is safe to do work ignoring the need to slow down or stop
                       because we know our position pretty well.
                       HOWEVER, you will have to RECHECK distances yourself if you are going to move
                       in this program location.
                       */
                       if (safeToRecede == 1) {
                           
                        /* Note: maybe a data structure would work here where the left/right speeds would be stored for each sensor index
                        
                           typedef struct { int active; int left; int right;} ESCAPE_SPEED;
                           ESCAPE_SPEED blockedSensor[] = { ... }
                           
                           if (blockedSensor[FRONT_CENTER_SENSOR].active){
                               drive_speed(blockedSensor[FRONT_CENTER_SENSOR].left, blockedSensor[FRONT_CENTER_SENSOR].right);
                               or 
                               index = FRONT_CENTER_SENSOR;
                           }
                           ...
                           or
                           drive_speed(blockedSensor[index].left, blockedSensor[FRONT_CENTER_SENSOR].right);
                           
                        
                        */
                           
                        // The order here determines priority.
                        #ifdef FRONT_CENTER_SENSOR
                        if (blockedSensor[FRONT_CENTER_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, -MINIMUM_SPEED);
                        #ifdef FRONT_3D_MOUNTED_SENSOR
                        } else if (blockedSensor[FRONT_3D_MOUNTED_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, -MINIMUM_SPEED);
                        #endif
                        #ifdef FRONT_UPPER_DECK_SENSOR
                        } else if (blockedSensor[FRONT_UPPER_DECK_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, -MINIMUM_SPEED);
                        #endif
                        #ifdef FRONT_NEAR_LEFT_SENSOR
                        } else if (blockedSensor[FRONT_NEAR_LEFT_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, -(MINIMUM_SPEED * 2)); // Curve out to the right
                        #endif
                        #ifdef FRONT_NEAR_RIGHT_SENSOR
                        } else if (blockedSensor[FRONT_NEAR_RIGHT_SENSOR] == 1) {
                            drive_speed(-(MINIMUM_SPEED * 2), -MINIMUM_SPEED); // Curve out to the left
                        #endif
                        #ifdef FRONT_FAR_LEFT_SENSOR
                        } else if (blockedSensor[FRONT_FAR_LEFT_SENSOR] == 1) {
                            drive_speed(0, -MINIMUM_SPEED); // Turn out to the right slowly
                        #endif
                        #ifdef FRONT_FAR_RIGHT_SENSOR
                        } else if (blockedSensor[FRONT_FAR_RIGHT_SENSOR] == 1) {
                            drive_speed(-MINIMUM_SPEED, 0); // Turn out to the left slowly
                        #endif
                        }
                        #endif
                    } else if (safeToProceed == 1) { // Escaping for rear sensors, these will move more generically forward.
                        #ifdef REAR_CENTER_SENSOR
                        if (blockedSensor[REAR_CENTER_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, MINIMUM_SPEED);
                        #ifdef REAR_3D_MOUNTED_SENSOR
                        } else if (blockedSensor[REAR_3D_MOUNTED_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, MINIMUM_SPEED);
                        #endif
                        #ifdef REAR_UPPER_DECK_SENSOR
                        } else if (blockedSensor[REAR_UPPER_DECK_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, MINIMUM_SPEED);
                        #endif
                        #ifdef REAR_NEAR_RIGHT_SENSOR
                        } else if (blockedSensor[REAR_NEAR_RIGHT_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, MINIMUM_SPEED * 2);
                        #endif
                        #ifdef REAR_NEAR_LEFT_SENSOR
                        } else if (blockedSensor[REAR_NEAR_LEFT_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED * 2, MINIMUM_SPEED);
                        #endif
                        #ifdef REAR_FAR_RIGHT_SENSOR
                        } else if (blockedSensor[REAR_FAR_RIGHT_SENSOR] == 1) {
                            drive_speed(MINIMUM_SPEED, 0);
                        #endif
                        #ifdef REAR_FAR_LEFT_SENSOR
                        } else if (blockedSensor[REAR_FAR_LEFT_SENSOR] == 1) {
                            drive_speed(0, MINIMUM_SPEED);
                        #endif
                        }
                        #endif
                    } else { // We are trapped!!
                        // Turns out we cannot escape, so turn off "Escaping",
                        // and now drive control should refuse to move forward or back,
                        // due to safeToRecede & safeToProceed both == 1,
                        // but it should be willing to rotate in place,
                        // which is a normal function of both arlobot_explore
                        // and the navigation stack's "clearing" function.
                        Escaping = 0;
                    }
                } else { // This is the "halt" but don't "escape" action for that middle ground.
                    if (Escaping == 1) {// If it WAS Escaping, stop it now.
                        drive_speed(0, 0); // return to stopped before giving control back to main thread
                    }
                    Escaping = 0; // Blocked, but not close enough to Escape yet
                }
            }

            throttleRamp = throttleRamp + 1;
            if(throttleRamp > THROTTLE_STOP)
                throttleRamp = 0;

        } else {
            /* All limits and blocks must be cleared if we are going to ignore
            proximity. Otherwise we get stuck! */
            Escaping = 0;
            safeToProceed = 1;
            safeToRecede = 1;
            abd_speedLimit = 100;
            abdR_speedLimit = 100;
        }
        pause(1); // Just throttles this cog a little.
    }
}


void SAFETY_Init()
{
    
}
void SAFETY_Start()
{
    // Start safetyOverride cog: (AFTER the Motors are initialized!)
    cogstart(&safetyOverride, NULL, safetyOverrideStack, sizeof safetyOverrideStack);
    
}

#endif