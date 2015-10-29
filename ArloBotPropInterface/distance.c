#include "distance.h"


#ifdef hasQuickStartBoard
// For Quickstart Board communication
fdserial *propterm;
void pollPropBoard2(void *par); // Use a cog to fill range variables with ping distances
static int prop2stack[128]; // If things get weird make this number bigger!
#else
// Local Sensor Polling Cog
void pollPingSensors(void *par); // Use a cog to fill range variables with ping distances
static int pstack[128]; // If things get weird make this number bigger!
#endif


#ifdef hasQuickStartBoard
void pollPropBoard2(void *par) {
    propterm = fdserial_open(QUICKSTART_RX_PIN, QUICKSTART_TX_PIN, 0, 115200);
    pause(100); // Give the serial connection time to come up. Perhaps this is not required?
    const int bufferLength = 10; // Longer than longest possible received line
    char buf[bufferLength];
    int count = 0, pingSensorNumber = 0, irSensorNumber = 0;
    int rateLimit = 10; // This is the incoming rate limiter. Without some limit the entire Propeller will hang.
    while (1) {
        pause(rateLimit);
        // Tell the other end we are alive, so it doesn't just spin pointlessly.
        // It also keeps the sensors quiet when this end is in an idle state.
        dprint(propterm, "i");
        //Do not put any delay here.
        if (fdserial_rxReady(propterm) != 0) {
            //high(26); // LEDs for debugging
            count = 0;
            while (count < bufferLength) {
                buf[count] = readChar(propterm);
                if (buf[count] == '.') // Using . for end of line instead of line break
                    break;
                count++;
            }
            // For Debugging - Test for failing lines
            /* if (buf[0] != 'p' && buf[0] != 'i')
               dprint(term, "%c\n", buf[0]); */

            if (buf[0] == 'p') {
                char *token;
                token = strtok(buf, delimiter);
                token = strtok(NULL, delimiter);
                char *unconverted;
                pingSensorNumber = strtod(token, &unconverted);
                token = strtok(NULL, delimiter);
                if (pingSensorNumber < NUMBER_OF_PING_SENSORS) {
                    pingArray[pingSensorNumber] = strtod(token, &unconverted);
                    // For Debugging:
                    /* dprint(term, "p%d:%3d ", pingSensorNumber, pingArray[pingSensorNumber]);
                       if(pingSensorNumber == 9)
                       dprint(term, "\n"); */
                   }
           } else if (buf[0] == 'i') {
            char *token;
            token = strtok(buf, delimiter);
            token = strtok(NULL, delimiter);
            char *unconverted;
            irSensorNumber = strtod(token, &unconverted);
            token = strtok(NULL, delimiter);
            if (irSensorNumber < NUMBER_OF_IR_SENSORS) {
                irArray[irSensorNumber] = strtod(token, &unconverted);
                    // For Debugging:
                    //dprint(term, "i%d:%3d ", irSensorNumber, irArray[irSensorNumber]);
            }
        }
            //low(26); // LEDs for debugging
    }
    #ifdef hasFloorObstacleSensors
    for (int i = 0; i < NUMBER_OF_FLOOR_SENSORS; i++) {
        floorArray[i] = input(FIRST_FLOOR_SENSOR_PIN + i);
    }
    #endif
}
}
#else
static void pollPingSensors(void *par)
{
    while (1)
    {
        US_Ping();
        IR_Ping();
        US_Distance(us_distances);
        IR_Distance(ir_distances);
    }
}    
#endif

DS_Init()
{
    
}

DS_Start()
{
#ifdef hasQuickStartBoard
// Start 2nd Propeller Board Communication cog
    cogstart(&pollPropBoard2, NULL, prop2stack, sizeof prop2stack);
#else
// Start the local sensor polling cog
    cogstart(&pollPingSensors, NULL, pstack, sizeof pstack);
#endif
}

