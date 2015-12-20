/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools
#include "fdserial.h"
#include "hwconfig.h"
#include "utils.h"

static uint32_t last_time;
static uint8_t ii;
#define MAX_SENSORS (7)
#define START (0)
static float dist[MAX_SENSORS];
static uint32_t counter;

static float ReadUltrasonicDist(uint8_t addr)
{
    set_outputs(MUX_ADDR_END, MUX_ADDR_START, (int) addr); 
    low(TRIG_PIN);
    pulse_out(TRIG_PIN, 10);
    uint32_t pulse = pulse_in(ECHO_PIN, 1);
    //dprint(msg_serial, "%d:%f\n", pulse, pulse/58.0);
    return pulse/58.0;    
}    


int main()                                    // Main function
{
  uint32_t start_time;
  uint32_t end_time;

  // Add startup code here.
  set_direction(MUX_ADDR_START, 1);
  set_direction(MUX_ADDR_START + 1, 1);
  set_direction(MUX_ADDR_START+ 2, 1);
  set_direction(MUX_ADDR_START + 3, 1);

  last_time = millis();
  
  while(1)
  {
    
    for (ii = START; ii < MAX_SENSORS; ++ii)
    {
        start_time = millis();
        dist[ii] = ReadUltrasonicDist(ii);
        //print("time: %d:%d\n", millis() - start_time, ii);
    }
    
    uint32_t diff = millis() - last_time;
    if (diff > 1000)
    {
        print("%d:%d:", diff, counter++);
        last_time = millis();
        for (ii = START; ii < MAX_SENSORS - 1; ++ii)
        {
            print("%7.1f:", dist[ii]);
        }            
        print("%7.1f\n", dist[ii]);
    }
    
  }  
}
