#include "arlobotconfig.h"
#include "simpletools.h"
#include "sensormux.h"

/* CD74HC4067 */

#define NUM_MUX_ADDR_PINS (4)

#define MUX_ADDR_PIN_0  (6)
#define MUX_ADDR_PIN_1  (7)
#define MUX_ADDR_PIN_2  (8)
#define MUX_ADDR_PIN_3  (9)


static uint8_t addr_pins[NUM_MUX_ADDR_PINS] = {MUX_ADDR_PIN_0, MUX_ADDR_PIN_1, MUX_ADDR_PIN_2, MUX_ADDR_PIN_3};

static const uint8_t mux_channel[16][NUM_MUX_ADDR_PINS] = {
    {0,0,0,0},
    {0,0,0,1},
    {0,0,1,0},
    {0,0,1,1},
    {0,1,0,0},
    {0,1,0,1},
    {0,1,1,0},
    {0,1,1,1},
    {1,0,0,0},
    {1,0,0,1},
    {1,0,1,0},
    {1,0,1,1},
    {1,1,0,0},
    {1,1,0,1},
    {1,1,1,0},
    {1,1,1,1},
    };

void SENMUX_Init()
{
}

void SENMUX_Config()
{
    set_directions(9, 6, 0x0000000F);
    /*
    set_direction(MUX_ADDR_PIN_0, 1);
    set_direction(MUX_ADDR_PIN_1, 1);
    set_direction(MUX_ADDR_PIN_2, 1);
    set_direction(MUX_ADDR_PIN_3, 1);
    */
}

void SENMUX_Select(uint8_t addr)
{
    uint8_t ii;
    
    // To use this you need to change the mux_channel to be a value instead of an array
    //set_outputs(addr_pins[3], addr_pins[0], mux_channel[addr]);
    set_outputs(9, 6, (int) addr);
    
    /*
    for (ii = 0; ii < NUM_MUX_ADDR_PINS; ++ii)
    {
        set_output(addr_pins[ii], mux_channel[addr][ii]);
    }
    */
}
