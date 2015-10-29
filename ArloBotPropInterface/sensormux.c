#include "arlobotconfig.h"
#include "simpletools.h"
#include "sensormux.h"

/* CD74HC4067 */

#define NUM_MUX_ADDR_PINS (4)

#define MUX_ADDR_PIN_0  (10)
#define MUX_ADDR_PIN_1  (11)
#define MUX_ADDR_PIN_2  (12)
#define MUX_ADDR_PIN_3  (13)


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
}

void SENMUX_Select(uint8_t addr)
{
    uint8_t ii;
    
    for (ii = 0; ii < NUM_MUX_ADDR_PINS; ++ii)
    {
        set_output(addr_pins[ii], mux_channel[addr][ii]);
    }
}
