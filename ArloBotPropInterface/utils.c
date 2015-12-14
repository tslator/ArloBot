#include "utils.h"

uint32_t millis()
{
  return (CNT / (CLKFREQ / 1000));
}