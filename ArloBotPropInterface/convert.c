#include "convert.h"

float CmToInch(float cm)
{
    return cm / CM_PER_INCH; 
}

float InchToCm(float inch) 
{
    return inch * CM_PER_INCH; 
}
