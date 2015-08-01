# Simple PID template #

This is a simple template class for PID computation first intended for the Arduino and fixed-point maths.
It features output clamping, integral anti-windup. Integration mechanism can be selected at compile time.
The currently available integration schemes are : standard, trapezoidal and Simpson.

Only automatic mode is supported for the moment.

# Example #
The following example used fixed-point PID and outputs the process value (input) in a file.
```cpp
#include <stdio.h>

// This is a dummy "clock". The time is increased by 100ms every call.
unsigned long int millis()
{
    static const unsigned long int deltaTime = 100;
    static unsigned long int clock = 0;
    unsigned long int now = clock;
    clock += deltaTime;
    return now;
}

#define PID_INTEGRATION_TRAPEZOIDAL
#include "PID.h"

#define FIXED_POINT_BIAS 256

typedef PID<int> PID_t;

int main()
{
    PID_t pid;
    int iterationCount = 100;
    
    PID_t::value_type input;
    PID_t::value_type output;
    
    FILE *plot = fopen("plot.dat", "wb");
    
    pid.Setup(16,3, 15,8, 5,4);
    pid.SetTarget(7 * FIXED_POINT_BIAS);
    pid.SetOutputLimits  (-16 * FIXED_POINT_BIAS, 16 * FIXED_POINT_BIAS);
    pid.SetIntegralLimits(-64 * FIXED_POINT_BIAS, 64 * FIXED_POINT_BIAS); 
    
    input = 0;
    pid.Start(input);
    for(int i=0; i<iterationCount; i++)
    {
        output  = pid.Update(input);
        input  += (output/2 - input) / 16;
        fprintf(plot, "%d\t\t%f\n", i, input / (float)FIXED_POINT_BIAS);
    }

    fclose(plot);
}

```
The generated file is then used to produce the following plot with Gnuplot.
![](http://blockos.org/mooz/input.png) 

# License #

All files are licensed under the Apache License version 2.0 unless otherwise marked. Full text of the Apache License version 2.0 is available in LICENSE.txt
