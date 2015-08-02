# Simple PID template #

This is a simple template class for PID computation first intended for the Arduino and fixed-point maths.
It features output clamping, integral anti-windup. Integration mechanism can be selected at compile time.
The currently available integration schemes are : standard, trapezoidal and Simpson.

Only automatic mode is supported for the moment.

# Example #
The following example uses fixed-point PID and outputs the setpoint and process value (speed) in a file.
```cpp
// file: test/cruise.cpp
#include <cstdint>
#include <iostream>
#include "PID.h"

// Fixed point bias.
#define BIAS 256

typedef PID<int64_t, int64_t> PID_t;

int main()
{
    PID_t control;
    
    // Vehicle mass.
    PID_t::value_type mass      = 1000;
    // Damping factor.
    PID_t::value_type damping   = 50;
    // Target speed.
    PID_t::value_type reference = 10;
    // Current vehicle speed.
    PID_t::value_type speed     = 0;
    // PID output.
    PID_t::value_type output    = 0;
    
    control.Setup(720,1, 40,1, 10,1, 100);
    control.SetSetpoint(reference * BIAS);
    control.SetOutputLimits  (      0*BIAS, 32768*BIAS);
    control.SetIntegralLimits( -32768*BIAS, 32768*BIAS);
    
    control.Start(speed);
    
    for(int i=0; i<120; i++)
    {
        // Compute PID.
        output = control.Update(speed);
        // Compute speed.
        speed += (output - damping*speed) * control.GetSamplingTime() / mass / 1000;
    
        // Output speed and setpoint.
        std::cout << i << '\t' << (speed / (float)BIAS) << '\t';
        std::cout << (control.GetSetpoint() / (float)BIAS) << std::endl;
    }

    return 0;
}
```
The generated file is then used to produce the following plot with Gnuplot.
![](http://blockos.org/mooz/input.png) 

# License #

All files are licensed under the Apache License version 2.0 unless otherwise marked. Full text of the Apache License version 2.0 is available in LICENSE.txt
