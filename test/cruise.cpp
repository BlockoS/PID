// Simple PID template.
//
// Copyright (c) 2015 - Vincent Cruz.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Cruise control test.
// Build:
//   g++ -Wall -std=c++11 -I .. cruise.cpp
// Run:
//   ./a.out > plot.dat
// Plot (gnuplot):
//   set terminal "pngcairo"
//   set output "pid.png"
//   set yrange [0:12]
//   plot "plot.dat" using 2 title "speed" with lines, "plot.dat" using 3 title "setpoint" with lines
//
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
