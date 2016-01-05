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

// Validate configuration.
#if(defined(PID_INTEGRATION_SIMPSON) && defined(PID_INTEGRATION_TRAPEZOIDAL))
    #error Integration types are mutually exclusive.
#endif

// Saturate value.
#define saturate(value, minimum, maximum) \
do { \
    if (value < minimum) { value = minimum; } \
    if (value > maximum) { value = maximum; } \
} while(0);

/// Constructor.
/// @param [in] a Numerator.
/// @param [in] b Denominator.
template<typename Value_t, typename Time_t>
PID<Value_t, Time_t>::Gain::Gain(Value_t a, Value_t b)
    : n(a)
    , d(b)
{}

/// Default constructor.
template<typename Value_t, typename Time_t>
PID<Value_t, Time_t>::PID()
    : _kP         (0,1)
    , _kI         (0,1)
    , _kD         (0,1)
    , _setpoint   (0)
    , _integral   (0)
    , _outputMin  (-1024)
    , _outputMax  ( 1024)
    , _integralMin(-2048)
    , _integralMax( 2048)
    , _lastInput  (0)
    , _lastOutput (0)
    , _dt         (100)
{
    _previousError[0] = _previousError[1] = 0;
}

/// Destructor.
template<typename Value_t, typename Time_t>
PID<Value_t, Time_t>::~PID()
{}

/// Setup gains.
/// @param [in] nP Proportional gain numerator.
/// @param [in] dP Proportional gain denominator.
/// @param [in] nI Integral gain numerator.
/// @param [in] dI Integral gain denominator.
/// @param [in] nD Derivative gain numerator.
/// @param [in] dD Derivative gain denominato.
/// @param [in] dt Sampling time (default: 100ms).
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::Setup(Value_t nP, Value_t dP,
                                 Value_t nI, Value_t dI,
                                 Value_t nD, Value_t dD,
                                 Time_t  dt)
{
    // Premultiply gains by sampling time.
    _kP = Gain(   dt*nP, dP);
    _kI = Gain(dt*dt*nI, dI);
    _kD = Gain(      nD, dD);
    _dt = dt;
    _integral = 0;
    _previousError[0] = _previousError[1] = 0;
}
/// Set minimum and maximum output value.
/// @param [in] outMin Minimum output value.
/// @param [in] outMax Maximum output value.
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::SetOutputLimits(Value_t outMin, Value_t outMax)
{
    _outputMin = outMin;
    _outputMax = outMax;
    saturate(_lastOutput, _outputMin, _outputMax);
}
/// Set integral limits.
/// @param [in] intMin Minimum integral value.
/// @param [in] intMax Maximum integral value.
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::SetIntegralLimits(Value_t intMin, Value_t intMax)
{
    // Premultiply by dt squared.
    Time_t dt2 = _dt*_dt;
    _integralMin = intMin * dt2;
    _integralMax = intMax * dt2;
    saturate(_integral, _integralMin, _integralMax);
}
/// Set setpoint.
/// @param [in] s Setpoint.
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::SetSetpoint(Value_t const& s)
{
    _setpoint = s;
    _integral = 0;
}
/// Start PID controller.
/// @param [in] input Start value.
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::Start(Value_t input)
{
    _lastInput = input;
    _integral  = _lastOutput;
    _previousError[0] = _previousError[1] = 0;
}
/// Update output based on the current state values and setpoint.
/// The output is clamped against the previously specified
/// output limits. Integral anti-windup is performed only if the
/// output is saturated.
/// @param [in] input Measured value.
/// @return Updated output value.
template<typename Value_t, typename Time_t>
Value_t PID<Value_t, Time_t>::Update(Value_t input)
{
    // Error between input value and setpoint.
    Value_t error = _setpoint - input;

    // Perform PID computation.
    Value_t proportional, derivative;
    proportional = _kP.n * error / _kP.d;
    derivative   = _kD.n * (input - _lastInput) * 1000 / _kD.d;
    _lastInput = input;
    
    Value_t sum, div;
#if(defined(PID_INTEGRATION_SIMPSON))
    // Simpson rule.
    sum = _previousError[1] + 4*_previousError[0] + error;
    div = 6000;
    _previousError[1] = _previousError[0];
    _previousError[0] = error;
#elif(defined(PID_INTEGRATION_TRAPEZOIDAL))
    // Trapezoidal integration.
    sum = error + _previousError[0];
    div = 2000;
    _previousError[0] = error;
#else
    // Standard integration.
    sum = error;
    div = 1000;
#endif
    _integral += (_kI.n * sum) / (div * _kI.d);
    saturate(_integral, _integralMin, _integralMax);
    
    // Compute output.
    _lastOutput = (proportional + _integral - derivative) / _dt;
    saturate(_lastOutput, _outputMin, _outputMax);

    return _lastOutput;
}
/// Stop PID controller.
template <typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::Stop()
{
    // Nothing atm.
}
/// Get proportional gain.
template <typename Value_t, typename Time_t>
typename PID<Value_t, Time_t>::Gain const& PID<Value_t, Time_t>::ProportionalGain() const
{
    return Gain(_kP.n/_dt, _kP.d);
}
/// Get proportional gain.
template <typename Value_t, typename Time_t>
typename PID<Value_t, Time_t>::Gain const& PID<Value_t, Time_t>::IntegralGain() const
{
    return Gain(_kI.n/(_dt*_dt), _kI.d);
}
/// Get proportional gain.
template <typename Value_t, typename Time_t>
typename PID<Value_t, Time_t>::Gain const& PID<Value_t, Time_t>::DerivativeGain() const
{
    return _kD;
}
/// Get last output.
template <typename Value_t, typename Time_t>
Value_t const& PID<Value_t, Time_t>::LastOutput() const
{
    return _lastOutput;
}
/// Get minimum output limit.
template <typename Value_t, typename Time_t>
Value_t const& PID<Value_t, Time_t>::GetMinOutputLimit() const
{
    return _outputMin;
}
/// Get maximum output limit.
template <typename Value_t, typename Time_t>
Value_t const& PID<Value_t, Time_t>::GetMaxOutputLimit() const
{
    return _outputMax;
}
/// Get minimum integral limit.
template <typename Value_t, typename Time_t>
Value_t const& PID<Value_t, Time_t>::GetMinIntegralLimit() const
{
    return _integralMin;
}
/// Get maximum integral limit.
template <typename Value_t, typename Time_t>
Value_t const& PID<Value_t, Time_t>::GetMaxIntegralLimit() const
{
    return _integralMax;
}
/// Get setpoint.
template <typename Value_t, typename Time_t>
Value_t const& PID<Value_t, Time_t>::GetSetpoint() const
{
    return _setpoint;
}
/// Get sampling time.
template <typename Value_t, typename Time_t>
Time_t const& PID<Value_t, Time_t>::GetSamplingTime() const
{
    return _dt;
}
