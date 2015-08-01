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
    , _target     (0)
    , _integral   (0)
    , _outputMin  (-1024)
    , _outputMax  ( 1024)
    , _integralMin(-2048)
    , _integralMax( 2048)
    , _lastInput  (0)
    , _lastOutput (0)
    , _lastTime   (0)
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
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::Setup(Value_t nP, Value_t dP,
                                 Value_t nI, Value_t dI,
                                 Value_t nD, Value_t dD)
{
    _kP = Gain(nP, dP);
    _kI = Gain(nI, dI);
    _kD = Gain(nD, dD);
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
    if(_lastOutput < _outputMin) { _lastOutput = _outputMin; }
    else if(_lastOutput > _outputMax) { _lastOutput = _outputMax; }
}
/// Set integral limits.
/// @param [in] intMin Minimum integral value.
/// @param [in] intMax Maximum integral value.
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::SetIntegralLimits(Value_t intMin, Value_t intMax)
{
    _integralMin = intMin;
    _integralMax = intMax;
    if(_integral < _integralMin) { _integral = _integralMin; }
    else if(_integral > _integralMax) { _integral = _integralMax; }
}
/// Set target (setpoint).
/// @param [in] t Target value (setpoint).
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::SetTarget(Value_t const& t)
{
    _target   = t;
    _integral = 0;
}
/// Start PID controller.
/// @param [in] input Start value.
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::Start(Value_t input)
{
    _lastInput = input;
    _lastTime  = millis();
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
    // Elapsed time since last update.
    Value_t now = static_cast<Value_t>(millis());
    Value_t dt  = now - _lastTime;
    // Error between input value and target value.
    Value_t error = _target - input;

    // Perform PID computation.
    Value_t proportional, derivative, currentIntegral;
    proportional = _kP.n * error * dt / _kP.d;
    derivative   = _kD.n * (input - _lastInput) * 1000 / _kD.d;
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
    currentIntegral = _integral + (_kI.n * sum * dt * dt) / (div * _kI.d);
    if(currentIntegral < _integralMin) { currentIntegral = _integralMin; }
    else if(currentIntegral > _integralMax) { currentIntegral = _integralMax; }
    
    // Compute output.
    _lastOutput = (proportional + currentIntegral - derivative) / dt;

    // Clamp output value and perform integral anti-windup computation.
    if(_lastOutput < _outputMin)
    {
        _lastOutput = _outputMin;
        if(_integral > currentIntegral) { _integral = currentIntegral; }
    }
    else if(_lastOutput > _outputMax)
    {
        _lastOutput = _outputMax;
        if(_integral < currentIntegral) { _integral = currentIntegral; }
    }
    else
    {
        _integral = currentIntegral;
    }
    
    _lastInput = input;
    _lastTime  = now;
    
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
    return _kP;
}
/// Get proportional gain.
template <typename Value_t, typename Time_t>
typename PID<Value_t, Time_t>::Gain const& PID<Value_t, Time_t>::IntegralGain() const
{
    return _kI;
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
