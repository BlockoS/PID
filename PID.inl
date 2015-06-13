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
    : _kP        (0,1)
    , _kI        (0,1)
    , _kD        (0,1)
    , _target    (0)
    , _integral  (0)
    , _min       (-1024)
    , _max       ( 1024)
    , _lastInput (0)
    , _lastError (0)
    , _lastOutput(0)
    , _lastTime  (0)
{}

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
}
/// Set minimum and maximum output value.
/// @param [in] outMin Minimum output value.
/// @param [in] outMax Maximum output value.
template<typename Value_t, typename Time_t>
void PID<Value_t, Time_t>::SetOutputBounds(Value_t outMin, Value_t outMax)
{
    _min = outMin;
    _max = outMax;
    if(_lastOutput < _min) { _lastOutput = _min; }
    else if(_lastOutput > _max) { _lastOutput = _max; }
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
    _lastError = 0;
}
/// Update output based on the current state values and setpoint.
/// The output is clamped against the previously specified
/// output bounds. Integral anti-windup is performed only if the
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
    Value_t proportional, derivative;
    proportional = _kP.n * error * dt / _kP.d;
    derivative   = _kD.n * (input - _lastInput) * 1000 / _kD.d;
    // Trapezoidal integration.
    Value_t currentIntegral = _integral + (_kI.n * (error + _lastError) * dt * dt / 2000 / _kI.d);
    
    // Compute output.
    _lastOutput = (proportional + currentIntegral - derivative) / dt;

    // Clamp output value and perform integral anti-windup computation.
    if(_lastOutput < _min)
    {
        _lastOutput = _min;
        if(_integral > currentIntegral) { _integral = currentIntegral; }
    }
    else if(_lastOutput > _max)
    {
        _lastOutput = _max;
        if(_integral < currentIntegral) { _integral = currentIntegral; }
    }
    else
    {
        _integral = currentIntegral;
    }
    
    _lastInput = input;
    _lastTime  = now;
    _lastError = error;
    
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
