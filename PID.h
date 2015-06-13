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

#ifndef PID_H
#define PID_H

/// Simple PID controller template.
/// This class is intended to be used under the Arduino environement.
/// Neverless if you want to use elsewhere, you must provide an
/// an implementation of the @b millis() function. This function
/// returns the number of millisecondes elapsed since the beginning of
/// operations.
///
/// The template parameters are:
///     * The value type. It can be a @b float or an @b long int. Keep in
///       mind that this type must be signed, otherwise the output value
///       may be completly broken.
///     * The time type. It is basically the time of the value returned
///       by @b millis().
/// @code
///     #include "PID.h"
///     
///     typedef PID<int> PID_t;
///     
///     int main()
///     {
///         PID_t pid;
///         PID_t::value_type input;
///         
///         PID_t::value_type output;
///         
///         pid.Setup(12,5, 3,100, 1,1);
///         pid.SetTarget(10 * 256);
///         pid.SetOutputBounds(-32 * 256, 32 * 256);
///         
///         input = sensor.Read();
///         pid.Start(input);
///         
///         while(1)
///         {
///             input = sensor.Read();
///             output = pid.Update(input);
///             device.doSomething(output);
///         }
/// @endcode
template<typename Value_t, typename Time_t = unsigned long int>
class PID
{
    public:
        /// Fractional gain.
        struct Gain
        {
            /// Numerator.
            Value_t n;
            /// Denominator.
            Value_t d;
            /// Constructor.
            /// @param [in] a Numerator.
            /// @param [in] b Denominator.
            Gain(Value_t a, Value_t b);
        };
        
        typedef Value_t value_type;
        typedef Time_t  time_type;
        
    public:
        /// Default constructor.
        PID();
        /// Destructor.
        ~PID();
        /// Setup gains.
        /// @param [in] nP Proportional gain numerator.
        /// @param [in] dP Proportional gain denominator.
        /// @param [in] nI Integral gain numerator.
        /// @param [in] dI Integral gain denominator.
        /// @param [in] nD Derivative gain numerator.
        /// @param [in] dD Derivative gain denominato.
        void Setup(Value_t nP, Value_t dP,
                   Value_t nI, Value_t dI,
                   Value_t nD, Value_t dD);
        /// Set target (setpoint).
        /// @param [in] t Target value (setpoint).
        void SetTarget(Value_t const& t);
        /// Set minimum and maximum output value.
        /// @param [in] outMin Minimum output value.
        /// @param [in] outMax Maximum output value.
        void SetOutputBounds(Value_t outMin, Value_t outMax);
        /// Start PID controller.
        /// @param [in] input Start value.
        void Start(Value_t input);
        /// Update output based on the current state values and setpoint.
        /// The output is clamped against the previously specified
        /// output bounds. Integral anti-windup is performed only if the
        /// output is saturated.
        /// @param [in] input Measured value.
        /// @return Updated output value.
        Value_t Update(Value_t input);
        /// Stop PID controller.
        void Stop();
        /// Get proportional gain.
        Gain const& ProportionalGain() const;
        /// Get proportional gain.
        Gain const& IntegralGain() const;
        /// Get proportional gain.
        Gain const& DerivativeGain() const;
        /// Get last output.
        Value_t const& LastOutput() const;

    private:
        /// Proportional gain.
        Gain _kP;
        /// Inegral gain.
        Gain _kI;
        /// Derivative gain.
        Gain _kD;
        /// Target value (setpoint)
        Value_t _target;
        /// Integral term.
        Value_t _integral;
        /// Minimum output value.
        Value_t _min;
        /// Maximum output value.
        Value_t _max;
        /// Last update input value.
        Value_t _lastInput;
        /// Last error.
        Value_t _lastError;
        /// Last output value.
        Value_t _lastOutput;
        /// Last time update was run.
        Value_t _lastTime;
};

#include "PID.inl"

#endif // PID_H
