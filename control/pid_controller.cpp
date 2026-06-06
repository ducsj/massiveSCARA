/**
 * =============================================================================
 * This is file 4/12 – PID Controller module.
 * =============================================================================
 * 
 * Purpose: Implementation of the PIDController class.
 * 
 * This implements a discrete-time PID control loop commonly used in robotics
 * and industrial control systems. The controller continuously adjusts its
 * output to minimize the error between a measured value and a target value.
 * 
 * Key concepts:
 * - Proportional (P): Reacts to current error - "how far off are we now?"
 * - Integral (I): Reacts to accumulated past errors - "have we been off for a while?"
 * - Derivative (D): Reacts to rate of change - "is error getting bigger or smaller?"
 * 
 * =============================================================================
 */

#include "pid_controller.h"       // Include the header file for this class

/**
 * Constructor - initialize PID gains and state to safe defaults.
 */
PIDController::PIDController() {
    // Initialize PID gains to zero (no control output until gains are set)
    _kp = 0.0f;    // Proportional gain
    _ki = 0.0f;    // Integral gain
    _kd = 0.0f;    // Derivative gain
    
    // Initialize target to zero
    _target = 0.0f;
    
    // Initialize previous input to zero (for derivative calculation)
    _lastInput = 0.0f;
    
    // Initialize integral to zero (no accumulated error yet)
    _integral = 0.0f;
    
    // Initialize last error to zero
    _lastError = 0.0f;
    
    // Initialize time to 0 (will be set on first update)
    _lastTime = 0;
    
    // Default output limits: full range of float
    // These should be changed via setOutputLimits() for your application
    _outputMin = -1000.0f;
    _outputMax = 1000.0f;
    
    // Default sample time: 10 milliseconds (100 Hz update rate)
    _sampleTimeMs = 10;
}

/**
 * Set the PID tuning parameters (the gains).
 * 
 * Tuning tips:
 * - Start with Kp only, set Ki and Kd to 0
 * - Increase Kp until the system oscillates around the target
 * - Reduce Kp slightly to stop the oscillation
 * - Then add small amounts of Ki to eliminate steady-state error
 * - Finally, add Kd to dampen any remaining oscillations
 * 
 * @param kp  Proportional gain - reacts to current error
 * @param ki  Integral gain - reacts to accumulated past errors
 * @param kd  Derivative gain - reacts to rate of error change
 */
void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;   // Store the proportional gain
    _ki = ki;   // Store the integral gain
    _kd = kd;   // Store the derivative gain
}

/**
 * Set the target position for the PID controller.
 * 
 * @param targetAngle  The desired value we want to achieve (e.g., 45 degrees)
 */
void PIDController::setTarget(float targetAngle) {
    _target = targetAngle;   // Store the target value
}

/**
 * Set limits on the PID output to prevent damage or unsafe behavior.
 * 
 * For a motor, this might be the maximum speed in steps per second.
 * For a heater, this might be the maximum power percentage.
 * 
 * @param min  Minimum allowed output value
 * @param max  Maximum allowed output value
 */
void PIDController::setOutputLimits(float min, float max) {
    // Store the minimum and maximum output limits
    _outputMin = min;
    _outputMax = max;
}

/**
 * Set how often update() will be called.
 * 
 * This affects the I and D term calculations, which depend on the
 * time between updates (dt). Use the same value every time you call update().
 * 
 * @param sampleTimeMs  Time between update() calls in milliseconds
 */
void PIDController::setSampleTime(unsigned long sampleTimeMs) {
    // Store the sample time
    _sampleTimeMs = sampleTimeMs;
}

/**
 * Constrain a value to stay within specified bounds.
 * If value is below min, return min. If above max, return max. Otherwise return value.
 * 
 * @param value  The value to constrain
 * @param min    Lower bound
 * @param max    Upper bound
 * @return The constrained value
 */
float PIDController::_constrain(float value, float min, float max) {
    // If value is less than minimum, return minimum
    if (value < min) {
        return min;
    }
    // If value is greater than maximum, return maximum
    if (value > max) {
        return max;
    }
    // Otherwise value is within bounds, return it unchanged
    return value;
}

/**
 * Reset the PID controller state.
 * 
 * Call this when:
 * - Starting a new move to a different target
 * - Stopping and restarting the controller
 * - After an error condition
 * 
 * This clears the integral term so we don't have leftover accumulated error
 * from previous operations.
 */
void PIDController::reset() {
    // Clear the accumulated integral term
    // Without this, the I term would still be large from previous operations
    _integral = 0.0f;
    
    // Clear the last error
    _lastError = 0.0f;
    
    // Reset the time tracker (next update will be treated as first update)
    _lastTime = 0;
}

/**
 * Run one step of the PID control loop.
 * 
 * Call this function at regular intervals (every sampleTimeMs) with the
 * current measured value. It calculates the control output needed to drive
 * the system toward the target.
 * 
 * The PID formula:
 *   error = target - current
 *   P = Kp * error
 *   I = Ki * Σ(error * dt)  [with anti-windup clamping]
 *   D = Kd * (error - lastError) / dt
 *   output = P + I + D
 * 
 * @param currentAngle  The current measured value (e.g., encoder reading in degrees)
 * @return The control output, clamped to the configured limits
 */
float PIDController::update(float currentAngle) {
    // Get the current time in milliseconds
    unsigned long now = millis();
    
    // Calculate time since last update (dt) in seconds
    // This is crucial for proper scaling of I and D terms
    unsigned long dt = now - _lastTime;
    
    // If this is the first update or if enough time has passed for a new sample
    if (dt >= _sampleTimeMs) {
        // Calculate the error: how far are we from the target?
        // Positive = we're below target, need to increase output
        // Negative = we're above target, need to decrease output
        float error = _target - currentAngle;
        
        // ---- PROPORTIONAL TERM (P) ----
        // The P term reacts to current error. Bigger error = bigger response.
        float p = _kp * error;
        
        // ---- INTEGRAL TERM (I) with anti-windup ----
        // The I term accumulates past errors to eliminate steady-state error.
        // We add the new error (scaled by dt) to the accumulated integral.
        _integral += error * (dt / 1000.0f);  // dt is in ms, convert to seconds
        
        // ANTI-WINDUP CLAMPING:
        // The integral term can "wind up" to huge values if the error persists
        // for a long time. This causes overshoot and oscillation when we finally
        // reach the target. To prevent this, we clamp the integral term so it
        // can't push the output beyond the limits.
        
        // First, calculate what P term would be at zero error
        // This tells us how much "room" the P term has left to work with
        float pAtZeroError = 0.0f;  // P would be 0 if error was 0
        
        // Calculate where the I term would need to be to hit the output limits
        // We want: P + I = limit, so I = limit - P
        float integralMax = (_outputMax - p) - pAtZeroError;
        float integralMin = (_outputMin - p) - pAtZeroError;
        
        // Actually, a simpler approach: calculate output range available for I
        // output = P + I + D, so I = output - P - D
        // At the limits: I_limited = limit - P (ignoring D for simplicity)
        // So we clamp the integral term itself
        float outputWithoutI = p;  // P term only (D term is calculated below)
        
        // Calculate the integral limits based on output limits
        // This ensures P + I stays within bounds
        float integralLimitMax = _outputMax - outputWithoutI;
        float integralLimitMin = _outputMin - outputWithoutI;
        
        // Clamp the integral term to prevent windup
        // The integral is divided by ki to get the "input" to the integral term
        // We scale by ki to get the actual contribution to output
        if (_ki != 0.0f) {
            // Clamp the integral accumulator
            // Note: integral * ki = I contribution, so integral = I / ki
            float iContribution = _ki * _integral;
            
            // If I would push us over the max, clamp the integral
            if (iContribution > _outputMax - p) {
                _integral = (_outputMax - p) / _ki;
            }
            // If I would push us under the min, clamp the integral
            if (iContribution < _outputMin - p) {
                _integral = (_outputMin - p) / _ki;
            }
        }
        
        // Calculate the integral contribution to output
        float i = _ki * _integral;
        
        // ---- DERIVATIVE TERM (D) ----
        // The D term reacts to the rate of change of error.
        // It helps dampen oscillations and predict future error.
        float d = 0.0f;
        
        // Only calculate D if we have a valid previous error
        // and enough time has passed (avoid division by zero or noise)
        if (dt > 0) {
            // Calculate rate of change of error
            float errorRate = (error - _lastError) / (dt / 1000.0f);
            d = _kd * errorRate;
        }
        
        // ---- CALCULATE TOTAL OUTPUT ----
        // Sum all three terms: P + I + D
        float output = p + i + d;
        
        // ---- APPLY OUTPUT LIMITS ----
        // Ensure output stays within configured bounds
        output = _constrain(output, _outputMin, _outputMax);
        
        // ---- UPDATE STATE FOR NEXT ITERATION ----
        // Store current values for next calculation
        _lastError = error;        // Save current error for next D calculation
        _lastInput = currentAngle; // Save current input for next D calculation
        _lastTime = now;           // Save current time for next dt calculation
        
        return output;  // Return the calculated control output
    }
    
    // If not enough time has passed, return 0 (or you could return the last output)
    // This prevents the controller from running too fast
    return 0.0f;
}