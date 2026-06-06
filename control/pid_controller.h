/**
 * =============================================================================
 * This is file 4/12 – PID Controller module.
 * =============================================================================
 * 
 * Purpose: Header file for the PIDController class that implements a
 * Proportional-Integral-Derivative control loop.
 * 
 * A PID controller constantly adjusts its output to make a system (like a
 * motor) reach and maintain a target value (like a specific angle). It does
 * this by calculating three terms based on how far off target we are:
 *   - P (Proportional): reacts to current error
 *   - I (Integral): reacts to accumulated past errors
 *   - D (Derivative): reacts to rate of error change
 * 
 * This is the classic algorithm used in countless control systems!
 * 
 * =============================================================================
 */

#ifndef PID_CONTROLLER_H         // Prevent this file from being included multiple times
#define PID_CONTROLLER_H          // Define the guard symbol so #ifndef knows we've loaded this file

// Include Arduino framework for unsigned long type
#include <Arduino.h>

/**
 * PIDController class
 * 
 * Implements a discrete-time PID control loop.
 * 
 * How it works:
 * 1. We have a target value (e.g., 45 degrees)
 * 2. We measure the current value (e.g., motor encoder reading)
 * 3. We calculate the error (target - current)
 * 4. We compute P, I, and D terms
 * 5. We sum them to get the output (e.g., motor speed command)
 * 6. Repeat at regular intervals
 * 
 * The gains (Kp, Ki, Kd) control how aggressively each term affects the output.
 */
class PIDController {
public:
    /**
     * Constructor - initializes PID gains to safe defaults.
     */
    PIDController();

    /**
     * Set the PID tuning parameters (the gains).
     * 
     * These values control how aggressively the controller responds:
     * - Kp (Proportional): Higher = faster response but may overshoot/oscillate
     * - Ki (Integral): Higher = eliminates steady-state error but may cause oscillation
     * - Kd (Derivative): Higher = dampens oscillations but may amplify noise
     * 
     * @param kp  Proportional gain
     * @param ki  Integral gain
     * @param kd  Derivative gain
     */
    void setGains(float kp, float ki, float kd);

    /**
     * Set the target position for the PID controller to aim for.
     * 
     * @param targetAngle  Target value in degrees (or whatever units you're using)
     */
    void setTarget(float targetAngle);

    /**
     * Set limits on the PID output value.
     * 
     * This prevents the output from going too high or too low, which could
     * damage motors or cause unsafe behavior. For example, you might limit
     * the output to -200 to +200 steps per second.
     * 
     * @param min  Minimum output value (e.g., -200)
     * @param max  Maximum output value (e.g., 200)
     */
    void setOutputLimits(float min, float max);

    /**
     * Set how often update() should be called for proper PID calculation.
     * 
     * The PID formula needs to know the time between updates (dt) to
     * calculate the I and D terms correctly. Call this once in setup()
     * to tell the controller your loop timing.
     * 
     * @param sampleTimeMs  Time between update() calls in milliseconds (default: 10)
     */
    void setSampleTime(unsigned long sampleTimeMs);

    /**
     * Run one PID calculation step.
     * 
     * Call this at regular intervals (every sampleTimeMs) with the current
     * measured value. Returns the control output that should be sent to
     * the motor.
     * 
     * The PID formula:
     *   error = target - current
     *   P = Kp * error
     *   I = Ki * Σ(error * dt)
     *   D = Kd * (error - lastError) / dt
     *   output = P + I + D
     * 
     * @param currentAngle  The current measured value (e.g., encoder reading in degrees)
     * @return The control output value, clamped to the set limits
     */
    float update(float currentAngle);

    /**
     * Reset the PID controller state.
     * 
     * Clears the integral term and last error value. Call this when:
     * - Switching to a new target
     * - Starting/stopping the controller
     * - Recovering from an error condition
     */
    void reset();

private:
    // PID tuning parameters (gains)
    float _kp;   // Proportional gain - reacts to current error
    float _ki;   // Integral gain - reacts to accumulated past errors
    float _kd;   // Derivative gain - reacts to rate of error change

    // Target and current values
    float _target;        // The desired value we're trying to reach
    float _lastInput;     // Previous input value (for derivative calculation)

    // PID state variables
    float _integral;      // Accumulated integral term (sum of errors over time)
    float _lastError;     // Previous error value (for derivative calculation)
    unsigned long _lastTime;  // Timestamp of last update() call (for dt calculation)

    // Output limits
    float _outputMin;     // Minimum allowed output value
    float _outputMax;     // Maximum allowed output value

    // Sample time
    unsigned long _sampleTimeMs;  // Desired time between updates in milliseconds

    /**
     * Constrain a value to stay within min and max bounds.
     * This prevents output from going outside the set limits.
     * 
     * @param value  The value to constrain
     * @param min    Minimum allowed value
     * @param max    Maximum allowed value
     * @return The constrained value
     */
    float _constrain(float value, float min, float max);
};

#endif  // PID_CONTROLLER_H       // End of the #ifndef guard block