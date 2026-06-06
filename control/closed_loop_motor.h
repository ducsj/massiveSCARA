/**
 * =============================================================================
 * This is file 5/12 – Closed-Loop Motor module.
 * =============================================================================
 * 
 * Purpose: Header file for the ClosedLoopMotor class that combines motor
 * control, encoder feedback, and PID control into a single unit.
 * 
 * This class implements closed-loop motor control where:
 * 1. We read the current angle from an encoder
 * 2. We calculate the error (how far from target)
 * 3. We use a PID controller to compute the right speed
 * 4. We step the motor at that speed until we reach the target
 * 
 * This is different from open-loop control (which just blindly steps)
 * because we use feedback to know if we're actually reaching the target.
 * 
 * Hardware: Motor + Encoder + PID Controller
 * 
 * =============================================================================
 */

#ifndef CLOSED_LOOP_MOTOR_H       // Prevent this file from being included multiple times
#define CLOSED_LOOP_MOTOR_H       // Define the guard symbol so #ifndef knows we've loaded this file

// Include Arduino framework for timing functions and types
#include <Arduino.h>

// Include our configuration for motor/encoder settings
#include "../config/robot_config.h"

// Forward declarations to avoid circular includes
class EncoderDriver;
class PIDController;

/**
 * ClosedLoopMotor class
 * 
 * Combines motor control, encoder feedback, and PID control for precise
 * position control of a single motor axis.
 * 
 * How it works:
 * 1. setTargetAngle() sets the desired angle
 * 2. update() is called repeatedly in the main loop
 * 3. update() reads the encoder to get current position
 * 4. update() runs the PID to calculate motor speed
 * 5. update() steps the motor at that speed
 * 6. When error is small enough, motor stops
 * 
 * This approach uses direct GPIO control for stepping (not MotorDriver's
 * moveTo system) because we want continuous speed control based on PID output.
 */
class ClosedLoopMotor {
public:
    /**
     * Constructor - initializes all members to safe defaults.
     */
    ClosedLoopMotor();

    /**
     * Initialize the closed-loop motor controller.
     * 
     * @param motorId      Which motor: 0 = left arm, 1 = right arm
     * @param stepPin      GPIO pin for step pulses
     * @param dirPin       GPIO pin for direction
     * @param rpm          Initial motor speed in RPM (default: 15)
     * @param encoderPtr   Pointer to the global EncoderDriver instance
     */
    void begin(int motorId, int stepPin, int dirPin, int rpm = 15, 
               EncoderDriver* encoderPtr = nullptr);

    /**
     * Set the target angle for this motor to move to.
     * 
     * @param angleDegrees  Target angle in degrees
     */
    void setTargetAngle(float angleDegrees);

    /**
     * Get the current angle from the encoder.
     * 
     * @return Current angle in degrees (calibrated)
     */
    float getCurrentAngle();

    /**
     * Update the motor - call this frequently in your main loop.
     * 
     * This is the main control loop. It:
     * 1. Reads the current angle from encoder
     * 2. Runs the PID to calculate motor speed
     * 3. Steps the motor at the calculated speed
     * 
     * Call this every 10ms for smooth control.
     */
    void update();

    /**
     * Enable the motor controller.
     * When enabled, update() will actively control the motor.
     */
    void enable();

    /**
     * Disable the motor controller.
     * When disabled, update() does nothing.
     */
    void disable();

    /**
     * Check if the motor has reached its target.
     * 
     * @param tolerance  How close (in degrees) counts as "arrived"
     * @return true if within tolerance of target
     */
    bool isAtTarget(float tolerance = 1.0f);

    /**
     * Set the PID tuning parameters.
     * 
     * @param kp  Proportional gain
     * @param ki  Integral gain
     * @param kd  Derivative gain
     */
    void setPIDGains(float kp, float ki, float kd);

private:
    /**
     * Take one physical step in the current direction.
     * This is the low-level motor control.
     */
    void _step();

    // Motor identification
    int _motorId;   // Which motor: 0 = left, 1 = right

    // GPIO pin numbers for direct motor control
    int _stepPin;   // Pin for step pulses
    int _dirPin;    // Pin for direction

    // Pointer to the encoder driver (shared between both motors)
    EncoderDriver* _encoder;

    // PID controller for this motor axis
    PIDController _pid;

    // Target and current angles
    float _targetAngle;     // Where we want to be
    float _currentAngle;    // Where we are (from encoder)

    // Enable/disable flag
    bool _enabled;          // true = actively controlling

    // Stepping timing (for non-blocking stepping)
    unsigned long _lastStepTime;     // When we last stepped (micros timestamp)
    unsigned long _stepDelayMicro;   // Microseconds between steps (0 = stopped)
    long _currentStepCount;          // Current step position (for debugging)

    // Last update time for dt calculation
    unsigned long _lastUpdateTime;  // When update() was last called
};

#endif  // CLOSED_LOOP_MOTOR_H     // End of the #ifndef guard block