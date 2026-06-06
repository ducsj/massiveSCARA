/**
 * =============================================================================
 * This is file 5/12 – Closed-Loop Motor module.
 * =============================================================================
 * 
 * Purpose: Implementation of the ClosedLoopMotor class.
 * 
 * This class combines:
 * - MotorDriver: for step/dir pin control (we replicate the logic here for
 *   continuous speed control)
 * - EncoderDriver: for reading actual motor position
 * - PIDController: for calculating the right speed to reach target
 * 
 * The key innovation is that we DON'T use MotorDriver's moveTo() system.
 * Instead, we directly step the motor at a speed determined by the PID output.
 * This gives us smooth, continuous speed control based on error magnitude.
 * 
 * How the stepping works:
 * - PID output is in steps/second (-200 to +200 typically)
 * - We convert this to a microsecond delay between steps
 * - If output is 100 steps/sec, delay = 1,000,000 / 100 = 10,000 microseconds
 * - In update(), we check if enough time has passed and take one step
 * 
 * =============================================================================
 */

#include "closed_loop_motor.h"     // Include the header file for this class
#include "pid_controller.h"       // Include PID controller
#include "../drivers/encoder_driver.h"  // Include encoder driver

// Steps per revolution for 28BYJ-48 in half-step mode
// This is used to convert between RPM and steps/second
#define STEPS_PER_REVOLUTION 4096

/**
 * Constructor - initialize all member variables to safe defaults.
 */
ClosedLoopMotor::ClosedLoopMotor() {
    // Initialize motor ID to invalid value
    _motorId = -1;
    
    // Initialize pin numbers to 0
    _stepPin = 0;
    _dirPin = 0;
    
    // No encoder connected yet (will be set in begin())
    _encoder = nullptr;
    
    // Initialize angles to 0
    _targetAngle = 0.0f;
    _currentAngle = 0.0f;
    
    // Start disabled (user must call enable())
    _enabled = false;
    
    // Initialize stepping to stopped
    _lastStepTime = 0;
    _stepDelayMicro = 0;   // 0 = no stepping
    _currentStepCount = 0;
    
    // Initialize timing
    _lastUpdateTime = 0;
}

/**
 * Initialize the closed-loop motor controller.
 * 
 * @param motorId      Which motor: 0 = left arm, 1 = right arm
 * @param stepPin      GPIO pin for step pulses
 * @param dirPin       GPIO pin for direction
 * @param rpm          Initial motor speed in RPM (used for max speed reference)
 * @param encoderPtr   Pointer to the global EncoderDriver instance
 */
void ClosedLoopMotor::begin(int motorId, int stepPin, int dirPin, int rpm, 
                            EncoderDriver* encoderPtr) {
    // Store the motor identification number
    _motorId = motorId;
    
    // Store the GPIO pin numbers for later use
    _stepPin = stepPin;
    _dirPin = dirPin;
    
    // Store the pointer to the encoder driver
    // This lets us read the actual motor position
    _encoder = encoderPtr;
    
    // Configure the GPIO pins as outputs
    // These pins send signals to the ULN2003A driver
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    
    // Initialize pins to LOW (no active signal)
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);
    
    // Configure the PID controller
    // These are starting values - tune them for your robot
    _pid.setGains(PID_KP, PID_KI, PID_KD);  // From robot_config.h
    
    // Set PID output limits based on motor speed
    // At rpm, the motor does: rpm * 4096 / 60 steps per second
    // For example, 15 RPM = 15 * 4096 / 60 = 1024 steps/sec
    float maxStepsPerSec = rpm * STEPS_PER_REVOLUTION / 60.0f;
    _pid.setOutputLimits(-maxStepsPerSec, maxStepsPerSec);
    
    // Set PID sample time to 10ms (100 Hz update rate)
    _pid.setSampleTime(10);
    
    // Start with target at current position (no movement)
    _targetAngle = 0.0f;
    
    // Reset PID state (clear any accumulated values)
    _pid.reset();
    
    // Enable the controller by default
    _enabled = true;
    
    // Initialize timing
    _lastUpdateTime = millis();
    _lastStepTime = micros();
}

/**
 * Set the target angle for this motor to move to.
 * 
 * @param angleDegrees  Target angle in degrees
 */
void ClosedLoopMotor::setTargetAngle(float angleDegrees) {
    // Store the new target angle
    _targetAngle = angleDegrees;
    
    // Set the target in the PID controller
    // This is what the PID tries to reach
    _pid.setTarget(angleDegrees);
    
    // Reset the PID state when changing targets
    // This prevents old accumulated errors from affecting new movement
    _pid.reset();
}

/**
 * Get the current angle from the encoder.
 * 
 * @return Current angle in degrees (calibrated)
 */
float ClosedLoopMotor::getCurrentAngle() {
    // If we have an encoder, read from it
    if (_encoder != nullptr) {
        // Use the calibrated angle which includes zero offset
        _currentAngle = _encoder->getCalibratedAngle(_motorId);
    }
    
    // Return the current angle
    return _currentAngle;
}

/**
 * Take one physical step in the current direction.
 * This is called internally when timing allows.
 */
void ClosedLoopMotor::_step() {
    // Get the current time for timing the step pulse
    unsigned long now = micros();
    
    // Set the direction pin based on whether we're moving forward or backward
    // The PID output sign tells us the direction:
    // Positive output = forward (clockwise)
    // Negative output = backward (counter-clockwise)
    // We determine direction by checking if output >= 0
    float output = _pid.update(_currentAngle);  // Get current PID output
    bool direction = output >= 0;               // true = forward, false = backward
    
    // Write the direction to the pin
    digitalWrite(_dirPin, direction ? HIGH : LOW);
    
    // Small delay to let the direction signal settle
    // This prevents the motor from getting confused
    delayMicroseconds(1);
    
    // Generate the step pulse:
    // 1. Set step pin HIGH to start the step
    digitalWrite(_stepPin, HIGH);
    
    // 2. Hold the pulse for minimum width (1 microsecond is enough)
    delayMicroseconds(1);
    
    // 3. Set step pin LOW to end the step
    // The motor will hold this position until next step
    digitalWrite(_stepPin, LOW);
    
    // Record when this step happened (for timing the next step)
    _lastStepTime = now;
    
    // Update our step counter:
    // Increment if moving forward, decrement if backward
    if (direction) {
        _currentStepCount++;   // Moving forward
    } else {
        _currentStepCount--;   // Moving backward
    }
}

/**
 * Update the motor - call this frequently in your main loop.
 * 
 * This is the main control loop that:
 * 1. Reads the current angle from the encoder
 * 2. Runs the PID to calculate motor speed
 * 3. Steps the motor at the calculated speed
 * 
 * The PID output is in steps per second:
 * - Output of 100 means move at 100 steps per second
 * - This translates to a delay of 10,000 microseconds between steps
 * 
 * @param none
 */
void ClosedLoopMotor::update() {
    // Skip if disabled
    if (!_enabled) {
        return;
    }
    
    // Get the current time in milliseconds
    unsigned long now = millis();
    
    // Calculate time since last update (dt) in milliseconds
    unsigned long dt = now - _lastUpdateTime;
    
    // Only update at the configured sample rate (10ms)
    // This prevents the PID from running too fast
    if (dt < 10) {
        return;
    }
    
    // ---- STEP 1: READ ENCODER ----
    // Get the current motor position from the encoder
    _currentAngle = getCurrentAngle();
    
    // ---- STEP 2: RUN PID CONTROLLER ----
    // The PID calculates how fast we should move based on error
    // Output range: -maxSpeed to +maxSpeed (steps per second)
    // Positive = move forward, Negative = move backward
    float output = _pid.update(_currentAngle);
    
    // ---- STEP 3: CONVERT PID OUTPUT TO STEP TIMING ----
    // The PID output is in steps per second
    // We need to convert this to a microsecond delay between steps
    
    // If output is close to zero, we're basically at target - stop stepping
    if (abs(output) < 1.0f) {
        // Motor is close enough to target, stop stepping
        _stepDelayMicro = 0;   // 0 means "don't step"
    } else {
        // Calculate delay between steps
        // delay = 1,000,000 microseconds / steps_per_second
        // Example: 100 steps/sec → 10,000 microseconds between steps
        _stepDelayMicro = 1000000.0f / abs(output);
    }
    
    // ---- STEP 4: STEP THE MOTOR ----
    // Check if it's time for the next step
    unsigned long nowMicro = micros();
    unsigned long elapsed = nowMicro - _lastStepTime;
    
    // If enough time has passed, take one step
    if (_stepDelayMicro > 0 && elapsed >= _stepDelayMicro) {
        _step();   // Take one physical step
    }
    
    // Save the current time for next iteration
    _lastUpdateTime = now;
}

/**
 * Enable the motor controller.
 * When enabled, update() will actively control the motor.
 */
void ClosedLoopMotor::enable() {
    _enabled = true;
}

/**
 * Disable the motor controller.
 * When disabled, update() does nothing.
 */
void ClosedLoopMotor::disable() {
    _enabled = false;
    
    // Also stop any current stepping
    _stepDelayMicro = 0;
}

/**
 * Check if the motor has reached its target.
 * 
 * @param tolerance  How close (in degrees) counts as "arrived"
 * @return true if within tolerance of target
 */
bool ClosedLoopMotor::isAtTarget(float tolerance) {
    // Read the current angle
    float current = getCurrentAngle();
    
    // Calculate how far we are from target
    float error = abs(_targetAngle - current);
    
    // Return true if error is within tolerance
    return error <= tolerance;
}

/**
 * Set the PID tuning parameters.
 * 
 * @param kp  Proportional gain
 * @param ki  Integral gain
 * @param kd  Derivative gain
 */
void ClosedLoopMotor::setPIDGains(float kp, float ki, float kd) {
    // Pass the gains to the PID controller
    _pid.setGains(kp, ki, kd);
}