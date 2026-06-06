/**
 * =============================================================================
 * This is file 2/12 – Motor Driver module.
 * =============================================================================
 * 
 * Purpose: Implementation of the MotorDriver class for controlling a
 * 28BYJ-48 stepper motor via ULN2003A driver.
 * 
 * Uses non-blocking timing with micros() so the main loop stays responsive.
 * The 28BYJ-48 motor requires 4096 steps per full revolution in half-step mode.
 * 
 * =============================================================================
 */

#include "motor_driver.h"         // Include the header file for this class

/**
 * Constructor - initializes all member variables to safe defaults.
 * Pins are not configured yet; call begin() to set them up.
 */
MotorDriver::MotorDriver() {
    // Initialize pins to invalid values (will be set in begin())
    _stepPin = 0;
    _dirPin = 0;
    
    // Start at position 0 (this is our reference point)
    _currentStep = 0;
    _targetStep = 0;
    
    // Default timing: 15 RPM is a safe starting speed
    // Calculate: 60000000 / (15 * 4096) = ~976 microseconds per step
    _delayMicros = 60000000UL / (15 * 4096);
    
    // Start with "never stepped" - first step happens immediately
    _lastStepTime = 0;
}

/**
 * Initialize the motor driver with GPIO pins and set initial speed.
 * 
 * @param stepPin   GPIO pin for step pulses (connected to ULN2003A IN1-IN4)
 * @param dirPin    GPIO pin for direction control (connected to ULN2003A direction)
 * @param rpm       Initial speed in revolutions per minute
 */
void MotorDriver::begin(int stepPin, int dirPin, int rpm) {
    // Store the pin numbers so we can use them later
    _stepPin = stepPin;
    _dirPin = dirPin;
    
    // Configure both pins as outputs (they send signals to the driver board)
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    
    // Initialize pins to LOW (no voltage = motor not actively driven)
    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);
    
    // Calculate the correct timing for the requested speed
    setSpeed(rpm);
}

/**
 * Change the motor speed by recalculating the delay between steps.
 * 
 * Formula: delayMicroseconds = 60000000 / (rpm * stepsPerRevolution)
 * 
 * For 28BYJ-48 in half-step mode: stepsPerRevolution = 4096
 * Example at 15 RPM: 60000000 / (15 * 4096) = 976 microseconds
 * 
 * @param rpm New speed in revolutions per minute
 */
void MotorDriver::setSpeed(int rpm) {
    // Avoid division by zero if someone passes 0 RPM
    if (rpm <= 0) {
        rpm = 1;  // Minimum speed of 1 RPM
    }
    
    // Calculate microseconds between each step
    // 60000000 = 60 seconds * 1000000 microseconds (one minute in microseconds)
    // 4096 = steps per revolution for 28BYJ-48 in half-step mode
    _delayMicros = 60000000UL / (rpm * 4096);
}

/**
 * Set the target position for the motor to move to.
 * The target is relative to the current position:
 *   - moveTo(100) = move forward 100 steps
 *   - moveTo(-50) = move backward 50 steps
 *   - moveTo(0) = stop (already at target)
 * 
 * The direction (forward/backward) is determined automatically based on
 * whether targetSteps is positive or negative.
 * 
 * @param targetSteps Number of steps to move from current position
 */
void MotorDriver::moveTo(long targetSteps) {
    // Store the target as an offset from current position
    // Example: if currentStep is 50 and we call moveTo(100),
    // the new target will be 150 (we want to end up 100 steps ahead)
    _targetStep = _currentStep + targetSteps;
}

/**
 * Check if the motor has reached its target position.
 * 
 * @return true if motor is still stepping toward target, false if arrived
 */
bool MotorDriver::isMoving() {
    // We are moving if current position differs from target position
    return _currentStep != _targetStep;
}

/**
 * Get the current step count (position relative to where we started).
 * 
 * @return Current step position (can be negative if we moved backward)
 */
long MotorDriver::getCurrentStep() {
    // Return the current position counter
    return _currentStep;
}

/**
 * Take one physical step in the direction needed to reach target.
 * This is the core stepping logic that actually drives the motor.
 */
void MotorDriver::_step() {
    // Determine which direction we need to go
    // If target is greater than current position, we need to go forward (positive)
    // If target is less than current position, we need to go backward (negative)
    bool direction = _targetStep > _currentStep;
    
    // Set the direction pin:
    // - HIGH (true) = one direction (e.g., clockwise)
    // - LOW (false) = opposite direction (e.g., counter-clockwise)
    digitalWrite(_dirPin, direction ? HIGH : LOW);
    
    // Small delay to let the direction signal settle
    // This ensures the motor doesn't get confused by changing direction too fast
    delayMicroseconds(1);
    
    // Generate the step pulse:
    // - HIGH starts the step (coil energizes)
    digitalWrite(_stepPin, HIGH);
    
    // Minimum pulse width - this is how long the pulse must be held HIGH
    // for the ULN2003A to register it. 1 microsecond is usually enough.
    delayMicroseconds(1);
    
    // - LOW ends the step (coil remains energized but pulse is done)
    digitalWrite(_stepPin, LOW);
    
    // Record when this step happened (for timing the next one)
    _lastStepTime = micros();
    
    // Update our position counter:
    // - Increment if moving forward
    // - Decrement if moving backward
    if (direction) {
        _currentStep++;
    } else {
        _currentStep--;
    }
}

/**
 * Update motor position - call this frequently in your main loop.
 * 
 * This is non-blocking: it only takes one step if enough time has passed
 * since the last step. This allows your main loop to do other things.
 * 
 * How it works:
 * 1. Check if we're already at the target (skip if done)
 * 2. Check if enough time has passed since last step
 * 3. If yes, take one step and update timing
 * 
 * Call this as often as possible (every loop iteration) for smooth motion.
 */
void MotorDriver::update() {
    // Skip if we already reached the target position
    if (_currentStep == _targetStep) {
        return;  // Nothing to do
    }
    
    // Get the current time in microseconds
    unsigned long now = micros();
    
    // Calculate how long it's been since the last step
    unsigned long elapsed = now - _lastStepTime;
    
    // Check if enough time has passed for the next step
    if (elapsed >= _delayMicros) {
        // Time to take another step!
        _step();
    }
    // If not enough time has passed, we just wait and check again next update()
}