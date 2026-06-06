/**
 * =============================================================================
 * This is file 2/12 – Motor Driver module.
 * =============================================================================
 * 
 * Purpose: Header file for the MotorDriver class that controls a single
 * 28BYJ-48 stepper motor using the ULN2003A driver board.
 * 
 * This provides open-loop motor control without encoders or homing sensors.
 * The motor moves step-by-step using precise timing via micros().
 * 
 * Hardware: 28BYJ-48 stepper motor + ULN2003A driver + ESP32-C3
 * 
 * =============================================================================
 */

#ifndef MOTOR_DRIVER_H           // Prevent this file from being included multiple times
#define MOTOR_DRIVER_H           // Define the guard symbol so #ifndef knows we've loaded this file

// Include Arduino framework for digitalWrite, pinMode, and types
#include <Arduino.h>

/**
 * MotorDriver class
 * 
 * Controls a single stepper motor in open-loop mode (no position feedback).
 * Uses non-blocking timing so other code can run in the main loop.
 */
class MotorDriver {
public:
    /**
     * Constructor - creates a new MotorDriver object.
     * No pins are configured yet; call begin() first.
     */
    MotorDriver();

    /**
     * Initialize the motor driver with GPIO pins.
     * 
     * @param stepPin   GPIO pin that sends step pulses to the ULN2003A driver
     * @param dirPin    GPIO pin that controls rotation direction
     * @param rpm       Initial speed in revolutions per minute (default: 15)
     */
    void begin(int stepPin, int dirPin, int rpm = 15);

    /**
     * Change the motor speed.
     * 
     * @param rpm New speed in revolutions per minute
     */
    void setSpeed(int rpm);

    /**
     * Set a target position for the motor to move to.
     * Movement is relative to current position:
     *   - Positive value = move forward that many steps
     *   - Negative value = move backward that many steps
     *   - Zero = stop (already there)
     * 
     * @param targetSteps Number of steps to move from current position
     */
    void moveTo(long targetSteps);

    /**
     * Update motor position - call this frequently in your main loop.
     * This is non-blocking: it only takes one step if enough time has passed.
     * 
     * Call this as often as possible for smooth motor operation.
     */
    void update();

    /**
     * Check if the motor is currently moving toward its target position.
     * 
     * @return true if motor is still stepping, false if reached target
     */
    bool isMoving();

    /**
     * Get the current step count (relative to where we started).
     * 
     * @return Current step position (can be positive or negative)
     */
    long getCurrentStep();

private:
    // GPIO pin numbers for step and direction signals
    int _stepPin;    // Pin that sends pulse for each step
    int _dirPin;     // Pin that controls rotation direction

    // Current and target step positions
    long _currentStep;   // Where we are right now (relative to start)
    long _targetStep;    // Where we want to go

    // Timing for non-blocking step generation
    unsigned long _delayMicros;     // Microseconds to wait between steps
    unsigned long _lastStepTime;    // When we took the last step (micros timestamp)

    /**
     * Take a single physical step in the correct direction.
     * This is called internally by update() when timing allows.
     */
    void _step();
};

#endif  // MOTOR_DRIVER_H        // End of the #ifndef guard block