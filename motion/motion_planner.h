/**
 * =============================================================================
 * This is file 10/12 – Motion Planner module.
 * =============================================================================
 * 
 * Purpose: Header file for the MotionPlanner class that generates smooth
 * motion trajectories for the SCARA robot using trapezoidal velocity profiles.
 * 
 * What is a Motion Planner?
 * 
 * When you want the robot to move from point A to point B, you could just
 * jump directly to the target position. But this would cause jerky, sudden
 * movements that could:
 * - Damage the motors
 * - Cause overshooting and oscillation
 * - Make the robot unstable
 * 
 * A motion planner solves this by generating a smooth path that:
 * - Starts slowly (acceleration phase)
 * - Reaches maximum speed (cruise phase)
 * - Slows down smoothly (deceleration phase)
 * 
 * This is called a "trapezoidal" profile because if you plot speed vs time,
 * it looks like a trapezoid: ramp up, flat top, ramp down.
 * 
 * Visual representation:
 * 
 *     Speed
 *       ^
 *       |    /--------\      Trapezoidal
 *       |   /          \    (has flat top)
 *       |  /            \
 *       | /              \
 *       +------------------> Time
 * 
 * For very short distances, we use a triangular profile (no flat top):
 * 
 *     Speed
 *       ^
 *       |  /\            Triangular
 *       | /  \           (no cruise phase)
 *       |/    \
 *       +------> Time
 * 
 * =============================================================================
 */

#ifndef MOTION_PLANNER_H         // Prevent this file from being included multiple times
#define MOTION_PLANNER_H          // Define the guard symbol so #ifndef knows we've loaded this file

// Include Arduino framework for micros() function
#include <Arduino.h>

/**
 * MotionPlanner class
 * 
 * Plans and executes smooth motion between two positions using
 * trapezoidal velocity profiles.
 * 
 * The planner takes:
 * - Start angles for both motors
 * - Target angles for both motors
 * - Maximum allowed speed (degrees/second)
 * - Maximum allowed acceleration (degrees/second²)
 * 
 * Then it generates intermediate positions that the motors should
 * follow to achieve smooth, controlled motion.
 * 
 * Usage:
 * 1. Call plan() to set up the motion
 * 2. Call nextStep() repeatedly to get each position
 * 3. When nextStep() returns false, motion is complete
 */
class MotionPlanner {
public:
    /**
     * Constructor - initializes all state variables to safe defaults.
     */
    MotionPlanner();

    /**
     * Plan a motion from start angles to target angles.
     * 
     * This calculates the trapezoidal velocity profile for the motion.
     * Call this before starting any movement.
     * 
     * @param start1           Starting angle for motor 1 (left) in degrees
     * @param start2           Starting angle for motor 2 (right) in degrees
     * @param target1          Target angle for motor 1 (left) in degrees
     * @param target2          Target angle for motor 2 (right) in degrees
     * @param maxSpeedDegPerSec  Maximum speed in degrees per second
     * @param accelDegPerSecSq   Maximum acceleration in degrees per second squared
     */
    void plan(float start1, float start2, float target1, float target2,
              float maxSpeedDegPerSec, float accelDegPerSecSq);

    /**
     * Get the next position in the motion trajectory.
     * 
     * Call this repeatedly in your main loop to get the next set of
     * angles the motors should be at. Each call advances the motion
     * by one time step.
     * 
     * @param angle1  OUTPUT: Target angle for motor 1 at this time step
     * @param angle2  OUTPUT: Target angle for motor 2 at this time step
     * @return true if this is a valid position, false if motion is complete
     */
    bool nextStep(float& angle1, float& angle2);

    /**
     * Check if the motion is still in progress.
     * 
     * @return true if motion is ongoing, false if complete or not started
     */
    bool isMoving();

    /**
     * Reset the motion planner.
     * 
     * Call this to cancel the current motion and prepare for a new one.
     * Also called automatically by plan().
     */
    void reset();

private:
    // =======================================================================
    // MOTION STATE VARIABLES
    // =======================================================================

    /** true if a motion is currently planned and in progress */
    bool _active;

    /** Time when the current motion started (microseconds) */
    unsigned long _startTime;

    /** Total duration of the motion in seconds */
    float _totalTime;

    /** Distance (angle change) for motor 1 in degrees */
    float _distance1;

    /** Distance (angle change) for motor 2 in degrees */
    float _distance2;

    /** Starting angle for motor 1 in degrees */
    float _start1;

    /** Starting angle for motor 2 in degrees */
    float _start2;

    /** Maximum speed in degrees per second */
    float _maxSpeed;

    /** Maximum acceleration in degrees per second squared */
    float _acceleration;

    // =======================================================================
    // TRAPEZOIDAL PROFILE PARAMETERS
    // =======================================================================
    // These describe the shape of the velocity profile

    /** Time spent accelerating at the beginning (seconds) */
    float _tAccel;

    /** Time spent at constant speed (seconds) */
    float _tConst;

    /** Time spent decelerating at the end (seconds) */
    float _tDecel;

    /** Distance covered during acceleration phase */
    float _distAccel;

    /** Distance covered during constant velocity phase */
    float _distConst;

    // =======================================================================
    // HELPER METHODS
    // =======================================================================

    /**
     * Calculate position along the trapezoidal profile at a given time.
     * 
     * This implements the trapezoidal motion formula:
     * - During acceleration: position = 0.5 * accel * t²
     * - During cruise: position = distAccel + maxSpeed * (t - tAccel)
     * - During deceleration: position uses symmetric formula
     * 
     * @param t    Current time since motion start (seconds)
     * @param dist Total distance to travel (degrees)
     * @return Position along the trajectory (0 to dist)
     */
    float _getPositionAtTime(float t, float dist);

    /**
     * Calculate the trapezoidal profile parameters.
     * 
     * Determines if we need triangular or trapezoidal profile based on
     * the distance to travel and the speed/acceleration limits.
     * 
     * @param distance  Total distance to travel (degrees)
     */
    void _computeProfile(float distance);
};

#endif  // MOTION_PLANNER_H       // End of the #ifndef guard block