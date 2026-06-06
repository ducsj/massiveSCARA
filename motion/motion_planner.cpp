/**
 * =============================================================================
 * This is file 10/12 – Motion Planner module.
 * =============================================================================
 * 
 * Purpose: Implementation of the MotionPlanner class.
 * 
 * This module generates smooth motion trajectories using trapezoidal
 * velocity profiles. The trapezoidal profile ensures:
 * - Smooth acceleration (no sudden jerks)
 * - Maximum speed limiting (protects motors)
 * - Controlled deceleration (no overshooting)
 * 
 * Trapezoidal Profile Explained:
 * 
 * A trapezoidal velocity profile has three phases:
 * 
 * 1. ACCELERATION PHASE (time: 0 to t_accel)
 *    - Speed increases from 0 to maxSpeed
 *    - Position: 0.5 * accel * t²
 * 
 * 2. CONSTANT VELOCITY PHASE (time: t_accel to t_accel + t_const)
 *    - Speed stays at maxSpeed
 *    - Position: dist_accel + maxSpeed * (t - t_accel)
 * 
 * 3. DECELERATION PHASE (time: t_accel + t_const to t_total)
 *    - Speed decreases from maxSpeed to 0
 *    - Position: Uses symmetric formula for smooth stop
 * 
 * If the distance is short, we might only have phases 1 and 3
 * (triangular profile, no constant velocity phase).
 * 
 * =============================================================================
 */

#include "motion_planner.h"       // Include the header file for this class

#include <math.h>                  // Include math functions: sqrt, fabs

/**
 * Constructor - initialize all state variables to safe defaults.
 */
MotionPlanner::MotionPlanner() {
    // Initialize all state variables to zero/empty
    reset();
}

/**
 * Reset the motion planner to its initial state.
 * Called by constructor and can be called manually to cancel motion.
 */
void MotionPlanner::reset() {
    // Clear the active flag - no motion is planned
    _active = false;
    
    // Reset timing to zero
    _startTime = 0;
    _totalTime = 0.0f;
    
    // Reset distances to zero
    _distance1 = 0.0f;
    _distance2 = 0.0f;
    
    // Reset start positions to zero
    _start1 = 0.0f;
    _start2 = 0.0f;
    
    // Reset speed and acceleration to zero
    _maxSpeed = 0.0f;
    _acceleration = 0.0f;
    
    // Reset trapezoidal profile parameters
    _tAccel = 0.0f;
    _tConst = 0.0f;
    _tDecel = 0.0f;
    _distAccel = 0.0f;
    _distConst = 0.0f;
}

/**
 * Plan a motion from start angles to target angles.
 * 
 * This function:
 * 1. Calculates the distance each motor needs to move
 * 2. Identifies the "master" axis (longer distance)
 * 3. Computes the trapezoidal profile for the master axis
 * 4. Scales the other axis proportionally
 * 
 * @param start1           Starting angle for motor 1 (degrees)
 * @param start2           Starting angle for motor 2 (degrees)
 * @param target1          Target angle for motor 1 (degrees)
 * @param target2          Target angle for motor 2 (degrees)
 * @param maxSpeedDegPerSec  Maximum speed (degrees/second)
 * @param accelDegPerSecSq   Maximum acceleration (degrees/second²)
 */
void MotionPlanner::plan(float start1, float start2, float target1, float target2,
                         float maxSpeedDegPerSec, float accelDegPerSecSq) {
    // First, reset any existing motion state
    reset();
    
    // Store the speed limits
    _maxSpeed = maxSpeedDegPerSec;
    _acceleration = accelDegPerSecSq;
    
    // Store the starting positions
    _start1 = start1;
    _start2 = start2;
    
    // Calculate the distance each motor needs to move
    // We use the shortest angular path (handling wrap-around)
    // For simplicity, we just use the direct difference
    // A more advanced version would normalize angles first
    _distance1 = target1 - start1;
    _distance2 = target2 - start2;
    
    // Use the larger distance as the master axis
    // This ensures the longer axis completes on time
    float masterDistance = fabs(_distance1) > fabs(_distance2) 
                           ? _distance1 : _distance2;
    
    // Calculate the absolute distance for profile calculation
    float absMasterDistance = fabs(masterDistance);
    
    // If there's no movement needed, we're done
    if (absMasterDistance < 0.001f) {
        _active = false;
        return;
    }
    
    // Compute the trapezoidal profile for the master axis
    _computeProfile(absMasterDistance);
    
    // Scale the slave axis to match the master timing
    // The slave axis moves proportionally to reach its target
    // at the same time as the master axis
    // (We don't need to recompute profile for slave, just scale position)
    
    // Mark the motion as active
    _active = true;
}

/**
 * Compute the trapezoidal profile parameters.
 * 
 * This determines:
 * - How long to spend accelerating
 * - How long to spend at constant speed
 * - How long to spend decelerating
 * 
 * The logic:
 * 1. Calculate how far we'd go if we only accelerated then decelerated
 *    (triangular profile - no constant velocity phase)
 * 2. If distance is less than that, use triangular profile
 * 3. Otherwise, use full trapezoidal with constant velocity phase
 * 
 * @param distance  Total distance to travel (must be positive)
 */
void MotionPlanner::_computeProfile(float distance) {
    // Calculate the distance covered during full acceleration
    // From physics: d = 0.5 * a * t², and v = a * t
    // So if we accelerate to maxSpeed and immediately decelerate:
    // t_accel = maxSpeed / acceleration
    // dist_triangular = 0.5 * accel * t_accel² + 0.5 * accel * t_accel²
    //                = accel * t_accel²
    //                = accel * (maxSpeed / accel)²
    //                = maxSpeed² / accel
    _tAccel = _maxSpeed / _acceleration;
    _distAccel = 0.5f * _acceleration * _tAccel * _tAccel;
    
    // Check if this is a triangular profile (distance fits in acceleration/deceleration)
    // If distance <= 2 * distAccel, we can reach maxSpeed and come back
    // without needing a constant velocity phase
    float distToTriangle = 2.0f * _distAccel;
    
    if (distance <= distToTriangle) {
        // TRIANGULAR PROFILE
        // We can't reach full speed - triangular profile instead
        // 
        // For triangular: we accelerate for time t, then decelerate for time t
        // distance = 0.5 * accel * t² + 0.5 * accel * t² = accel * t²
        // So: t = sqrt(distance / accel)
        // 
        // This is the time for ONE acceleration phase
        // Total time = 2 * t (accelerate + decelerate)
        _tAccel = sqrt(distance / _acceleration);
        _distAccel = 0.5f * _acceleration * _tAccel * _tAccel;
        _tConst = 0.0f;  // No constant velocity phase
        _distConst = 0.0f;
        _tDecel = _tAccel;  // Deceleration time equals acceleration time
    } else {
        // TRAPEZOIDAL PROFILE
        // We can reach full speed and maintain it for a while
        // 
        // Distance during acceleration = distAccel (as calculated above)
        // Distance during deceleration = distAccel (symmetric)
        // Distance during constant velocity = distance - 2 * distAccel
        _distConst = distance - 2.0f * _distAccel;
        
        // Time at constant velocity = distance / speed
        _tConst = _distConst / _maxSpeed;
        
        // Deceleration time equals acceleration time (symmetric)
        _tDecel = _tAccel;
    }
    
    // Calculate total motion time
    _totalTime = 2.0f * _tAccel + _tConst;
}

/**
 * Calculate position along the trapezoidal profile at a given time.
 * 
 * This is the core motion formula. It returns how far along the
 * trajectory we are at time t.
 * 
 * @param t    Current time since motion start (seconds)
 * @param dist Total distance to travel (degrees)
 * @return Position along trajectory (0 to dist)
 */
float MotionPlanner::_getPositionAtTime(float t, float dist) {
    // Clamp time to valid range (0 to totalTime)
    if (t <= 0) return 0.0f;
    if (t >= _totalTime) return dist;
    
    // Calculate position based on which phase we're in
    
    if (t < _tAccel) {
        // PHASE 1: ACCELERATION
        // Position = 0.5 * acceleration * t²
        // This is the equation for distance under constant acceleration
        return 0.5f * _acceleration * t * t;
    } 
    else if (t < _tAccel + _tConst) {
        // PHASE 2: CONSTANT VELOCITY
        // Position = distance already covered + speed * time
        // We've covered distAccel during acceleration
        // Then we travel at maxSpeed for (t - tAccel) seconds
        float tInConst = t - _tAccel;
        return _distAccel + _maxSpeed * tInConst;
    } 
    else {
        // PHASE 3: DECELERATION
        // This is symmetric to acceleration
        // Position = distance covered so far + formula for remaining
        // 
        // We can think of it as: start decelerating from the end
        // timeRemaining = tTotal - t
        // distanceRemaining = 0.5 * accel * timeRemaining²
        // position = totalDist - distanceRemaining
        float timeRemaining = _totalTime - t;
        float distRemaining = 0.5f * _acceleration * timeRemaining * timeRemaining;
        return dist - distRemaining;
    }
}

/**
 * Get the next position in the motion trajectory.
 * 
 * Call this repeatedly to get each set of angles the motors should be at.
 * Each call advances the motion by one time step based on elapsed time.
 * 
 * @param angle1  OUTPUT: Target angle for motor 1
 * @param angle2  OUTPUT: Target angle for motor 2
 * @return true if position is valid, false if motion complete
 */
bool MotionPlanner::nextStep(float& angle1, float& angle2) {
    // If no motion is active, return failure
    if (!_active) {
        return false;
    }
    
    // On first call, record the start time
    if (_startTime == 0) {
        _startTime = micros();
    }
    
    // Calculate elapsed time since start
    // micros() returns microseconds, so divide by 1,000,000 to get seconds
    unsigned long now = micros();
    float t = (now - _startTime) / 1000000.0f;
    
    // Check if motion is complete
    if (t >= _totalTime) {
        // Motion finished - set final positions and stop
        angle1 = _start1 + _distance1;
        angle2 = _start2 + _distance2;
        _active = false;
        return false;
    }
    
    // Calculate the master axis position using the trapezoidal profile
    // We use the larger of the two distances as our reference
    float masterDist = fabs(_distance1) > fabs(_distance2) 
                       ? fabs(_distance1) : fabs(_distance2);
    float masterPos = _getPositionAtTime(t, masterDist);
    
    // Scale the slave axis position proportionally
    // If master moves 50% of its distance, slave also moves 50% of its distance
    float ratio = (masterDist > 0.001f) ? masterPos / masterDist : 0.0f;
    
    // Calculate actual motor angles
    // Start position + (direction * scaled distance)
    // Direction is determined by the sign of the original distance
    angle1 = _start1 + _distance1 * ratio;
    angle2 = _start2 + _distance2 * ratio;
    
    // Return true to indicate we have a valid position
    return true;
}

/**
 * Check if the motion is still in progress.
 * 
 * @return true if motion is ongoing, false if complete or not started
 */
bool MotionPlanner::isMoving() {
    // We're moving if active and haven't finished
    if (!_active) {
        return false;
    }
    
    // Check if enough time has passed to complete the motion
    if (_startTime == 0) {
        return true;  // Motion planned but not yet started
    }
    
    unsigned long now = micros();
    float elapsed = (now - _startTime) / 1000000.0f;
    
    return elapsed < _totalTime;
}