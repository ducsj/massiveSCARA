/**
 * =============================================================================
 * This is file 9/12 – Inverse Kinematics module.
 * =============================================================================
 * 
 * Purpose: Implementation of the InverseKinematics class.
 * 
 * This file implements the mathematics to convert X,Y coordinates to motor
 * angles for a 5-bar parallel SCARA robot.
 * 
 * The algorithm:
 * 1. Calculate the distance from each motor to the target point
 * 2. Check if the target is within reach (not too close, not too far)
 * 3. Use the law of cosines to find the elbow angle at each motor
 * 4. Combine with the shoulder angle to get the motor angles
 * 
 * Law of Cosines refresher:
 * For a triangle with sides a, b, c and opposite angles A, B, C:
 * 
 *     c
 *    ┌───┐
 *  a │   │ b
 *    └───┘
 *      A
 * 
 *   c² = a² + b² - 2ab·cos(C)
 *   cos(C) = (a² + b² - c²) / (2ab)
 *   C = acos((a² + b² - c²) / (2ab))
 * 
 * In our robot:
 * - Side a = L1 (upper arm)
 * - Side b = L2 (forearm)
 * - Side c = D (distance from motor to target)
 * 
 * The angle at the motor (shoulder) is the angle from horizontal to the
 * line connecting the motor to the target, minus the angle at the elbow.
 * 
 * =============================================================================
 */

#include "inverse_kinematics.h"   // Include the header file for this class

#include <math.h>                  // Include math functions: sqrt, atan2, acos

// =============================================================================
// STATIC VARIABLE INITIALIZATION
// =============================================================================
// Static member variables must be initialized outside the class declaration.
// These hold the robot's physical dimensions.

/** Distance between the two motors in millimeters (default: 50mm). */
float InverseKinematics::_motorSpacing = 50.0f;

/** Length of the upper arm (motor to elbow) in millimeters (default: 70mm). */
float InverseKinematics::_l1 = 70.0f;

/** Length of the forearm (elbow to tool) in millimeters (default: 100mm). */
float InverseKinematics::_l2 = 100.0f;

// =============================================================================
// PUBLIC METHODS
// =============================================================================

/**
 * Set the robot's physical dimensions.
 * 
 * Call this in setup() with your robot's actual measurements.
 * 
 * @param motorSpacing  Distance between motors in mm
 * @param l1            Upper arm length in mm
 * @param l2            Forearm length in mm
 */
void InverseKinematics::setGeometry(float motorSpacing, float l1, float l2) {
    // Store the motor spacing (distance between left and right motors)
    _motorSpacing = motorSpacing;
    
    // Store the upper arm length (from motor shaft to elbow joint)
    _l1 = l1;
    
    // Store the forearm length (from elbow joint to tool)
    _l2 = l2;
}

/**
 * Get the robot's physical dimensions.
 * 
 * @param motorSpacing  OUTPUT: Distance between motors in mm
 * @param l1            OUTPUT: Upper arm length in mm
 * @param l2            OUTPUT: Forearm length in mm
 */
void InverseKinematics::getGeometry(float& motorSpacing, float& l1, float& l2) {
    // Copy the stored geometry values to the output parameters
    motorSpacing = _motorSpacing;
    l1 = _l1;
    l2 = _l2;
}

/**
 * Compute motor angles for a given X,Y position.
 * 
 * This is the main function of the inverse kinematics module.
 * It calculates what angle each motor needs to be at to position
 * the tool at the specified (x, y) coordinates.
 * 
 * @param x             Target X position in millimeters
 * @param y             Target Y position in millimeters
 * @param angleLeftDeg  OUTPUT: Left motor angle in degrees
 * @param angleRightDeg OUTPUT: Right motor angle in degrees
 * @param elbowUp       true = elbow points up, false = elbow points down
 * @return true if position is reachable, false if out of range
 */
bool InverseKinematics::computeAngles(float x, float y,
                                      float& angleLeftDeg, float& angleRightDeg,
                                      bool elbowUp) {
    // =================================================================
    // STEP 1: Calculate vectors from each motor to the target point
    // =================================================================
    
    // For the LEFT motor:
    // The left motor is located at (-motorSpacing/2, 0)
    // So the vector to the target is: (x - (-motorSpacing/2), y - 0)
    // Which simplifies to: (x + motorSpacing/2, y)
    float dxLeft = x + _motorSpacing / 2.0f;
    float dyLeft = y;
    
    // Calculate the straight-line distance from left motor to target
    // Using the Pythagorean theorem: distance = sqrt(dx² + dy²)
    float distanceLeft = sqrt(dxLeft * dxLeft + dyLeft * dyLeft);
    
    // For the RIGHT motor:
    // The right motor is located at (+motorSpacing/2, 0)
    // So the vector to the target is: (x - motorSpacing/2, y - 0)
    float dxRight = x - _motorSpacing / 2.0f;
    float dyRight = y;
    
    // Calculate the straight-line distance from right motor to target
    float distanceRight = sqrt(dxRight * dxRight + dyRight * dyRight);
    
    // =================================================================
    // STEP 2: Check if the target is reachable
    // =================================================================
    
    // A position is reachable if the distance from a motor is:
    // - Greater than |L1 - L2| (not too close - arms can fold)
    // - Less than L1 + L2 (not too far - arms can extend)
    // 
    // If the point is outside this range, it's unreachable.
    if (!_isReachable(distanceLeft) || !_isReachable(distanceRight)) {
        // Target is out of reach - return failure
        return false;
    }
    
    // =================================================================
    // STEP 3: Calculate the LEFT motor angle
    // =================================================================
    
    // The shoulder angle is the angle from horizontal (forward direction)
    // to the line connecting the motor to the target point.
    // We use atan2 because it handles all quadrants correctly.
    // atan2(y, x) returns angle in radians from -π to +π
    float shoulderAngleLeft = atan2(dyLeft, dxLeft);
    
    // Now we need to find the elbow angle at the left motor.
    // This is the angle between the upper arm and the line to the target.
    // 
    // Using the law of cosines:
    // cos(elbow) = (L1² + D² - L2²) / (2 × L1 × D)
    // 
    // Where:
    // - L1 is the upper arm length
    // - L2 is the forearm length
    // - D is the distance from motor to target
    float cosElbowLeft = (_l1 * _l1 + distanceLeft * distanceLeft - _l2 * _l2) 
                         / (2.0f * _l1 * distanceLeft);
    
    // Clamp cosElbow to valid range for acos (-1 to 1)
    // This handles floating-point precision issues
    if (cosElbowLeft > 1.0f) cosElbowLeft = 1.0f;
    if (cosElbowLeft < -1.0f) cosElbowLeft = -1.0f;
    
    // The elbow angle (in radians) is the inverse cosine of the above
    float elbowAngleLeft = acos(cosElbowLeft);
    
    // For elbow-up configuration, the motor angle is the shoulder angle
    // minus the elbow angle (the elbow bends backward/upward).
    // For elbow-down, we add the elbow angle instead.
    float angleLeftRad;
    if (elbowUp) {
        // Elbow points up (toward the top of the workspace)
        angleLeftRad = shoulderAngleLeft - elbowAngleLeft;
    } else {
        // Elbow points down (toward the bottom)
        angleLeftRad = shoulderAngleLeft + elbowAngleLeft;
    }
    
    // Convert to degrees and normalize to 0-360 range
    angleLeftDeg = _normalizeAngle(_toDegrees(angleLeftRad));
    
    // =================================================================
    // STEP 4: Calculate the RIGHT motor angle
    // =================================================================
    
    // Same process as for the left motor, but using the right motor's
    // position and vector.
    float shoulderAngleRight = atan2(dyRight, dxRight);
    
    // Law of cosines for the right elbow angle
    float cosElbowRight = (_l1 * _l1 + distanceRight * distanceRight - _l2 * _l2) 
                          / (2.0f * _l1 * distanceRight);
    
    // Clamp to valid range
    if (cosElbowRight > 1.0f) cosElbowRight = 1.0f;
    if (cosElbowRight < -1.0f) cosElbowRight = -1.0f;
    
    float elbowAngleRight = acos(cosElbowRight);
    
    // Calculate the right motor angle
    // Note: For the right motor, the sign of elbow angle is flipped
    // because the arm geometry is mirrored.
    float angleRightRad;
    if (elbowUp) {
        // For elbow-up, we ADD the elbow angle on the right side
        // (because the right arm mirrors the left)
        angleRightRad = shoulderAngleRight + elbowAngleRight;
    } else {
        // For elbow-down, we SUBTRACT the elbow angle on the right side
        angleRightRad = shoulderAngleRight - elbowAngleRight;
    }
    
    // Convert to degrees and normalize to 0-360 range
    angleRightDeg = _normalizeAngle(_toDegrees(angleRightRad));
    
    // =================================================================
    // SUCCESS!
    // =================================================================
    
    // The target position is reachable and we've calculated the angles.
    return true;
}

// =============================================================================
// PRIVATE HELPER METHODS
// =============================================================================

/**
 * Convert radians to degrees.
 * 
 * There are 2π radians in a full circle (360°).
 * So to convert radians to degrees, multiply by 180/π.
 * 
 * @param radians  Angle in radians
 * @return Angle in degrees
 */
float InverseKinematics::_toDegrees(float radians) {
    // Multiply radians by (180/π) to get degrees
    return radians * 180.0f / M_PI;
}

/**
 * Normalize an angle to the 0-360 degree range.
 * 
 * This ensures angles are always positive and less than 360.
 * For example:
 * - -45° becomes 315°
 * - 400° becomes 40°
 * - 720° becomes 0°
 * 
 * @param angle  Angle in degrees (can be any value)
 * @return Normalized angle in range 0-360
 */
float InverseKinematics::_normalizeAngle(float angle) {
    // Use the modulo operator to get the remainder after dividing by 360
    float normalized = fmod(angle, 360.0f);
    
    // Handle negative angles
    // If the result is negative, add 360 to make it positive
    // fmod(-45, 360) = -45, so -45 + 360 = 315 ✓
    if (normalized < 0) {
        normalized += 360.0f;
    }
    
    return normalized;
}

/**
 * Check if a distance is within the robot's reachable range.
 * 
 * The robot can reach a point if the distance from a motor is:
 * - Greater than |L1 - L2|: This is the minimum reach, when the arms
 *   are folded completely on top of each other.
 * - Less than L1 + L2: This is the maximum reach, when the arms are
 *   fully extended in a straight line.
 * 
 * @param distance  Distance from motor to target point
 * @return true if reachable, false if not
 */
bool InverseKinematics::_isReachable(float distance) {
    // Calculate the minimum and maximum reachable distances
    // minimumReach = |L1 - L2| (can be 0 if arms are same length)
    float minimumReach = fabs(_l1 - _l2);
    
    // maximumReach = L1 + L2 (when arms are fully extended)
    float maximumReach = _l1 + _l2;
    
    // Check if the distance is within the reachable range
    // Add small tolerance for floating-point comparison
    float epsilon = 0.001f;
    
    return (distance >= minimumReach - epsilon) && (distance <= maximumReach + epsilon);
}