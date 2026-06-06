/**
 * =============================================================================
 * This is file 9/12 – Inverse Kinematics module.
 * =============================================================================
 * 
 * Purpose: Header file for the InverseKinematics class that converts
 * Cartesian (X, Y) coordinates to motor angles for a 5-bar parallel SCARA robot.
 * 
 * What is Inverse Kinematics?
 * 
 * In robotics, "kinematics" is the study of motion without considering forces.
 * "Inverse" kinematics means working backwards: instead of saying
 * "if I rotate the motors by X degrees, where does the tool go?",
 * we want to know "if I want the tool at position (X, Y), what angles
 * should the motors be at?"
 * 
 * This is essential for motion planning because users think in X,Y coordinates,
 * but the motors need to know angles.
 * 
 * The 5-bar SCARA robot geometry:
 * 
 *     Left Motor                          Right Motor
 *         M1 ────── elbow1 ─────┐
 *            \                   │
 *             \    L1            |  L2
 *              \                /
 *               \              /
 *                ────────────
 *                   elbow2
 *                    |
 *                    | (tool)
 * 
 * The two motors drive two parallel linkages that meet at the tool point.
 * 
 * =============================================================================
 */

#ifndef INVERSE_KINEMATICS_H     // Prevent this file from being included multiple times
#define INVERSE_KINEMATICS_H      // Define the guard symbol so #ifndef knows we've loaded this file

// Include Arduino framework for types
#include <Arduino.h>

/**
 * InverseKinematics class
 * 
 * Provides functions to calculate motor angles from X,Y coordinates.
 * 
 * The robot has two motors at fixed positions, each driving an arm of
 * length L1 connected to a forearm of length L2. The two forearms meet
 * at the tool point.
 * 
 * Coordinate system:
 * - Origin (0,0) is midway between the two motors
 * - X increases to the right (toward right motor)
 * - Y increases forward (toward the tool reach area)
 * - Left motor is at (-motorSpacing/2, 0)
 * - Right motor is at (+motorSpacing/2, 0)
 * 
 * Angles are measured from the horizontal (forward) direction:
 * - 0° = arm pointing straight forward
 * - Positive angles = arm angled to the left (counter-clockwise)
 * - Negative angles = arm angled to the right (clockwise)
 */
class InverseKinematics {
public:
    /**
     * Compute motor angles for a given X,Y position.
     * 
     * Given a desired tool position (x, y), this function calculates
     * the angles that both motors need to be at to reach that position.
     * 
     * The calculation uses the law of cosines to find the angles at each
     * motor based on the geometry of the linkages.
     * 
     * @param x             Target X position in millimeters (0 = center)
     * @param y             Target Y position in millimeters
     * @param angleLeftDeg  OUTPUT: Angle for left motor in degrees
     * @param angleRightDeg OUTPUT: Angle for right motor in degrees
     * @param elbowUp       true = elbow points up (pen above workspace)
     *                       false = elbow points down (pen below workspace)
     * 
     * @return true if position is reachable, false if out of reach
     * 
     * Example usage:
     *   float leftAngle, rightAngle;
     *   if (InverseKinematics::computeAngles(30, 50, leftAngle, rightAngle)) {
     *       // Move motors to leftAngle and rightAngle
     *   } else {
     *       // Position is out of reach
     *   }
     */
    static bool computeAngles(float x, float y, 
                              float& angleLeftDeg, float& angleRightDeg,
                              bool elbowUp = true);

    /**
     * Set the robot's physical dimensions.
     * 
     * Call this to configure the module with your robot's actual measurements.
     * 
     * @param motorSpacing  Distance between motors in millimeters
     * @param l1            Length of upper arm (motor to elbow) in millimeters
     * @param l2            Length of forearm (elbow to tool) in millimeters
     */
    static void setGeometry(float motorSpacing, float l1, float l2);

    /**
     * Get the robot's physical dimensions.
     * 
     * @param motorSpacing  OUTPUT: Distance between motors in mm
     * @param l1            OUTPUT: Upper arm length in mm
     * @param l2            OUTPUT: Forearm length in mm
     */
    static void getGeometry(float& motorSpacing, float& l1, float& l2);

private:
    // Robot geometry parameters (set by setGeometry)
    static float _motorSpacing;   // Distance between motors in mm
    static float _l1;             // Upper arm length in mm
    static float _l2;             // Forearm length in mm

    /**
     * Convert an angle from radians to degrees.
     * 
     * @param radians  Angle in radians
     * @return Angle in degrees
     */
    static float _toDegrees(float radians);

    /**
     * Normalize an angle to the 0-360 degree range.
     * 
     * This ensures angles are always positive and less than 360.
     * For example, -45° becomes 315°, and 400° becomes 40°.
     * 
     * @param angle  Angle in degrees (can be negative or > 360)
     * @return Normalized angle in range 0-360
     */
    static float _normalizeAngle(float angle);

    /**
     * Check if a position is within the robot's reachable workspace.
     * 
     * A point is reachable if:
     * - It's not too close to the motors (greater than minimum reach)
     * - It's not too far from the motors (less than maximum reach)
     * 
     * @param distance  Distance from a motor to the target point
     * @return true if reachable, false if not
     */
    static bool _isReachable(float distance);
};

#endif  // INVERSE_KINEMATICS_H    // End of the #ifndef guard block