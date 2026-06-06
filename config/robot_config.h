/**
 * =============================================================================
 * ROBOT CONFIGURATION FILE
 * =============================================================================
 * 
 * Purpose: This file stores all the hardware settings and physical dimensions
 * for the 5-bar parallel SCARA robot. Having everything in one place makes it
 * easy to change settings without hunting through the code.
 * 
 * Hardware: ESP32-C3 SuperMini microcontroller
 * 
 * Author: SCARA Robot Project
 * =============================================================================
 */

#ifndef ROBOT_CONFIG_H          // Prevent this file from being included multiple times
#define ROBOT_CONFIG_H           // Define the guard symbol so #ifndef knows we've loaded this file

// =============================================================================
// SECTION 1: ROBOT GEOMETRY
// =============================================================================
// These values define the physical dimensions and layout of your robot arms.
// All measurements are in millimeters (mm).

/** Distance between the two shoulder motors (center to center). */
#define MOTOR_SPACING 50.0f

/** Length of the first arm segment (from motor shaft to elbow). Unit: millimeters. */
#define L1 70.0f

/** Length of the second arm segment (from elbow to end effector). Unit: millimeters. */
#define L2 100.0f

// =============================================================================
// SECTION 2: MOTOR PIN ASSIGNMENTS
// =============================================================================
// Each motor needs two GPIO pins: one for step signals and one for direction.
// GPIO numbers match the ESP32-C3 pin labels.

/**
 * MOTOR 1 (Left arm) - Controls the left side of the 5-bar mechanism.
 * This motor moves the left upper arm.
 */

/** GPIO pin that sends step pulses to motor 1 (left arm). */
#define MOTOR1_STEP_PIN 2

/** GPIO pin that controls rotation direction for motor 1 (left arm). */
#define MOTOR1_DIR_PIN 3

/**
 * MOTOR 2 (Right arm) - Controls the right side of the 5-bar mechanism.
 * This motor moves the right upper arm.
 */

/** GPIO pin that sends step pulses to motor 2 (right arm). */
#define MOTOR2_STEP_PIN 4

/** GPIO pin that controls rotation direction for motor 2 (right arm). */
#define MOTOR2_DIR_PIN 5

/**
 * MOTOR 3 (Pen lift mechanism) - Raises and lowers the pen/tool.
 * This motor has no encoder feedback; we just count steps.
 */

/** GPIO pin that sends step pulses to motor 3 (pen lift). */
#define MOTOR3_STEP_PIN 6

/** GPIO pin that controls direction for motor 3 (pen lift). */
#define MOTOR3_DIR_PIN 7

// =============================================================================
// SECTION 3: ENCODER SETTINGS
// =============================================================================
// AS5600 magnetic encoders measure the actual motor angles.
// They connect through a TCA9548A I2C multiplexer because the ESP32-C3
// doesn't have enough I2C buses for all devices.

/** I2C address of the TCA9548A multiplexer (8-channel I2C switch). */
#define TCA9548A_ADDRESS 0x70

/** I2C channel on the multiplexer where the left encoder is connected. */
#define ENCODER_LEFT_CHANNEL 0

/** I2C channel on the multiplexer where the right encoder is connected. */
#define ENCODER_RIGHT_CHANNEL 1

/**
 * Encoder zero offset variables.
 * These store the difference between the encoder's home position and true zero.
 * Positive values mean the encoder reads higher than actual angle.
 * These are runtime variables (not constants) so they can be saved/loaded.
 */

/** Offset to subtract from left encoder readings to get true angle. */
float encoderLeftZeroOffset = 0.0f;   // Placeholder: manually calibrated later
float encoderRightZeroOffset = 0.0f;  // Placeholder: manually calibrated later

// =============================================================================
// SECTION 4: I2C COMMUNICATION PINS
// =============================================================================
// I2C is a two-wire bus for communicating with encoders and the multiplexer.

/** GPIO pin used for I2C data line (SDA). */
#define I2C_SDA_PIN 8

/** GPIO pin used for I2C clock line (SCL). */
#define I2C_SCL_PIN 9

/** I2C bus speed in Hz. 400000 = 400 kHz (fast mode). */
#define I2C_FREQUENCY 400000

// =============================================================================
// SECTION 5: PEN MOTOR PARAMETERS
// =============================================================================
// The pen lift motor doesn't have an encoder, so we control it by step count.

/**
 * Number of steps needed to move the pen from fully down to fully up position.
 * The 28BYJ-48 motor has 4096 steps per full revolution, but we use a fraction
 * of that since the pen only needs to lift a few millimeters.
 */
#define PEN_STEPS_UP 400

// =============================================================================
// SECTION 6: MOTION LIMITS
// =============================================================================
// These limits prevent the robot from hitting its physical boundaries.
// Angles are in degrees.

/** Maximum angle (degrees) the left motor can rotate from vertical. */
#define LEFT_MOTOR_MAX_ANGLE 90.0f

/** Minimum angle (degrees) the left motor can rotate from vertical. */
#define LEFT_MOTOR_MIN_ANGLE -90.0f

/** Maximum angle (degrees) the right motor can rotate from vertical. */
#define RIGHT_MOTOR_MAX_ANGLE 90.0f

/** Minimum angle (degrees) the right motor can rotate from vertical. */
#define RIGHT_MOTOR_MIN_ANGLE -90.0f

/** Maximum allowed speed for any motor in degrees per second. */
#define MAX_SPEED_DEG_PER_SEC 180.0f

/** Maximum allowed acceleration in degrees per second squared. */
#define MAX_ACCELERATION_DEG_PER_SEC2 360.0f

// =============================================================================
// SECTION 7: PID CONTROLLER DEFAULTS
// =============================================================================
// PID controllers automatically adjust motor power to reach target angles.
// These are starting values; you may need to tune them for your robot.

/**
 * PID proportional gain for position control.
 * Higher values = faster response but may overshoot.
 * Start with 1.0 and adjust in small increments.
 */
#define PID_KP 1.0f

/**
 * PID integral gain for position control.
 * Higher values = eliminates steady-state error but may cause oscillation.
 * Keep at 0 for initial testing.
 */
#define PID_KI 0.0f

/**
 * PID derivative gain for position control.
 * Higher values = dampens oscillations.
 * Start with 0.1 and adjust as needed.
 */
#define PID_KD 0.1f

/** Sample rate in Hz for the PID control loop. */
#define PID_SAMPLE_RATE_HZ 100

// =============================================================================
// SECTION 8: SERIAL COMMUNICATION
// =============================================================================
// Serial is used for debugging and receiving commands from a computer.

/** Serial baud rate in bits per second. 115200 is a standard speed. */
#define SERIAL_BAUD_RATE 115200

// =============================================================================
// END OF CONFIGURATION FILE
// =============================================================================

#endif  // ROBOT_CONFIG_H       // End of the #ifndef guard block