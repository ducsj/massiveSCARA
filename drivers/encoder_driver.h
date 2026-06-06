/**
 * =============================================================================
 * This is file 3/12 – Encoder Driver module.
 * =============================================================================
 * 
 * Purpose: Header file for the EncoderDriver class that reads angles from
 * AS5600 magnetic encoders using the TCA9548A I2C multiplexer.
 * 
 * The AS5600 is a 12-bit encoder that reports angles from 0 to 4095 (0xFFF).
 * The TCA9548A lets us connect multiple I2C devices that share the same
 * address (like two AS5600s) by switching between 8 different channels.
 * 
 * Hardware: 2x AS5600 encoders + TCA9548A multiplexer + ESP32-C3
 * 
 * =============================================================================
 */

#ifndef ENCODER_DRIVER_H          // Prevent this file from being included multiple times
#define ENCODER_DRIVER_H           // Define the guard symbol so #ifndef knows we've loaded this file

// Include Arduino framework for Serial, delay, and types
#include <Arduino.h>

// Include the configuration file to get I2C and encoder settings
#include "../config/robot_config.h"

/**
 * Motor ID constants - used to identify which motor/encoder we're talking to.
 * These match the TCA9548A channel assignments.
 */
#define MOTOR_LEFT 0   // Left arm motor (TCA channel 0)
#define MOTOR_RIGHT 1  // Right arm motor (TCA channel 1)

/**
 * EncoderDriver class
 * 
 * Reads angle data from AS5600 magnetic encoders through a TCA9548A
 * I2C multiplexer. Each AS5600 reports angles as 12-bit values (0-4095).
 * 
 * This class handles:
 * - I2C communication setup
 * - Multiplexer channel switching
 * - Angle reading and conversion to degrees
 * - Zero offset calibration
 */
class EncoderDriver {
public:
    /**
     * Constructor - initializes member variables to safe defaults.
     */
    EncoderDriver();

    /**
     * Initialize the encoder driver.
     * Sets up I2C communication and verifies encoders are responding.
     * Prints error messages to Serial if problems are detected.
     */
    void begin();

    /**
     * Read the raw angle from an encoder in degrees (0-360).
     * Automatically switches to the correct multiplexer channel first.
     * 
     * @param motorId  Which motor to read: MOTOR_LEFT (0) or MOTOR_RIGHT (1)
     * @return Angle in degrees from 0 to 360
     */
    float readAngleDegrees(uint8_t motorId);

    /**
     * Set the zero offset for a motor (for calibration).
     * After calibration, call this with the measured offset value.
     * 
     * @param motorId        Which motor: MOTOR_LEFT (0) or MOTOR_RIGHT (1)
     * @param offsetDegrees  Offset in degrees to subtract from raw reading
     */
    void setZeroOffset(uint8_t motorId, float offsetDegrees);

    /**
     * Get the calibrated angle with zero offset applied.
     * Returns raw angle minus offset, wrapped to 0-360 range.
     * 
     * @param motorId  Which motor: MOTOR_LEFT (0) or MOTOR_RIGHT (1)
     * @return Calibrated angle in degrees (0-360)
     */
    float getCalibratedAngle(uint8_t motorId);

    /**
     * Check if an encoder is responding on the I2C bus.
     * 
     * @param motorId  Which motor: MOTOR_LEFT (0) or MOTOR_RIGHT (1)
     * @return true if encoder responds, false if no response
     */
    bool isConnected(uint8_t motorId);

private:
    /**
     * Select a channel on the TCA9548A multiplexer.
     * This tells the multiplexer which set of I2C pins to connect to the bus.
     * 
     * The TCA9548A has 8 channels (0-7). Setting channel N connects
     * the ESP32 to the I2C devices on channel N's pins.
     * 
     * @param ch  Channel number (0-7)
     */
    void selectChannel(uint8_t ch);

    /**
     * Read the raw angle value from the AS5600 encoder.
     * The AS5600 stores angle in two registers: low byte and high byte.
     * 
     * @return Raw angle value from 0 to 4095
     */
    uint16_t readAS5600Angle();

    // Zero offsets for each motor (set during calibration)
    // These are subtracted from the raw angle reading
    float _zeroOffset[2];   // Offset for left (index 0) and right (index 1) motors

    // Last good angle readings (used when reading fails)
    float _lastAngle[2];    // Last valid angle for left (index 0) and right (index 1)

    // Connection status for each encoder
    bool _connected[2];     // true if encoder responds to I2C commands

    // Timer for error message rate limiting (print only every 10 seconds)
    unsigned long _lastErrorTime[2];  // When we last printed an error for each motor
};

#endif  // ENCODER_DRIVER_H        // End of the #ifndef guard block