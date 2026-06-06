/**
 * =============================================================================
 * This is file 3/12 – Encoder Driver module.
 * =============================================================================
 * 
 * Purpose: Implementation of the EncoderDriver class for reading angles from
 * AS5600 magnetic encoders via the TCA9548A I2C multiplexer.
 * 
 * The AS5600 is a 12-bit encoder giving 4096 distinct positions per revolution.
 * Angles are reported as 0-4095 (or 0-360 degrees after conversion).
 * 
 * The TCA9548A is an 8-channel I2C multiplexer that lets us talk to multiple
 * devices with the same I2C address by switching between channels.
 * 
 * =============================================================================
 */

#include "encoder_driver.h"        // Include the header file for this class
#include <Wire.h>                  // Include Arduino I2C library (Wire.h)

// AS5600 encoder register addresses
// These are the internal register addresses in the AS5600 chip
#define AS5600_ADDR 0x36          // I2C address of AS5600 encoder
#define AS5600_ANGLE_L 0x0C       // Register for low byte of angle (bits 0-7)
#define AS5600_ANGLE_H 0x0D       // Register for high byte of angle (bits 8-11)

// Constructor - initialize all member variables to safe defaults
EncoderDriver::EncoderDriver() {
    // Initialize zero offsets to 0 (no calibration offset applied)
    _zeroOffset[0] = 0.0f;   // Left motor offset
    _zeroOffset[1] = 0.0f;   // Right motor offset
    
    // Initialize last known angles to 0
    _lastAngle[0] = 0.0f;    // Left motor angle
    _lastAngle[1] = 0.0f;    // Right motor angle
    
    // Initialize connection status as "not checked yet"
    _connected[0] = false;
    _connected[1] = false;
    
    // Initialize error timers to 0 (so first error prints immediately)
    _lastErrorTime[0] = 0;
    _lastErrorTime[1] = 0;
}

/**
 * Initialize the encoder driver.
 * Sets up I2C communication and checks if encoders are responding.
 */
void EncoderDriver::begin() {
    // Start I2C communication on the specified pins
    // I2C_FREQUENCY is defined in robot_config.h (400000 = 400 kHz fast mode)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQUENCY);
    
    // Give the encoders a moment to initialize after I2C starts
    delay(10);
    
    // Check each encoder to see if it responds
    // Try to read from each motor's encoder channel
    for (uint8_t motorId = 0; motorId < 2; motorId++) {
        // isConnected() will set _connected[motorId] appropriately
        if (isConnected(motorId)) {
            // Print success message to Serial monitor
            Serial.print("Encoder ");
            Serial.print(motorId == MOTOR_LEFT ? "LEFT" : "RIGHT");
            Serial.println(" (AS5600) detected!");
        } else {
            // Print error message to Serial monitor
            Serial.print("ERROR: Encoder ");
            Serial.print(motorId == MOTOR_LEFT ? "LEFT" : "RIGHT");
            Serial.println(" (AS5600) NOT detected! Check wiring.");
        }
    }
}

/**
 * Check if an encoder is responding on the I2C bus.
 * We do this by trying to read a byte from the encoder's angle register.
 * If the encoder acknowledges (ACKs) the request, it's connected.
 * 
 * @param motorId  Which motor: MOTOR_LEFT (0) or MOTOR_RIGHT (1)
 * @return true if encoder responds, false if no response
 */
bool EncoderDriver::isConnected(uint8_t motorId) {
    // First, switch to the correct multiplexer channel for this motor
    // Left motor uses channel 0, right motor uses channel 1
    selectChannel(motorId == MOTOR_LEFT ? ENCODER_LEFT_CHANNEL : ENCODER_RIGHT_CHANNEL);
    
    // Try to read 1 byte from the AS5600 angle register
    // The return value tells us if the encoder responded
    Wire.beginTransmission(AS5600_ADDR);  // Start communication with AS5600
    Wire.write(AS5600_ANGLE_L);           // Request the low byte of angle
    uint8_t error = Wire.endTransmission(); // Send and check for error
    
    // If error is 0, the encoder acknowledged (it's connected)
    // Any other value means no acknowledgment (not connected)
    _connected[motorId] = (error == 0);
    
    return _connected[motorId];
}

/**
 * Select a channel on the TCA9548A I2C multiplexer.
 * 
 * The TCA9548A is an I2C switch that connects the main I2C bus to one of
 * 8 possible channels. Each channel has its own set of SDA/SCL pins.
 * 
 * To select a channel, we send a single byte to the TCA9548A's address.
 * The byte is a bitmask where bit N being set means "enable channel N".
 * For example: 0x01 enables channel 0, 0x02 enables channel 1, etc.
 * 
 * @param ch  Channel number (0-7)
 */
void EncoderDriver::selectChannel(uint8_t ch) {
    // Ensure channel is in valid range (0-7)
    // If out of range, clamp to 0
    if (ch > 7) {
        ch = 0;
    }
    
    // Begin I2C transmission to the TCA9548A (address 0x70)
    Wire.beginTransmission(TCA9548A_ADDRESS);
    
    // Send the channel select byte
    // (1 << ch) creates a bitmask: 1<<0 = 0x01, 1<<1 = 0x02, etc.
    // Setting bit N enables channel N
    Wire.write(1 << ch);
    
    // End the transmission - this actually sends the data
    Wire.endTransmission();
}

/**
 * Read the raw angle value from the AS5600 encoder.
 * The AS5600 stores angle as a 12-bit value (0-4095) in two 8-bit registers.
 * We need to read both registers and combine them into one number.
 * 
 * @return Raw angle value from 0 to 4095
 */
uint16_t EncoderDriver::readAS5600Angle() {
    uint16_t angle = 0;  // Variable to store the combined angle value
    
    // Request 1 byte from the low byte register (bits 0-7 of angle)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ANGLE_L);           // Point to low byte register
    Wire.endTransmission(false);          // Don't release the bus (repeated start)
    Wire.requestFrom(AS5600_ADDR, 1);     // Request 1 byte from AS5600
    
    if (Wire.available()) {
        // Read the low byte and store it (this is bits 0-7)
        angle = Wire.read();
    }
    
    // Request 1 byte from the high byte register (bits 8-11 of angle)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ANGLE_H);           // Point to high byte register
    Wire.endTransmission(false);          // Don't release the bus
    Wire.requestFrom(AS5600_ADDR, 1);     // Request 1 byte
    
    if (Wire.available()) {
        // Read the high byte and shift it into position
        // The high byte contains bits 8-11, so we shift it left 8 bits
        // Then OR it with the low byte we read earlier
        angle |= (Wire.read() << 8);
    }
    
    // Mask to 12 bits (0-4095) just to be safe
    // This ensures we don't get garbage if something went wrong
    return angle & 0x0FFF;
}

/**
 * Read the raw angle from an encoder in degrees (0-360).
 * 
 * The AS5600 reports angles as 12-bit values from 0 to 4095.
 * We convert this to degrees by: degrees = (raw / 4096) * 360
 * 
 * This function automatically switches to the correct multiplexer channel
 * before reading, so you don't have to worry about channel selection.
 * 
 * @param motorId  Which motor: MOTOR_LEFT (0) or MOTOR_RIGHT (1)
 * @return Angle in degrees from 0 to 360
 */
float EncoderDriver::readAngleDegrees(uint8_t motorId) {
    // Validate motor ID - only 0 (left) and 1 (right) are valid
    // If invalid, default to left motor
    if (motorId > 1) {
        motorId = 0;
    }
    
    // Select the correct multiplexer channel for this motor
    // Left motor uses channel 0, right motor uses channel 1
    uint8_t channel = (motorId == MOTOR_LEFT) ? ENCODER_LEFT_CHANNEL : ENCODER_RIGHT_CHANNEL;
    selectChannel(channel);
    
    // Read the raw angle value (0-4095) from the AS5600
    uint16_t rawAngle = readAS5600Angle();
    
    // Convert raw angle to degrees
    // The AS5600 has 4096 distinct positions (0-4095)
    // Each position represents 360/4096 = 0.0879 degrees
    float angleDegrees = rawAngle * 360.0f / 4096.0f;
    
    // Check if we got a valid reading (non-zero means the encoder responded)
    if (rawAngle > 0) {
        // Good reading - store it as the last known good angle
        _lastAngle[motorId] = angleDegrees;
        return angleDegrees;
    } else {
        // Bad reading (encoder didn't respond or returned 0)
        // If the encoder was previously connected, print an error
        if (_connected[motorId]) {
            // Get current time for rate limiting error messages
            unsigned long now = millis();
            
            // Only print error message once every 10 seconds
            // This prevents flooding the Serial monitor
            if (now - _lastErrorTime[motorId] > 10000) {
                Serial.print("ERROR: Encoder ");
                Serial.print(motorId == MOTOR_LEFT ? "LEFT" : "RIGHT");
                Serial.println(" read failed! Using last good value.");
                _lastErrorTime[motorId] = now;  // Update timer
            }
        }
        // Return the last known good angle instead of the bad reading
        return _lastAngle[motorId];
    }
}

/**
 * Set the zero offset for a motor (for calibration).
 * 
 * When you calibrate the robot, you measure the encoder value when the
 * arm is at a known position (usually "home" or "zero"). That offset is
 * stored here and subtracted from all future readings.
 * 
 * @param motorId        Which motor: MOTOR_LEFT (0) or MOTOR_RIGHT (1)
 * @param offsetDegrees  Offset in degrees to subtract from raw reading
 */
void EncoderDriver::setZeroOffset(uint8_t motorId, float offsetDegrees) {
    // Validate motor ID - only 0 (left) and 1 (right) are valid
    if (motorId > 1) {
        return;  // Invalid motor ID, do nothing
    }
    
    // Store the offset in the array
    // This offset will be subtracted from raw readings
    _zeroOffset[motorId] = offsetDegrees;
}

/**
 * Get the calibrated angle with zero offset applied.
 * Returns raw angle minus offset, wrapped to 0-360 range.
 * 
 * The offset is applied to shift the zero point to match your calibration.
 * After applying the offset, we "wrap" the angle to stay in the 0-360 range
 * using the modulo operation.
 * 
 * Example: If raw angle is 10 degrees and offset is 5 degrees,
 * the calibrated angle is (10 - 5) = 5 degrees.
 * 
 * @param motorId  Which motor: MOTOR_LEFT (0) or MOTOR_RIGHT (1)
 * @return Calibrated angle in degrees (0-360)
 */
float EncoderDriver::getCalibratedAngle(uint8_t motorId) {
    // Validate motor ID
    if (motorId > 1) {
        motorId = 0;  // Default to left motor if invalid
    }
    
    // Get the raw angle from the encoder
    float rawAngle = readAngleDegrees(motorId);
    
    // Apply the zero offset (subtract it from the raw reading)
    float calibrated = rawAngle - _zeroOffset[motorId];
    
    // Wrap the angle to the 0-360 range
    // The modulo (%) operator gives the remainder after division
    // This ensures we stay in the 0-360 range even after subtracting offset
    while (calibrated < 0) {
        calibrated += 360.0f;  // Add 360 if negative (wrap around)
    }
    while (calibrated >= 360.0f) {
        calibrated -= 360.0f;  // Subtract 360 if too large (wrap around)
    }
    
    return calibrated;
}