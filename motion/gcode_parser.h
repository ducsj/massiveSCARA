/**
 * =============================================================================
 * This is file 11/12 – G-code Parser module.
 * =============================================================================
 * 
 * Purpose: Header file for the GcodeParser class that parses G-code commands
 * and extracts movement targets and pen commands.
 * 
 * What is G-code?
 * 
 * G-code is a language used to control CNC machines, 3D printers, and robots.
 * It consists of commands like:
 * - G0 X10 Y20: Move to position (10, 20)
 * - G1 X30 Y40 F100: Move to (30, 40) at speed 100
 * - M3: Turn something on (like pen down)
 * - M5: Turn something off (like pen up)
 * 
 * This parser doesn't execute the commands - it just extracts the information
 * so the main program can pass it to the motion planner.
 * 
 * Supported commands:
 * - G0/G1: Linear move to coordinates
 * - G90: Absolute positioning mode
 * - G91: Incremental positioning mode
 * - G21: Millimeters (our default)
 * - M3: Pen down (spindle on)
 * - M5: Pen up (spindle off)
 * 
 * =============================================================================
 */

#ifndef GCODE_PARSER_H           // Prevent this file from being included multiple times
#define GCODE_PARSER_H            // Define the guard symbol so #ifndef knows we've loaded this file

// Include Arduino framework for String type
#include <Arduino.h>

/**
 * GcodeParser class
 * 
 * Parses individual lines of G-code and extracts:
 * - Target coordinates (X, Y)
 * - Whether to move
 * - Whether to change pen state
 * 
 * Usage:
 * 1. Call begin() to initialize
 * 2. Call parseLine() for each line of G-code
 * 3. If hasMovement is true, use x_target and y_target
 * 4. If hasPenCommand is true, use penDown to control pen
 * 
 * The parser maintains state between lines:
 * - Current position (for incremental mode)
 * - Absolute/incremental mode
 * - This allows it to handle commands that don't specify all coordinates
 */
class GcodeParser {
public:
    /**
     * Constructor - creates a new GcodeParser object.
     */
    GcodeParser();

    /**
     * Initialize the parser.
     * 
     * Call this before parsing any G-code.
     * Sets up default state (absolute mode, mm units).
     */
    void begin();

    /**
     * Parse a single line of G-code.
     * 
     * Extracts target coordinates and commands from a line of G-code.
     * 
     * @param line           The G-code line to parse
     * @param x_target       OUTPUT: Target X coordinate (meaningful if hasMovement=true)
     * @param y_target       OUTPUT: Target Y coordinate (meaningful if hasMovement=true)
     * @param penDown        OUTPUT: true if pen down (M3), false if pen up (M5)
     * @param hasMovement    OUTPUT: true if line contains G0 or G1 command
     * @param hasPenCommand  OUTPUT: true if line contains M3 or M5 command
     * 
     * @return true if parsing succeeded, false if there was an error
     *         Use getLastError() to get the error message.
     */
    bool parseLine(String line, float& x_target, float& y_target,
                   bool& penDown, bool& hasMovement, bool& hasPenCommand);

    /**
     * Set the positioning mode (absolute or incremental).
     * 
     * @param absolute  true for G90 (absolute), false for G91 (incremental)
     */
    void setAbsoluteMode(bool absolute);

    /**
     * Set the units to millimeters.
     * 
     * G21 command - this is our default, so this is mostly for completeness.
     */
    void setUnitsMM();

    /**
     * Update the current position.
     * 
     * Call this after each move so incremental mode works correctly.
     * 
     * @param x  Current X position in mm
     * @param y  Current Y position in mm
     */
    void setCurrentPosition(float x, float y);

    /**
     * Get the last error message.
     * 
     * @return Error message from the most recent parseLine() call
     */
    String getLastError();

    /**
     * Get the current X position.
     * 
     * @return Current X position in mm
     */
    float getCurrentX();

    /**
     * Get the current Y position.
     * 
     * @return Current Y position in mm
     */
    float getCurrentY();

    /**
     * Check if in absolute positioning mode.
     * 
     * @return true if absolute mode (G90), false if incremental (G91)
     */
    bool isAbsoluteMode();

private:
    // =======================================================================
    // PARSER STATE
    // =======================================================================

    /** Current X position in mm */
    float _currentX;

    /** Current Y position in mm */
    float _currentY;

    /** true = absolute mode (G90), false = incremental mode (G91) */
    bool _absoluteMode;

    /** Units are millimeters (G21) - we only support mm */
    bool _unitsMM;

    /** Last error message */
    String _lastError;

    // =======================================================================
    // HELPER METHODS
    // =======================================================================

    /**
     * Remove comments from a G-code line.
     * 
     * Comments can be:
     * - After a semicolon (; everything after is a comment)
     * - Between parentheses (( ... ) - but we keep it simple and ignore ;)
     * 
     * @param line  Input line
     * @return Line with comments removed
     */
    String _removeComments(String line);

    /**
     * Trim whitespace from the beginning and end of a string.
     * 
     * @param str  Input string
     * @return Trimmed string
     */
    String _trim(String str);

    /**
     * Parse a coordinate value from a word like "X30" or "Y-12.5".
     * 
     * @param word  Word containing a letter and number (e.g., "X30")
     * @param letter  The letter to look for ('X', 'Y', etc.)
     * @param value  OUTPUT: The parsed number
     * @return true if letter was found and value parsed, false otherwise
     */
    bool _parseCoordinate(const String& word, char letter, float& value);
};

#endif  // GCODE_PARSER_H          // End of the #ifndef guard block