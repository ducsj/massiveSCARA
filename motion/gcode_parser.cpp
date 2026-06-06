/**
 * =============================================================================
 * This is file 11/12 – G-code Parser module.
 * =============================================================================
 * 
 * Purpose: Implementation of the GcodeParser class.
 * 
 * This module parses G-code commands used by CNC machines and plotters.
 * It extracts:
 * - Target coordinates for movement
 * - Pen up/down commands
 * - Mode changes (absolute/incremental)
 * 
 * G-code Examples:
 * - G0 X10 Y20    : Rapid move to (10, 20)
 * - G1 X30 Y40 F100 : Linear move to (30, 40) at speed 100
 * - G90           : Switch to absolute positioning
 * - G91           : Switch to incremental positioning
 * - M3            : Pen down (spindle on)
 * - M5            : Pen up (spindle off)
 * 
 * =============================================================================
 */

#include "gcode_parser.h"        // Include the header file for this class

/**
 * Constructor - initialize member variables to defaults.
 */
GcodeParser::GcodeParser() {
    // Initialize to defaults - will be set properly in begin()
    _currentX = 0.0f;
    _currentY = 0.0f;
    _absoluteMode = true;   // Default to absolute positioning
    _unitsMM = true;      // Default to millimeters
    _lastError = "";
}

/**
 * Initialize the parser.
 * Call this before parsing any G-code.
 */
void GcodeParser::begin() {
    // Reset position to origin
    _currentX = 0.0f;
    _currentY = 0.0f;
    
    // Default to absolute positioning mode (G90)
    _absoluteMode = true;
    
    // Default to millimeters (G21)
    _unitsMM = true;
    
    // Clear any error
    _lastError = "";
}

/**
 * Parse a single line of G-code.
 * 
 * @param line           The G-code line to parse
 * @param x_target       OUTPUT: Target X coordinate
 * @param y_target       OUTPUT: Target Y coordinate
 * @param penDown        OUTPUT: true if pen down (M3), false if pen up (M5)
 * @param hasMovement     OUTPUT: true if line has movement command
 * @param hasPenCommand  OUTPUT: true if line has pen command
 * @return true if parsing succeeded, false on error
 */
bool GcodeParser::parseLine(String line, float& x_target, float& y_target,
                           bool& penDown, bool& hasMovement, bool& hasPenCommand) {
    // Initialize output parameters to safe defaults
    x_target = _currentX;   // Default to current position
    y_target = _currentY;
    penDown = false;        // Default pen up
    hasMovement = false;    // No movement yet
    hasPenCommand = false; // No pen command yet
    
    // Clear any previous error
    _lastError = "";
    
    // =============================================================
    // STEP 1: Clean up the input line
    // =============================================================
    
    // Remove comments (everything after semicolon)
    line = _removeComments(line);
    
    // Trim leading and trailing whitespace
    line = _trim(line);
    
    // If line is empty after cleanup, it's valid but does nothing
    if (line.length() == 0) {
        return true;
    }
    
    // =============================================================
    // STEP 2: Split line into words
    // =============================================================
    // G-code words are separated by spaces
    // Example: "G1 X30 Y40 F100" -> ["G1", "X30", "Y40", "F100"]
    
    // Find the first space to get the first word (the command)
    int spaceIndex = line.indexOf(' ');
    String firstWord = (spaceIndex == -1) ? line : line.substring(0, spaceIndex);
    
    // If there are more words, get them as parameters
    String params = (spaceIndex == -1) ? "" : line.substring(spaceIndex + 1);
    
    // =============================================================
    // STEP 3: Parse the first word (command)
    // =============================================================
    // First word format: "G1", "M3", "G90", etc.
    // First character is the letter, rest is the number
    
    if (firstWord.length() < 2) {
        _lastError = "Invalid command: " + firstWord;
        return false;
    }
    
    // Get the command letter (G, M, etc.)
    char commandLetter = firstWord.charAt(0);
    
    // Get the command number (everything after the letter)
    String numberStr = firstWord.substring(1);
    
    // Convert number string to integer
    int commandNumber = numberStr.toInt();
    
    // =============================================================
    // STEP 4: Parse coordinate parameters
    // =============================================================
    // We support X and Y coordinates
    // Format: "X30", "Y-12.5", etc.
    
    float xValue = _currentX;  // Default to current position
    float yValue = _currentY;
    bool xProvided = false;    // Did we get an X coordinate?
    bool yProvided = false;    // Did we get a Y coordinate?
    
    // Split parameters into individual words
    // Use a simple approach: find each "letter+number" pattern
    int i = 0;
    while (i < params.length()) {
        // Skip spaces
        while (i < params.length() && params.charAt(i) == ' ') {
            i++;
        }
        
        if (i >= params.length()) break;
        
        // Get the next word (until next space)
        int wordStart = i;
        while (i < params.length() && params.charAt(i) != ' ') {
            i++;
        }
        
        if (i > wordStart) {
            String word = params.substring(wordStart, i);
            
            // Parse coordinates from this word
            float value;
            
            if (_parseCoordinate(word, 'X', value)) {
                xValue = value;
                xProvided = true;
            }
            else if (_parseCoordinate(word, 'Y', value)) {
                yValue = value;
                yProvided = true;
            }
            // Ignore other letters (F for feed rate, etc.)
        }
    }
    
    // =============================================================
    // STEP 5: Process the command
    // =============================================================
    
    switch (commandLetter) {
        case 'G': {
            // G-codes for motion and modes
            switch (commandNumber) {
                case 0:
                case 1:
                    // G0: Rapid move (no feed rate)
                    // G1: Linear move (with optional feed rate)
                    // Both are treated as linear moves for our robot
                    hasMovement = true;
                    break;
                    
                case 90:
                    // G90: Absolute positioning mode
                    // All coordinates are relative to the origin
                    setAbsoluteMode(true);
                    break;
                    
                case 91:
                    // G91: Incremental positioning mode
                    // All coordinates are relative to current position
                    setAbsoluteMode(false);
                    break;
                    
                case 21:
                    // G21: Units in millimeters
                    // This is our default, so nothing to do
                    setUnitsMM();
                    break;
                    
                default:
                    // Unknown G-code - ignore but log
                    Serial.print("Warning: Unknown G-code G");
                    Serial.println(commandNumber);
                    break;
            }
            break;
        }
        
        case 'M': {
            // M-codes for miscellaneous commands
            switch (commandNumber) {
                case 3:
                case 4:
                case 5:
                    // M3/M4: Spindle on (pen down)
                    // M5: Spindle off (pen up)
                    hasPenCommand = true;
                    penDown = (commandNumber == 3 || commandNumber == 4);
                    break;
                    
                default:
                    // Unknown M-code - ignore
                    Serial.print("Warning: Unknown M-code M");
                    Serial.println(commandNumber);
                    break;
            }
            break;
        }
        
        default:
            // Unknown command letter - ignore
            break;
    }
    
    // =============================================================
    // STEP 6: Calculate target coordinates
    // =============================================================
    
    if (hasMovement) {
        if (_absoluteMode) {
            // In absolute mode, coordinates are from origin
            // Use provided values (or current if not provided)
            x_target = xProvided ? xValue : _currentX;
            y_target = yProvided ? yValue : _currentY;
        } else {
            // In incremental mode, coordinates are offsets from current position
            x_target = _currentX + (xProvided ? xValue : 0.0f);
            y_target = _currentY + (yProvided ? yValue : 0.0f);
        }
    }
    
    // Successfully parsed
    return true;
}

/**
 * Set the positioning mode (absolute or incremental).
 */
void GcodeParser::setAbsoluteMode(bool absolute) {
    _absoluteMode = absolute;
}

/**
 * Set the units to millimeters.
 */
void GcodeParser::setUnitsMM() {
    _unitsMM = true;
}

/**
 * Update the current position.
 * Call this after each move so incremental mode works correctly.
 */
void GcodeParser::setCurrentPosition(float x, float y) {
    _currentX = x;
    _currentY = y;
}

/**
 * Get the last error message.
 */
String GcodeParser::getLastError() {
    return _lastError;
}

/**
 * Get the current X position.
 */
float GcodeParser::getCurrentX() {
    return _currentX;
}

/**
 * Get the current Y position.
 */
float GcodeParser::getCurrentY() {
    return _currentY;
}

/**
 * Check if in absolute positioning mode.
 */
bool GcodeParser::isAbsoluteMode() {
    return _absoluteMode;
}

// =============================================================================
// PRIVATE HELPER METHODS
// =============================================================================

/**
 * Remove comments from a G-code line.
 * 
 * Comments start with semicolon (;)
 * Everything after the semicolon is ignored.
 */
String GcodeParser::_removeComments(String line) {
    // Find the semicolon
    int commentIndex = line.indexOf(';');
    
    // If found, return everything before it
    if (commentIndex != -1) {
        return line.substring(0, commentIndex);
    }
    
    // No comment found, return the line as-is
    return line;
}

/**
 * Trim whitespace from the beginning and end of a string.
 */
String GcodeParser::_trim(String str) {
    // Trim leading whitespace
    int start = 0;
    while (start < str.length() && isSpace(str.charAt(start))) {
        start++;
    }
    
    // Trim trailing whitespace
    int end = str.length();
    while (end > start && isSpace(str.charAt(end - 1))) {
        end--;
    }
    
    // Return the trimmed substring
    return str.substring(start, end);
}

/**
 * Parse a coordinate value from a word like "X30" or "Y-12.5".
 * 
 * @param word   Word containing a letter and number (e.g., "X30")
 * @param letter The letter to look for ('X', 'Y', etc.)
 * @param value  OUTPUT: The parsed number
 * @return true if letter was found and value parsed, false otherwise
 */
bool GcodeParser::_parseCoordinate(const String& word, char letter, float& value) {
    // Check if word starts with the expected letter
    if (word.length() < 2 || word.charAt(0) != letter) {
        return false;
    }
    
    // Get the number part (everything after the letter)
    String numberStr = word.substring(1);
    
    // Convert to float
    value = numberStr.toFloat();
    
    // Check if conversion was successful
    // toFloat() returns 0 if no valid conversion
    // We consider it an error only if the string was non-empty but didn't parse
    if (numberStr.length() > 0 && value == 0.0f) {
        // Check if it actually contains a valid number
        // by seeing if there's any digit
        bool hasDigit = false;
        for (int i = 0; i < numberStr.length(); i++) {
            if (isDigit(numberStr.charAt(i))) {
                hasDigit = true;
                break;
            }
        }
        if (!hasDigit && numberStr.indexOf('0') == -1) {
            return false;  // No valid number found
        }
    }
    
    return true;
}