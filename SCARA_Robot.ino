/**
 * =============================================================================
 * This is file 12/12 – Main Integration sketch.
 * =============================================================================
 * 
 * Purpose: Main program that integrates all SCARA robot modules into a
 * working robot system.
 * 
 * This sketch brings together:
 * - Motor drivers (open-loop stepper control)
 * - Encoder drivers (position feedback)
 * - PID controllers (smooth motion)
 * - Closed-loop motors (combining motor + encoder + PID)
 * - Inverse kinematics (X,Y to angle conversion)
 * - Motion planner (smooth trajectories)
 * - G-code parser (CNC command interpretation)
 * - Web server (remote control interface)
 * - OTA manager (wireless firmware updates)
 * 
 * Hardware Configuration (from robot_config.h):
 * - ESP32-C3 SuperMini
 * - 2x AS5600 encoders via TCA9548A I2C multiplexer
 * - 3x 28BYJ-48 stepper motors with ULN2003A drivers
 * - TCRT5000 homing sensor (not used in this version)
 * 
 * Author: SCARA Robot Project
 * Version: 1.0
 * 
 * =============================================================================
 */

// =============================================================================
// INCLUDE ALL MODULE HEADERS
// =============================================================================
// These includes bring in all the classes and functions we need

// Robot configuration (pins, dimensions, limits)
#include "config/robot_config.h"

// Motor driver for stepper motor control
#include "drivers/motor_driver.h"

// Encoder driver for reading AS5600 magnetic encoders
#include "drivers/encoder_driver.h"

// PID controller for smooth motion
#include "control/pid_controller.h"

// Closed-loop motor (combines motor + encoder + PID)
#include "control/closed_loop_motor.h"

// OTA manager for wireless firmware updates
#include "system/ota_manager.h"

// Web server for remote control interface
#include "web/web_server.h"

// Inverse kinematics (X,Y to motor angles)
#include "kinematics/inverse_kinematics.h"

// Motion planner for smooth trajectories
#include "motion/motion_planner.h"

// G-code parser for CNC commands
#include "motion/gcode_parser.h"

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================
// These objects are accessible from anywhere in the program

/** Encoder driver - reads positions from AS5600 sensors */
EncoderDriver encoder;

/** Left arm motor controller */
ClosedLoopMotor leftMotor;

/** Right arm motor controller */
ClosedLoopMotor rightMotor;

/** Pen lift motor (open-loop control) */
MotorDriver penMotor;

/** OTA/WiFi manager for wireless updates */
OTAManager ota;

/** Web server for HTTP API and web interface */
WebServer webServer;

/** Inverse kinematics calculator */
InverseKinematics ik;

/** Motion trajectory planner */
MotionPlanner planner;

/** G-code command parser */
GcodeParser parser;

// =============================================================================
// GLOBAL STATE VARIABLES
// =============================================================================
// These track the current state of the robot

/** Current X position in millimeters (relative to center) */
float currentX = 0.0f;

/** Current Y position in millimeters (relative to center) */
float currentY = 0.0f;

/** Current left motor angle in degrees (from encoder) */
float currentLeftAngle = 0.0f;

/** Current right motor angle in degrees (from encoder) */
float currentRightAngle = 0.0f;

/** Pen state: true = down (drawing), false = up */
bool penDown = false;

/** Whether the robot has been homed (calibrated at reference position) */
bool homed = true;  // Set true - assume manually homed at startup

/** Whether the robot is currently moving */
bool moving = false;

/** Last time we updated the motors (milliseconds) */
unsigned long lastMotorUpdate = 0;

/** How often to update motors (milliseconds) */
const unsigned long motorUpdateInterval = 10;  // 10ms = 100 Hz

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
// These functions are defined later in the file but used before they're defined

/**
 * Move the robot to an absolute X,Y position.
 * Uses inverse kinematics to calculate motor angles, then motion planner
 * to generate a smooth trajectory.
 * 
 * @param x  Target X position in millimeters
 * @param y  Target Y position in millimeters
 */
void moveToXY(float x, float y);

/**
 * Update all motors - call this frequently in the main loop.
 * This updates the closed-loop motor controllers which generate step pulses.
 */
void updateMotors();

/**
 * Process a single line of G-code.
 * Parses the command and triggers movement or pen control.
 * 
 * @param line  A single line of G-code to process
 */
void processGcodeLine(String line);

/**
 * Set the pen up or down.
 * Controls the pen lift motor to raise or lower the tool.
 * 
 * @param down  true = pen down (drawing), false = pen up
 */
void setPen(bool down);

// =============================================================================
// WEB SERVER CALLBACK FUNCTIONS
// =============================================================================
// These are called by the web server when API endpoints are accessed

/**
 * Callback: Handle move command from web interface.
 * Called when POST /move or POST /moverel is received.
 * 
 * @param x       Target X position in mm
 * @param y       Target Y position in mm
 * @param relative  true = relative move, false = absolute move
 */
void handleMoveCallback(float x, float y, bool relative) {
    if (relative) {
        // Relative move: add to current position
        moveToXY(currentX + x, currentY + y);
    } else {
        // Absolute move: go directly to position
        moveToXY(x, y);
    }
}

/**
 * Callback: Handle pen control from web interface.
 * Called when POST /pen is received.
 * 
 * @param up  true = pen up, false = pen down
 */
void handlePenCallback(bool up) {
    setPen(!up);  // Note: up=true means pen is up (not down)
}

/**
 * Callback: Handle homing command from web interface.
 * Called when POST /home is received.
 * For now, just sets homed flag - real homing would use sensors.
 */
void handleHomeCallback() {
    // Reset position to center
    currentX = 0.0f;
    currentY = 50.0f;  // Assume Y=50 is home position
    currentLeftAngle = 0.0f;
    currentRightAngle = 0.0f;
    homed = true;
    
    // Set encoder offsets to current positions (calibration)
    encoder.setZeroOffset(0, currentLeftAngle);
    encoder.setZeroOffset(1, currentRightAngle);
    
    Serial.println("Homing complete!");
}

/**
 * Callback: Handle encoder calibration from web interface.
 * Called when POST /calibrate/encoder is received.
 * Sets the current encoder reading as zero degrees.
 * 
 * @param motorId  Motor ID: 0 = left, 1 = right
 */
void handleCalibrateEncoderCallback(uint8_t motorId) {
    if (motorId <= 1) {
        float currentAngle = encoder.getCalibratedAngle(motorId);
        encoder.setZeroOffset(motorId, currentAngle);
        Serial.print("Encoder ");
        Serial.print(motorId == 0 ? "LEFT" : "RIGHT");
        Serial.print(" zero set to: ");
        Serial.println(currentAngle);
    }
}

/**
 * Callback: Handle PID tuning from web interface.
 * Called when POST /calibrate/pid is received.
 * Updates the PID gains for both motors.
 * 
 * @param kp  Proportional gain
 * @param ki  Integral gain
 * @param kd  Derivative gain
 */
void handleCalibratePIDCallback(float kp, float ki, float kd) {
    leftMotor.setPIDGains(kp, ki, kd);
    rightMotor.setPIDGains(kp, ki, kd);
    Serial.print("PID updated: Kp=");
    Serial.print(kp);
    Serial.print(", Ki=");
    Serial.print(ki);
    Serial.print(", Kd=");
    Serial.println(kd);
}

/**
 * Callback: Handle status request from web interface.
 * Called when GET /status is received.
 * Populates the JSON response with current robot state.
 * 
 * @param response  JSON object to populate with status data
 */
void handleStatusCallback(JsonObject& response) {
    response["x"] = currentX;
    response["y"] = currentY;
    response["angle_left"] = currentLeftAngle;
    response["angle_right"] = currentRightAngle;
    response["pen_up"] = !penDown;
    response["homed"] = homed;
    response["moving"] = moving;
}

/**
 * Callback: Handle jog command from web interface.
 * Called when POST /jog is received.
 * Moves the robot a small amount in X or Y direction.
 * 
 * @param axis   Which axis: "x" or "y"
 * @param steps  Number of steps (in mm, converted to position)
 */
void handleJogCallback(const char* axis, int steps) {
    float deltaX = 0.0f;
    float deltaY = 0.0f;
    
    if (strcmp(axis, "x") == 0) {
        deltaX = steps;
    } else if (strcmp(axis, "y") == 0) {
        deltaY = steps;
    }
    
    // Move relative from current position
    moveToXY(currentX + deltaX, currentY + deltaY);
}

/**
 * Callback: Handle raw motor command from web interface.
 * Called when POST /motor_raw is received.
 * Steps a motor directly for testing purposes.
 * 
 * @param motorId  Motor ID: 0 = left, 1 = right
 * @param steps    Number of steps (positive or negative)
 */
void handleMotorRawCallback(uint8_t motorId, int steps) {
    // This is for testing - step the motor directly
    // In a full implementation, this would use a test function
    Serial.print("Motor ");
    Serial.print(motorId);
    Serial.print(" raw step: ");
    Serial.println(steps);
}

/**
 * Callback: Handle workspace bounds request.
 * Called when GET /workspace is received.
 * Returns the robot's physical workspace limits.
 * 
 * @param response  JSON object to populate with workspace data
 */
void handleWorkspaceCallback(JsonObject& response) {
    float motorSpacing, l1, l2;
    ik.getGeometry(motorSpacing, l1, l2);
    
    response["min_x"] = -(l1 + l2 - motorSpacing / 2);
    response["max_x"] = (l1 + l2 - motorSpacing / 2);
    response["min_y"] = 0.0f;
    response["max_y"] = l1 + l2;
    response["motor_spacing"] = motorSpacing;
    response["arm1_length"] = l1;
    response["arm2_length"] = l2;
}

// =============================================================================
// SETUP FUNCTION
// =============================================================================
// This runs once when the robot powers on or resets

void setup() {
    // Start serial communication for debugging
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Wait for serial to connect (helps with some USB-serial adapters)
    delay(1000);
    
    // Print startup banner
    Serial.println();
    Serial.println("===========================================");
    Serial.println("  SCARA Robot - Initializing...");
    Serial.println("===========================================");
    
    // Initialize I2C communication for encoders
    // Using pins defined in robot_config.h (SDA=8, SCL=9)
    Serial.println("Initializing I2C...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_FREQUENCY);
    
    // Initialize encoder driver
    Serial.println("Initializing encoders...");
    encoder.begin();
    
    // Initialize closed-loop motors with encoder feedback
    Serial.println("Initializing motors...");
    
    // Left motor: motor ID 0, step pin 2, dir pin 3, 15 RPM, encoder pointer
    leftMotor.begin(0, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 15, &encoder);
    leftMotor.setPIDGains(PID_KP, PID_KI, PID_KD);
    
    // Right motor: motor ID 1, step pin 4, dir pin 5, 15 RPM, encoder pointer
    rightMotor.begin(1, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 15, &encoder);
    rightMotor.setPIDGains(PID_KP, PID_KI, PID_KD);
    
    // Initialize pen motor (open-loop, no encoder)
    penMotor.begin(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, 15);
    penMotor.moveTo(0);  // Start with pen up
    
    // Set initial target angles (current position)
    currentLeftAngle = encoder.getCalibratedAngle(0);
    currentRightAngle = encoder.getCalibratedAngle(1);
    leftMotor.setTargetAngle(currentLeftAngle);
    rightMotor.setTargetAngle(currentRightAngle);
    
    // Initialize inverse kinematics with robot geometry
    Serial.println("Initializing kinematics...");
    ik.setGeometry(MOTOR_SPACING, L1, L2);
    
    // Initialize G-code parser
    parser.begin();
    
    // Start OTA and WiFi manager
    // This will connect to WiFi or start config portal
    Serial.println("Starting WiFi and OTA...");
    ota.begin("SCARA-Robot");
    
    // Start web server for remote control
    Serial.println("Starting web server...");
    webServer.begin();
    
    // Connect web server callbacks to our handler functions
    webServer.onMove = handleMoveCallback;
    webServer.onPen = handlePenCallback;
    webServer.onHome = handleHomeCallback;
    webServer.onCalibrateEncoder = handleCalibrateEncoderCallback;
    webServer.onCalibratePID = handleCalibratePIDCallback;
    webServer.onStatus = handleStatusCallback;
    webServer.onJog = handleJogCallback;
    webServer.onMotorRaw = handleMotorRawCallback;
    webServer.onWorkspace = handleWorkspaceCallback;
    
    // Initialize motion planner (no active motion)
    planner.reset();
    
    // Assume robot is homed at startup
    // In a real system, you'd run a homing sequence here
    homed = true;
    currentY = 50.0f;  // Default home position
    
    // Print completion message
    Serial.println();
    Serial.println("===========================================");
    Serial.println("  SCARA Robot Ready!");
    Serial.println("===========================================");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  G0 X30 Y40   - Move to position (30, 40)");
    Serial.println("  M3           - Pen down");
    Serial.println("  M5           - Pen up");
    Serial.println();
    Serial.print("Web interface: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    Serial.println("===========================================");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
// This runs continuously after setup() completes

void loop() {
    // Handle OTA firmware updates (non-blocking)
    ota.handle();
    
    // Handle web server client connections (non-blocking)
    webServer.handleClient();
    
    // Update motor positions (call frequently for smooth motion)
    updateMotors();
    
    // Update current angle readings from encoders
    currentLeftAngle = leftMotor.getCurrentAngle();
    currentRightAngle = rightMotor.getCurrentAngle();
    
    // Process any motion from the planner
    if (moving && planner.isMoving()) {
        // Get next position from motion planner
        float targetAngle1, targetAngle2;
        if (planner.nextStep(targetAngle1, targetAngle2)) {
            // Set new target angles for motors
            leftMotor.setTargetAngle(targetAngle1);
            rightMotor.setTargetAngle(targetAngle2);
        } else {
            // Motion complete
            moving = false;
            Serial.println("Motion complete!");
            
            // Update current position from inverse kinematics
            float leftAngle, rightAngle;
            if (ik.computeAngles(currentX, currentY, leftAngle, rightAngle)) {
                // Position is valid
            }
        }
    }
    
    // Process serial input for debugging/G-code
    // This allows typing G-code commands directly in Serial Monitor
    if (Serial.available() > 0) {
        // Read a line from serial
        String line = Serial.readStringUntil('\n');
        line.trim();  // Remove whitespace
        
        if (line.length() > 0) {
            Serial.print("> ");
            Serial.println(line);
            processGcodeLine(line);
        }
    }
    
    // Small delay to prevent watchdog timer issues
    delay(1);
}

// =============================================================================
// MOVE TO X,Y FUNCTION
// =============================================================================

void moveToXY(float x, float y) {
    // Calculate target motor angles using inverse kinematics
    float targetLeftAngle, targetRightAngle;
    
    // Try to compute angles for the target position
    if (!ik.computeAngles(x, y, targetLeftAngle, targetRightAngle, true)) {
        // Position is out of reach!
        Serial.print("ERROR: Position (");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.println(") is out of reach!");
        return;
    }
    
    Serial.print("Moving to (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(") -> Left: ");
    Serial.print(targetLeftAngle);
    Serial.print(", Right: ");
    Serial.print(targetRightAngle);
    Serial.println();
    
    // Get current angles from motors
    float startLeftAngle = leftMotor.getCurrentAngle();
    float startRightAngle = rightMotor.getCurrentAngle();
    
    // Plan the motion trajectory
    // Parameters: start angles, target angles, max speed, max acceleration
    planner.plan(startLeftAngle, startRightAngle, 
                 targetLeftAngle, targetRightAngle,
                 MAX_SPEED_DEG_PER_SEC, MAX_ACCELERATION_DEG_PER_SEC2);
    
    // Mark as moving
    moving = true;
    
    // Update stored position
    currentX = x;
    currentY = y;
}

// =============================================================================
// UPDATE MOTORS FUNCTION
// =============================================================================

void updateMotors() {
    // Call update on all motors
    // These generate step pulses based on PID control
    leftMotor.update();
    rightMotor.update();
    penMotor.update();
}

// =============================================================================
// PROCESS G-CODE LINE FUNCTION
// =============================================================================

void processGcodeLine(String line) {
    // Parse the G-code line
    float x_target, y_target;
    bool penDownCmd, hasMovement, hasPen;
    
    if (parser.parseLine(line, x_target, y_target, penDownCmd, hasMovement, hasPen)) {
        // Check if there's a pen command
        if (hasPen) {
            setPen(penDownCmd);
        }
        
        // Check if there's a movement command
        if (hasMovement) {
            moveToXY(x_target, y_target);
        }
        
        // Update parser's current position
        if (hasMovement) {
            parser.setCurrentPosition(x_target, y_target);
        }
    } else {
        // Parse error
        Serial.print("Parse error: ");
        Serial.println(parser.getLastError());
    }
}

// =============================================================================
// SET PEN FUNCTION
// =============================================================================

void setPen(bool down) {
    if (down != penDown) {
        // State changed, move the pen
        penDown = down;
        
        if (penDown) {
            // Pen down - move motor to extend pen
            Serial.println("Pen DOWN");
            penMotor.moveTo(PEN_STEPS_UP);  // Assuming PEN_STEPS_UP lowers the pen
        } else {
            // Pen up - move motor to retract pen
            Serial.println("Pen UP");
            penMotor.moveTo(0);
        }
    }
}