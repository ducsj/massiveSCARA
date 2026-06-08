/**
 * =============================================================================
 * This is file 7/12 – Web Server + REST API module.
 * =============================================================================
 * 
 * Purpose: Header file for the SCARAWebServer class that provides a REST API
 * for controlling the SCARA robot over WiFi.
 * 
 * Features:
 * - REST API endpoints for movement, calibration, and status
 * - Serves static web files (HTML, CSS, JS) from LittleFS
 * - JSON responses for all API calls
 * - Built on ESP32 WebServer library for compatibility
 * 
 * This module allows you to:
 * - Control the robot from a web browser
 * - Send movement commands via HTTP POST
 * - Get robot status via HTTP GET
 * - Upload a custom web interface later
 * 
 * =============================================================================
 */

#ifndef WEB_SERVER_H            // Prevent this file from being included multiple times
#define WEB_SERVER_H             // Define the guard symbol so #ifndef knows we've loaded this file

// Include Arduino framework
#include <Arduino.h>

// Include ESP32 WebServer library (built-in, no external library needed)
#include <WebServer.h>

// Include ArduinoJson for parsing and creating JSON
#include <ArduinoJson.h>

/**
 * SCARAWebServer class
 * 
 * Provides a web interface and REST API for the SCARA robot.
 * 
 * Endpoints:
 * - POST /move - Move to absolute (x,y) coordinates
 * - POST /moverel - Move relative from current position
 * - POST /pen - Control pen up/down
 * - POST /home - Run homing sequence
 * - POST /calibrate/encoder - Set encoder zero offset
 * - POST /calibrate/pid - Update PID gains
 * - GET /status - Get current robot state
 * - POST /jog - Jog X or Y axis
 * - POST /motor_raw - Directly step a motor
 * - GET /workspace - Get workspace boundary limits
 * 
 * Also serves static files from LittleFS (HTML, CSS, JS).
 */
class SCARAWebServer {
public:
    /**
     * Constructor - creates a new SCARAWebServer object.
     */
    SCARAWebServer();

    /**
     * Initialize the web server.
     * 
     * This sets up:
     * 1. LittleFS filesystem for static files
     * 2. All REST API endpoints
     * 3. CORS headers for cross-origin requests
     * 
     * The server listens on port 80 (standard HTTP).
     */
    void begin();

    /**
     * Handle client connections - call in main loop.
     */
    void handleClient();

    // =======================================================================
    // Callbacks for robot integration (set these from main code)
    // =======================================================================
    
    /**
     * Callback function type for move commands.
     * Will be called when /move or /moverel endpoint is received.
     */
    typedef void (*MoveCallback)(float x, float y, bool relative);

    /**
     * Callback function type for pen control.
     * Will be called when /pen endpoint is received.
     */
    typedef void (*PenCallback)(bool up);

    /**
     * Callback function type for homing.
     * Will be called when /home endpoint is received.
     */
    typedef void (*HomeCallback)();

    /**
     * Callback function type for encoder calibration.
     * Will be called when /calibrate/encoder endpoint is received.
     */
    typedef void (*CalibrateEncoderCallback)(uint8_t motorId);

    /**
     * Callback function type for PID calibration.
     * Will be called when /calibrate/pid endpoint is received.
     */
    typedef void (*CalibratePIDCallback)(float kp, float ki, float kd);

    /**
     * Callback function type for status request.
     * Will be called when /status endpoint is received.
     */
    typedef void (*StatusCallback)(JsonObject& response);

    /**
     * Callback function type for jogging.
     * Will be called when /jog endpoint is received.
     */
    typedef void (*JogCallback)(const char* axis, int steps);

    /**
     * Callback function type for raw motor control.
     * Will be called when /motor_raw endpoint is received.
     */
    typedef void (*MotorRawCallback)(uint8_t motorId, int steps);

    /**
     * Callback function type for workspace bounds.
     * Will be called when /workspace endpoint is received.
     */
    typedef void (*WorkspaceCallback)(JsonObject& response);

    // Callback function pointers (set these from main code)
    MoveCallback onMove = nullptr;
    PenCallback onPen = nullptr;
    HomeCallback onHome = nullptr;
    CalibrateEncoderCallback onCalibrateEncoder = nullptr;
    CalibratePIDCallback onCalibratePID = nullptr;
    StatusCallback onStatus = nullptr;
    JogCallback onJog = nullptr;
    MotorRawCallback onMotorRaw = nullptr;
    WorkspaceCallback onWorkspace = nullptr;

private:
    // The web server instance (using ESP32 built-in WebServer)
    WebServer _server;

    // Port number for the web server
    static const int WEB_PORT = 80;

    // Handler functions for API endpoints
    void handleStatus();
    void handleMove();
    void handleMoveRel();
    void handlePen();
    void handleHome();
    void handleCalibrateEncoder();
    void handleCalibratePID();
    void handleJog();
    void handleMotorRaw();
    void handleWorkspace();
    void handleNotFound();

    // Helper to send JSON response
    void sendJsonResponse(bool success, const char* message);
};

#endif  // WEB_SERVER_H           // End of the #ifndef guard block