/**
 * =============================================================================
 * This is file 6/12 – OTA + WiFi Manager module.
 * =============================================================================
 * 
 * Purpose: Header file for the OTAManager class that handles:
 * 1. WiFi connection management using WiFiManager (captive portal for easy setup)
 * 2. Over-The-Air (OTA) firmware updates via Arduino IDE or esptool
 * 
 * With this module, you can:
 * - Connect the robot to WiFi without hardcoding credentials
 * - Update firmware wirelessly over WiFi
 * - Access the robot's Serial output remotely
 * 
 * First-time setup: When the robot can't connect to WiFi, it creates an
 * access point named "SCARA-Robot-Setup" where you can select your network.
 * 
 * =============================================================================
 */

#ifndef OTA_MANAGER_H           // Prevent this file from being included multiple times
#define OTA_MANAGER_H            // Define the guard symbol so #ifndef knows we've loaded this file

// Include Arduino framework for String type
#include <Arduino.h>

// WiFi library (built into ESP32 core)
#include <WiFi.h>

// Arduino OTA library (built into ESP32 core)
#include <ArduinoOTA.h>

// WiFiManager library (by tzapu) - handles WiFi credentials and captive portal
// Install via Arduino Library Manager: search for "WiFiManager by tzapu"
#include <WiFiManager.h>

/**
 * OTAManager class
 * 
 * This class handles two things:
 * 1. WiFi Connection - Uses WiFiManager to connect to WiFi or create an AP
 * 2. OTA Updates - Allows firmware updates over WiFi
 * 
 * How WiFiManager works:
 * - First, it tries to connect using saved credentials
 * - If that fails, it starts a configuration portal (access point)
 * - Users connect to the AP and select their WiFi network
 * - Credentials are saved automatically for next time
 * 
 * How OTA works:
 * - After connecting to WiFi, ArduinoOTA listens for upload requests
 * - You can upload new firmware from Arduino IDE (same as USB upload)
 * - Requires password for security (default: "scara123")
 */
class OTAManager {
public:
    /**
     * Constructor - creates a new OTAManager object.
     */
    OTAManager();

    /**
     * Initialize WiFi and OTA.
     * 
     * This function:
     * 1. Starts WiFiManager
     * 2. Tries to connect to saved WiFi (or starts config portal)
     * 3. Prints IP address when connected
     * 4. Sets up ArduinoOTA for wireless firmware updates
     * 
     * @param hostname  Name for this device on the network (e.g., "SCARA-Robot")
     *                   This name will appear in Arduino IDE's port menu.
     */
    void begin(const char* hostname);

    /**
     * Handle OTA requests - call this frequently in your main loop.
     * 
     * This checks for incoming OTA update requests and processes them.
     * It's non-blocking - returns immediately if no update is happening.
     */
    void handle();

private:
    /**
     * Check if currently connected to WiFi.
     * 
     * @return true if WiFi is connected
     */
    bool isConnected();

    // WiFiManager instance - handles WiFi connection and credential storage
    WiFiManager _wifiManager;

    // Hostname for this device (used for OTA and mDNS)
    String _hostname;

    // Flag to track if OTA has been started
    bool _otaStarted;
};

#endif  // OTA_MANAGER_H          // End of the #ifndef guard block