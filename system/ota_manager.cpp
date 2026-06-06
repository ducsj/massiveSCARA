/**
 * =============================================================================
 * This is file 6/12 – OTA + WiFi Manager module.
 * =============================================================================
 * 
 * Purpose: Implementation of the OTAManager class.
 * 
 * This module provides:
 * 1. Easy WiFi setup via captive portal (no hardcoded credentials)
 * 2. Wireless firmware updates via Arduino OTA
 * 
 * Usage flow:
 * 1. Robot starts and tries to connect to saved WiFi
 * 2. If connection fails, it creates an AP called "SCARA-Robot-Setup"
 * 3. User connects phone/computer to that AP and selects their WiFi
 * 4. Robot connects and prints its IP address
 * 5. User can now upload firmware wirelessly using Arduino IDE
 * 
 * =============================================================================
 */

#include "ota_manager.h"         // Include the header file for this class

// Default password for OTA uploads (change this for production!)
#define OTA_PASSWORD "scara123"

// WiFi configuration portal timeout in seconds
// If no connection is made within this time, show config portal
#define WIFI_PORTAL_TIMEOUT 120  // 2 minutes

/**
 * Constructor - initialize member variables to safe defaults.
 */
OTAManager::OTAManager() {
    // Initialize hostname to empty string
    _hostname = "";
    
    // Initialize OTA started flag to false
    _otaStarted = false;
}

/**
 * Check if currently connected to WiFi.
 * 
 * @return true if WiFi is connected and has an IP address
 */
bool OTAManager::isConnected() {
    // WiFi.status() returns WL_CONNECTED when connected
    return WiFi.status() == WL_CONNECTED;
}

/**
 * Initialize WiFi and OTA.
 * 
 * This function sets up both WiFi connection and OTA firmware updates.
 * It's designed to be non-blocking as much as possible.
 * 
 * @param hostname  Device name for OTA and network (e.g., "SCARA-Robot")
 */
void OTAManager::begin(const char* hostname) {
    // Store the hostname for later use
    _hostname = hostname;
    
    // Print startup message
    Serial.println();
    Serial.println("===========================================");
    Serial.println("WiFi + OTA Manager Initializing...");
    Serial.println("===========================================");

    // ---- STEP 1: CONFIGURE WIFI MANAGER ----
    // Set a timeout for the configuration portal
    // If we can't connect to WiFi within this time, show the config portal
    _wifiManager.setTimeout(WIFI_PORTAL_TIMEOUT);
    
    // Set the hostname for this device
    // This appears in:
    // - Your router's DHCP client list
    // - Arduino IDE's port menu
    // - mDNS discovery (hostname.local)
    WiFi.setHostname(hostname);

    // ---- STEP 2: TRY TO CONNECT TO WIFI ----
    // WiFiManager.autoConnect() does:
    // 1. Try to connect using previously saved credentials
    // 2. If that fails, start a config portal (AP mode)
    // 3. Wait for user to select a network and enter password
    // 4. Save the credentials for next time
    // 
    // The parameter "SCARA-Robot-Setup" is the AP name when in config mode
    
    Serial.println("Attempting WiFi connection...");
    Serial.println("If this takes > 2 minutes, connect to AP: SCARA-Robot-Setup");
    
    // autoConnect() blocks while trying to connect
    // If it returns true, we connected to saved WiFi
    // If it returns false, the config portal timed out
    bool connected = _wifiManager.autoConnect("SCARA-Robot-Setup");
    
    if (connected) {
        // Success! We connected to saved WiFi
        Serial.println("Connected to WiFi!");
    } else {
        // Config portal timed out without connecting
        Serial.println("WiFi configuration timed out.");
        Serial.println("Will retry on next reboot...");
    }

    // ---- STEP 3: PRINT CONNECTION STATUS ----
    if (isConnected()) {
        // Print the IP address we received
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        
        // Print the signal strength (RSSI)
        Serial.print("Signal strength (RSSI): ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        
        // Print the network name we're connected to
        Serial.print("Connected to: ");
        Serial.println(WiFi.SSID());
    } else {
        // Not connected - WiFi will be retried on next boot
        Serial.println("WiFi not connected. OTA will not be available.");
    }

    // ---- STEP 4: SET UP ARDUINO OTA ----
    // Only set up OTA if we're connected to WiFi
    if (isConnected()) {
        Serial.println();
        Serial.println("Setting up OTA (Over-The-Air updates)...");
        
        // Set the hostname for OTA
        // This is how the device will appear in Arduino IDE
        ArduinoOTA.setHostname(hostname);
        
        // Set the password for OTA authentication
        // Without this, anyone on the network could upload firmware
        ArduinoOTA.setPassword(OTA_PASSWORD);
        
        // Set a more descriptive password message
        ArduinoOTA.setPasswordHash("scara123");
        
        // ---- SET UP OTA CALLBACKS ----
        // These functions are called at different stages of the OTA process
        
        // Called when OTA update starts
        ArduinoOTA.onStart([]() {
            Serial.println();
            Serial.println("===========================================");
            Serial.println("OTA UPDATE STARTING");
            Serial.println("===========================================");
            
            // Get the type of update (sketch or filesystem)
            if (ArduinoOTA.getCommand() == U_FLASH) {
                // Regular sketch update
                Serial.println("Updating sketch (firmware)...");
            } else {
                // Filesystem update (for storing data)
                Serial.println("Updating filesystem...");
            }
        });
        
        // Called when OTA update is in progress
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            // Calculate percentage complete
            int percent = (progress / (total / 100));
            
            // Print progress bar
            Serial.printf("OTA Progress: %d%%", percent);
            
            // Print progress bar visualization
            int barWidth = 40;
            int filled = (barWidth * percent) / 100;
            Serial.print(" [");
            for (int i = 0; i < barWidth; i++) {
                if (i < filled) {
                    Serial.print("=");
                } else {
                    Serial.print(" ");
                }
            }
            Serial.println("]");
            
            // Also print raw numbers
            Serial.printf("  %u / %u bytes\n", progress, total);
        });
        
        // Called when OTA update finishes successfully
        ArduinoOTA.onEnd([]() {
            Serial.println();
            Serial.println("===========================================");
            Serial.println("OTA UPDATE COMPLETE!");
            Serial.println("Device will restart in 1 second...");
            Serial.println("===========================================");
        });
        
        // Called when there's an OTA error
        ArduinoOTA.onError([](ota_error_t error) {
            Serial.println();
            Serial.println("===========================================");
            Serial.println("OTA UPDATE ERROR!");
            Serial.println("===========================================");
            
            // Print the specific error type
            Serial.print("Error: ");
            switch (error) {
                case OTA_AUTH_ERROR:
                    Serial.println("Authentication failed (wrong password?)");
                    break;
                case OTA_BEGIN_ERROR:
                    Serial.println("Begin failed (check flash size?)");
                    break;
                case OTA_CONNECT_ERROR:
                    Serial.println("Connection lost during transfer");
                    break;
                case OTA_RECEIVE_ERROR:
                    Serial.println("Receive error (corrupted data?)");
                    break;
                case OTA_END_ERROR:
                    Serial.println("End failed (write error?)");
                    break;
                default:
                    Serial.println("Unknown error");
            }
            
            Serial.println("Please try uploading again.");
        });
        
        // ---- START OTA ----
        // Begin listening for OTA update requests
        ArduinoOTA.begin();
        
        // Set the flag so handle() knows OTA is active
        _otaStarted = true;
        
        Serial.println("OTA is ready!");
        Serial.println();
        Serial.println("You can now upload firmware wirelessly:");
        Serial.print("  - In Arduino IDE, select Tools > Port > ");
        Serial.print(hostname);
        Serial.println(" (network)");
        Serial.print("  - Password: ");
        Serial.println(OTA_PASSWORD);
        Serial.println();
        Serial.println("===========================================");
        Serial.println("System ready!");
        Serial.println("===========================================");
    }
}

/**
 * Handle OTA requests - call this frequently in your main loop.
 * 
 * This function checks for incoming OTA update requests and processes them.
 * It's non-blocking - it returns immediately if no update is happening.
 * 
 * Call this in your main loop() function, every iteration.
 */
void OTAManager::handle() {
    // Only handle OTA if it was successfully started
    if (_otaStarted) {
        // ArduinoOTA.handle() does:
        // 1. Check if there's an OTA request waiting
        // 2. If yes, accept the connection and receive data
        // 3. Write the new firmware to flash
        // 4. Trigger a reboot when complete
        // 
        // This is non-blocking - returns quickly if no OTA activity
        ArduinoOTA.handle();
    }
    
    // Periodically check if WiFi is still connected
    // and reconnect if needed (WiFiManager handles this automatically)
    if (!isConnected() && _otaStarted) {
        // WiFi disconnected - try to reconnect
        Serial.println("WiFi disconnected, attempting reconnect...");
        
        // Reset WiFiManager and try again
        _wifiManager.autoConnect("SCARA-Robot-Setup");
        
        if (isConnected()) {
            Serial.print("Reconnected! IP: ");
            Serial.println(WiFi.localIP());
        }
    }
}