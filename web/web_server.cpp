/**
 * =============================================================================
 * This is file 7/12 – Web Server + REST API module.
 * =============================================================================
 * 
 * Purpose: Implementation of the SCARAWebServer class.
 * 
 * This module provides a REST API for controlling the SCARA robot over WiFi.
 * Uses ESP32 built-in WebServer library for compatibility.
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
 * =============================================================================
 */

#include "web_server.h"

// Include LittleFS for serving static files
#include <LittleFS.h>

/**
 * Constructor - initialize member variables to safe defaults.
 */
SCARAWebServer::SCARAWebServer() : _server(WEB_PORT) {
    // WebServer is initialized in initialization list
}

/**
 * Initialize the web server and set up all routes.
 */
void SCARAWebServer::begin() {
    Serial.println("===========================================");
    Serial.println("Web Server Initializing...");
    Serial.println("===========================================");

    // ---- INITIALIZE LITTLEFS ----
    if (!LittleFS.begin()) {
        Serial.println("Warning: LittleFS mount failed!");
    } else {
        Serial.println("LittleFS initialized successfully.");
        
        // List files in LittleFS
        File root = LittleFS.open("/", "r");
        if (root) {
            Serial.println("Files in LittleFS:");
            while (File file = root.openNextFile()) {
                Serial.print("  - ");
                Serial.println(file.name());
            }
        }
    }

    // ---- SET UP ROUTES ----

    // Root route - serve index.html from LittleFS
    _server.on("/", HTTP_GET, [this]() {
        if (LittleFS.exists("/index.html")) {
            _server.send(200, "text/html", LittleFS.open("/index.html", "r").readString());
        } else {
            _server.send(200, "text/html", "<html><body><h1>SCARA Robot</h1><p>Web interface not found. Please upload files to LittleFS.</p></body></html>");
        }
    });

    // GET /status
    _server.on("/status", HTTP_GET, [this]() {
        Serial.println("API: /status requested");
        
        StaticJsonDocument<512> doc;
        JsonObject response = doc.to<JsonObject>();
        
        response["success"] = true;
        response["message"] = "Status retrieved";
        
        if (onStatus != nullptr) {
            onStatus(response);
        } else {
            response["x"] = 0.0f;
            response["y"] = 0.0f;
            response["angle_left"] = 0.0f;
            response["angle_right"] = 0.0f;
            response["pen_up"] = true;
            response["homed"] = false;
        }
        
        String jsonString;
        serializeJson(doc, jsonString);
        _server.send(200, "application/json", jsonString);
    });

    // POST /move
    _server.on("/move", HTTP_POST, [this]() {
        Serial.println("API: /move requested");
        
        if (_server.hasArg("plain")) {
            String body = _server.arg("plain");
            Serial.print("Move command: ");
            Serial.println(body);
            
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                float x = doc["x"] | 0.0f;
                float y = doc["y"] | 0.0f;
                
                if (onMove != nullptr) {
                    onMove(x, y, false);
                }
                
                StaticJsonDocument<256> response;
                response["success"] = true;
                response["message"] = "Motion command received";
                response["target_x"] = x;
                response["target_y"] = y;
                
                String jsonString;
                serializeJson(response, jsonString);
                _server.send(200, "application/json", jsonString);
            } else {
                _server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON format\"}");
            }
        } else {
            _server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing request body\"}");
        }
    });

    // POST /moverel
    _server.on("/moverel", HTTP_POST, [this]() {
        Serial.println("API: /moverel requested");
        
        if (_server.hasArg("plain")) {
            String body = _server.arg("plain");
            Serial.print("Move relative command: ");
            Serial.println(body);
            
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                float dx = doc["dx"] | 0.0f;
                float dy = doc["dy"] | 0.0f;
                
                if (onMove != nullptr) {
                    onMove(dx, dy, true);
                }
                
                StaticJsonDocument<128> response;
                response["success"] = true;
                response["message"] = "Relative motion command received";
                response["dx"] = dx;
                response["dy"] = dy;
                
                String jsonString;
                serializeJson(response, jsonString);
                _server.send(200, "application/json", jsonString);
            } else {
                _server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON format\"}");
            }
        } else {
            _server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing request body\"}");
        }
    });

    // POST /pen
    _server.on("/pen", HTTP_POST, [this]() {
        Serial.println("API: /pen requested");
        
        if (_server.hasArg("plain")) {
            String body = _server.arg("plain");
            
            StaticJsonDocument<128> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                bool up = doc["up"] | false;
                
                if (onPen != nullptr) {
                    onPen(up);
                }
                
                StaticJsonDocument<128> response;
                response["success"] = true;
                response["message"] = up ? "Pen up" : "Pen down";
                response["pen_up"] = up;
                
                String jsonString;
                serializeJson(response, jsonString);
                _server.send(200, "application/json", jsonString);
            } else {
                _server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON format\"}");
            }
        } else {
            _server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing request body\"}");
        }
    });

    // POST /home
    _server.on("/home", HTTP_POST, [this]() {
        Serial.println("API: /home requested");
        
        if (onHome != nullptr) {
            onHome();
        }
        
        StaticJsonDocument<128> response;
        response["success"] = true;
        response["message"] = "Homing complete";
        
        String jsonString;
        serializeJson(response, jsonString);
        _server.send(200, "application/json", jsonString);
    });

    // POST /calibrate/encoder
    _server.on("/calibrate/encoder", HTTP_POST, [this]() {
        Serial.println("API: /calibrate/encoder requested");
        
        if (_server.hasArg("plain")) {
            String body = _server.arg("plain");
            
            StaticJsonDocument<128> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                uint8_t motorId = doc["motor"] | 0;
                
                if (onCalibrateEncoder != nullptr) {
                    onCalibrateEncoder(motorId);
                }
                
                StaticJsonDocument<128> response;
                response["success"] = true;
                response["message"] = "Encoder calibrated";
                response["motor"] = motorId;
                
                String jsonString;
                serializeJson(response, jsonString);
                _server.send(200, "application/json", jsonString);
            } else {
                _server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON format\"}");
            }
        } else {
            _server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing request body\"}");
        }
    });

    // POST /calibrate/pid
    _server.on("/calibrate/pid", HTTP_POST, [this]() {
        Serial.println("API: /calibrate/pid requested");
        
        if (_server.hasArg("plain")) {
            String body = _server.arg("plain");
            
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                float kp = doc["kp"] | 1.0f;
                float ki = doc["ki"] | 0.0f;
                float kd = doc["kd"] | 0.1f;
                
                if (onCalibratePID != nullptr) {
                    onCalibratePID(kp, ki, kd);
                }
                
                StaticJsonDocument<256> response;
                response["success"] = true;
                response["message"] = "PID updated";
                response["kp"] = kp;
                response["ki"] = ki;
                response["kd"] = kd;
                
                String jsonString;
                serializeJson(response, jsonString);
                _server.send(200, "application/json", jsonString);
            } else {
                _server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON format\"}");
            }
        } else {
            _server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing request body\"}");
        }
    });

    // POST /jog
    _server.on("/jog", HTTP_POST, [this]() {
        Serial.println("API: /jog requested");
        
        if (_server.hasArg("plain")) {
            String body = _server.arg("plain");
            
            StaticJsonDocument<128> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                String axis = doc["axis"] | "x";
                int steps = doc["steps"] | 10;
                
                if (onJog != nullptr) {
                    onJog(axis.c_str(), steps);
                }
                
                StaticJsonDocument<128> response;
                response["success"] = true;
                response["message"] = "Jog command executed";
                response["axis"] = axis;
                response["steps"] = steps;
                
                String jsonString;
                serializeJson(response, jsonString);
                _server.send(200, "application/json", jsonString);
            } else {
                _server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON format\"}");
            }
        } else {
            _server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing request body\"}");
        }
    });

    // POST /motor_raw
    _server.on("/motor_raw", HTTP_POST, [this]() {
        Serial.println("API: /motor_raw requested");
        
        if (_server.hasArg("plain")) {
            String body = _server.arg("plain");
            
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                uint8_t motorId = doc["motor"] | 0;
                int steps = doc["steps"] | 100;
                
                if (onMotorRaw != nullptr) {
                    onMotorRaw(motorId, steps);
                }
                
                StaticJsonDocument<128> response;
                response["success"] = true;
                response["message"] = "Motor stepped";
                response["motor"] = motorId;
                response["steps"] = steps;
                
                String jsonString;
                serializeJson(response, jsonString);
                _server.send(200, "application/json", jsonString);
            } else {
                _server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid JSON format\"}");
            }
        } else {
            _server.send(400, "application/json", "{\"success\":false,\"message\":\"Missing request body\"}");
        }
    });

    // GET /workspace
    _server.on("/workspace", HTTP_GET, [this]() {
        Serial.println("API: /workspace requested");
        
        StaticJsonDocument<512> doc;
        JsonObject response = doc.to<JsonObject>();
        
        if (onWorkspace != nullptr) {
            onWorkspace(response);
        } else {
            response["success"] = true;
            response["message"] = "Workspace boundaries";
            response["min_x"] = -120.0f;
            response["max_x"] = 120.0f;
            response["min_y"] = 0.0f;
            response["max_y"] = 170.0f;
            response["motor_spacing"] = 50.0f;
            response["arm1_length"] = 70.0f;
            response["arm2_length"] = 100.0f;
        }
        
        String jsonString;
        serializeJson(doc, jsonString);
        _server.send(200, "application/json", jsonString);
    });

    // 404 handler
    _server.onNotFound([this]() {
        Serial.print("404: ");
        Serial.println(_server.uri());
        
        StaticJsonDocument<128> response;
        response["success"] = false;
        response["message"] = "Endpoint not found";
        response["url"] = _server.uri();
        
        String jsonString;
        serializeJson(response, jsonString);
        _server.send(404, "application/json", jsonString);
    });

    // ---- START THE SERVER ----
    _server.begin();

    Serial.println("===========================================");
    Serial.println("Web Server Started!");
    Serial.print("Visit: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    Serial.println("===========================================");
}

/**
 * Handle client connections - call in main loop.
 */
void SCARAWebServer::handleClient() {
    _server.handleClient();
}

/**
 * Send a simple JSON response.
 */
void SCARAWebServer::sendJsonResponse(bool success, const char* message) {
    StaticJsonDocument<128> doc;
    doc["success"] = success;
    doc["message"] = message;
    
    String jsonString;
    serializeJson(doc, jsonString);
    _server.send(200, "application/json", jsonString);
}