/**
 * =============================================================================
 * This is file 7/12 – Web Server + REST API module.
 * =============================================================================
 * 
 * Purpose: Implementation of the WebServer class.
 * 
 * This module provides a REST API for controlling the SCARA robot over WiFi.
 * It's built on ESPAsyncWebServer for non-blocking operation.
 * 
 * Endpoints:
 * - POST /move - Move to absolute (x,y) coordinates (placeholder)
 * - POST /moverel - Move relative from current position (placeholder)
 * - POST /pen - Control pen up/down (placeholder)
 * - POST /home - Run homing sequence (placeholder)
 * - POST /calibrate/encoder - Set encoder zero offset (placeholder)
 * - POST /calibrate/pid - Update PID gains (placeholder)
 * - GET /status - Get current robot state (placeholder)
 * - POST /jog - Jog X or Y axis (placeholder)
 * - POST /motor_raw - Directly step a motor (placeholder)
 * - GET /workspace - Get workspace boundary limits (placeholder)
 * 
 * Note: Placeholder functions just print messages and return success.
 * Real functionality will be added in Steps 9-10 (Kinematics + Motion Planner).
 * 
 * =============================================================================
 */

#include "web_server.h"          // Include the header file for this class

// Include LittleFS for serving static files
#include <LittleFS.h>

// Web server port
#define WEB_SERVER_PORT 80

/**
 * Constructor - initialize member variables to safe defaults.
 */
WebServer::WebServer() {
    // Create the async web server on port 80
    _server = new AsyncWebServer(WEB_PORT);
}

/**
 * Initialize the web server and set up all routes.
 */
void WebServer::begin() {
    Serial.println("===========================================");
    Serial.println("Web Server Initializing...");
    Serial.println("===========================================");

    // ---- INITIALIZE LITTLEFS ----
    // LittleFS is a filesystem for storing files in flash memory
    // We'll use it to serve the web interface files (HTML, CSS, JS)
    if (!LittleFS.begin()) {
        // If LittleFS fails to mount, print an error
        // The server will still work, but static files won't be served
        Serial.println("Warning: LittleFS mount failed!");
        Serial.println("Web interface files will not be available.");
    } else {
        Serial.println("LittleFS initialized successfully.");
        
        // List the files in LittleFS (for debugging)
        File root = LittleFS.open("/", "r");
        if (root) {
            Serial.println("Files in LittleFS:");
            while (File file = root.openNextFile()) {
                Serial.print("  - ");
                Serial.println(file.name());
            }
        }
    }

    // ---- SET UP CORS (Cross-Origin Resource Sharing) ----
    // CORS headers allow the web interface to work from any origin
    // This is needed if you serve the web files from a different domain
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

    // ---- SET UP ROUTES ----

    // Root route - serve index.html from LittleFS
    // When someone visits http://robot-ip/ they get the web interface
    _server->on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        // Try to serve index.html
        request->send(LittleFS, "/index.html", "text/html");
    });

    // Serve static files from LittleFS
    // This handles CSS, JS, images, etc.
    // The server looks for the file in LittleFS and serves it with the correct MIME type
    _server->serveStatic("/", LittleFS, "/");

    // ---- API ROUTES ----

    // GET /status - Return current robot state
    _server->on("/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /status requested");
        
        // Create a JSON response
        StaticJsonDocument<512> doc;
        JsonObject response = doc.to<JsonObject>();
        
        // Add placeholder status data
        // Real data will come from callbacks when kinematics are integrated
        response["success"] = true;
        response["message"] = "Status retrieved";
        
        // If callback is set, call it to populate real data
        if (onStatus != nullptr) {
            onStatus(response);
        } else {
            // Placeholder values
            response["x"] = 0.0f;
            response["y"] = 0.0f;
            response["angle_left"] = 0.0f;
            response["angle_right"] = 0.0f;
            response["pen_up"] = true;
            response["homed"] = false;
        }
        
        // Send the JSON response
        String jsonString;
        serializeJson(doc, jsonString);
        request->send(200, "application/json", jsonString);
    });

    // POST /move - Move to absolute (x,y) coordinates
    _server->on("/move", HTTP_POST, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /move requested");
        
        // Check if we received a body with JSON data
        if (request->hasParam("body", true)) {
            String body = request->getParam("body", true)->value();
            Serial.print("Move command: ");
            Serial.println(body);
            
            // Parse the JSON body
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                float x = doc["x"] | 0.0f;
                float y = doc["y"] | 0.0f;
                
                // Call the move callback if set
                if (onMove != nullptr) {
                    onMove(x, y, false);  // false = absolute move
                }
                
                // Send success response
                StaticJsonDocument<256> response;
                response["success"] = true;
                response["message"] = "Motion command received";
                response["target_x"] = x;
                response["target_y"] = y;
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(200, "application/json", jsonString);
            } else {
                // Invalid JSON
                StaticJsonDocument<128> response;
                response["success"] = false;
                response["message"] = "Invalid JSON format";
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(400, "application/json", jsonString);
            }
        } else {
            // No body received
            StaticJsonDocument<128> response;
            response["success"] = false;
            response["message"] = "Missing request body";
            
            String jsonString;
            serializeJson(response, jsonString);
            request->send(400, "application/json", jsonString);
        }
    });

    // POST /moverel - Move relative from current position
    _server->on("/moverel", HTTP_POST, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /moverel requested");
        
        if (request->hasParam("body", true)) {
            String body = request->getParam("body", true)->value();
            Serial.print("Move relative command: ");
            Serial.println(body);
            
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                float dx = doc["dx"] | 0.0f;
                float dy = doc["dy"] | 0.0f;
                
                if (onMove != nullptr) {
                    onMove(dx, dy, true);  // true = relative move
                }
                
                StaticJsonDocument<256> response;
                response["success"] = true;
                response["message"] = "Relative motion command received";
                response["delta_x"] = dx;
                response["delta_y"] = dy;
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(200, "application/json", jsonString);
            } else {
                StaticJsonDocument<128> response;
                response["success"] = false;
                response["message"] = "Invalid JSON format";
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(400, "application/json", jsonString);
            }
        } else {
            StaticJsonDocument<128> response;
            response["success"] = false;
            response["message"] = "Missing request body";
            
            String jsonString;
            serializeJson(response, jsonString);
            request->send(400, "application/json", jsonString);
        }
    });

    // POST /pen - Control pen up/down
    _server->on("/pen", HTTP_POST, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /pen requested");
        
        if (request->hasParam("body", true)) {
            String body = request->getParam("body", true)->value();
            Serial.print("Pen command: ");
            Serial.println(body);
            
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                // Accept "up", "down", true, false, etc.
                String stateStr = doc["state"] | "up";
                bool penUp = (stateStr == "up" || stateStr == "true" || stateStr == "1");
                
                if (onPen != nullptr) {
                    onPen(penUp);
                }
                
                StaticJsonDocument<128> response;
                response["success"] = true;
                response["message"] = penUp ? "Pen lifted" : "Pen lowered";
                response["pen_up"] = penUp;
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(200, "application/json", jsonString);
            } else {
                StaticJsonDocument<128> response;
                response["success"] = false;
                response["message"] = "Invalid JSON format";
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(400, "application/json", jsonString);
            }
        } else {
            StaticJsonDocument<128> response;
            response["success"] = false;
            response["message"] = "Missing request body";
            
            String jsonString;
            serializeJson(response, jsonString);
            request->send(400, "application/json", jsonString);
        }
    });

    // POST /home - Run homing sequence
    _server->on("/home", HTTP_POST, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /home requested");
        
        if (onHome != nullptr) {
            onHome();
        }
        
        StaticJsonDocument<128> response;
        response["success"] = true;
        response["message"] = "Homing sequence started";
        
        String jsonString;
        serializeJson(response, jsonString);
        request->send(200, "application/json", jsonString);
    });

    // POST /calibrate/encoder - Set encoder zero offset
    _server->on("/calibrate/encoder", HTTP_POST, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /calibrate/encoder requested");
        
        if (request->hasParam("body", true)) {
            String body = request->getParam("body", true)->value();
            Serial.print("Encoder calibration: ");
            Serial.println(body);
            
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, body);
            
            if (!error) {
                uint8_t motorId = doc["motor"] | 0;
                
                if (motorId <= 1 && onCalibrateEncoder != nullptr) {
                    onCalibrateEncoder(motorId);
                }
                
                StaticJsonDocument<128> response;
                response["success"] = true;
                response["message"] = "Encoder zero set for motor " + String(motorId);
                response["motor"] = motorId;
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(200, "application/json", jsonString);
            } else {
                StaticJsonDocument<128> response;
                response["success"] = false;
                response["message"] = "Invalid JSON format";
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(400, "application/json", jsonString);
            }
        } else {
            StaticJsonDocument<128> response;
            response["success"] = false;
            response["message"] = "Missing request body";
            
            String jsonString;
            serializeJson(response, jsonString);
            request->send(400, "application/json", jsonString);
        }
    });

    // POST /calibrate/pid - Update PID gains
    _server->on("/calibrate/pid", HTTP_POST, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /calibrate/pid requested");
        
        if (request->hasParam("body", true)) {
            String body = request->getParam("body", true)->value();
            Serial.print("PID calibration: ");
            Serial.println(body);
            
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
                response["message"] = "PID gains updated";
                response["kp"] = kp;
                response["ki"] = ki;
                response["kd"] = kd;
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(200, "application/json", jsonString);
            } else {
                StaticJsonDocument<128> response;
                response["success"] = false;
                response["message"] = "Invalid JSON format";
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(400, "application/json", jsonString);
            }
        } else {
            StaticJsonDocument<128> response;
            response["success"] = false;
            response["message"] = "Missing request body";
            
            String jsonString;
            serializeJson(response, jsonString);
            request->send(400, "application/json", jsonString);
        }
    });

    // POST /jog - Jog X or Y axis
    _server->on("/jog", HTTP_POST, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /jog requested");
        
        if (request->hasParam("body", true)) {
            String body = request->getParam("body", true)->value();
            Serial.print("Jog command: ");
            Serial.println(body);
            
            StaticJsonDocument<256> doc;
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
                request->send(200, "application/json", jsonString);
            } else {
                StaticJsonDocument<128> response;
                response["success"] = false;
                response["message"] = "Invalid JSON format";
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(400, "application/json", jsonString);
            }
        } else {
            StaticJsonDocument<128> response;
            response["success"] = false;
            response["message"] = "Missing request body";
            
            String jsonString;
            serializeJson(response, jsonString);
            request->send(400, "application/json", jsonString);
        }
    });

    // POST /motor_raw - Directly step a motor
    _server->on("/motor_raw", HTTP_POST, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /motor_raw requested");
        
        if (request->hasParam("body", true)) {
            String body = request->getParam("body", true)->value();
            Serial.print("Motor raw command: ");
            Serial.println(body);
            
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
                request->send(200, "application/json", jsonString);
            } else {
                StaticJsonDocument<128> response;
                response["success"] = false;
                response["message"] = "Invalid JSON format";
                
                String jsonString;
                serializeJson(response, jsonString);
                request->send(400, "application/json", jsonString);
            }
        } else {
            StaticJsonDocument<128> response;
            response["success"] = false;
            response["message"] = "Missing request body";
            
            String jsonString;
            serializeJson(response, jsonString);
            request->send(400, "application/json", jsonString);
        }
    });

    // GET /workspace - Get workspace boundary limits
    _server->on("/workspace", HTTP_GET, [this](AsyncWebServerRequest* request) {
        Serial.println("API: /workspace requested");
        
        StaticJsonDocument<512> doc;
        JsonObject response = doc.to<JsonObject>();
        
        // If callback is set, call it to populate workspace data
        if (onWorkspace != nullptr) {
            onWorkspace(response);
        } else {
            // Placeholder workspace data based on arm lengths
            // L1 = 70mm, L2 = 100mm, motor spacing = 50mm
            // Reachable area is roughly a circle with radius L1 + L2 = 170mm
            // minus the inner area blocked by motor spacing
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
        request->send(200, "application/json", jsonString);
    });

    // Handle OPTIONS requests (for CORS preflight)
    _server->on("/move", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
        request->send(200);
    });

    _server->on("/moverel", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
        request->send(200);
    });

    _server->on("/pen", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
        request->send(200);
    });

    _server->on("/home", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
        request->send(200);
    });

    _server->on("/calibrate/encoder", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
        request->send(200);
    });

    _server->on("/calibrate/pid", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
        request->send(200);
    });

    _server->on("/jog", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
        request->send(200);
    });

    _server->on("/motor_raw", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
        request->send(200);
    });

    // Handle 404 - Not Found
    _server->onNotFound([](AsyncWebServerRequest* request) {
        Serial.print("404: ");
        Serial.println(request->url());
        
        StaticJsonDocument<128> response;
        response["success"] = false;
        response["message"] = "Endpoint not found";
        response["url"] = request->url();
        
        String jsonString;
        serializeJson(response, jsonString);
        request->send(404, "application/json", jsonString);
    });

    // ---- START THE SERVER ----
    _server->begin();

    Serial.println("===========================================");
    Serial.println("Web Server Started!");
    Serial.print("Visit: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    Serial.println("===========================================");
}

/**
 * Handle client connections - call in main loop.
 * 
 * This is mainly for any periodic maintenance.
 * The AsyncWebServer is event-driven, so it handles connections
 * automatically when requests come in.
 */
void WebServer::handleClient() {
    // The AsyncWebServer handles connections asynchronously,
    // so there's nothing to do here unless you have
    // periodic tasks to run.
    // This method is here for API consistency.
}

/**
 * Create a JSON response object with standard format.
 */
void WebServer::_createResponse(JsonDocument& response, bool success, const char* message) {
    // Set the success flag
    response["success"] = success;
    
    // Set the human-readable message
    response["message"] = message;
}

/**
 * Send a JSON response to a web request.
 */
void WebServer::_sendJsonResponse(AsyncWebServerRequest* request, bool success, 
                                 const char* message, JsonObject* data) {
    // Create a JSON document for the response
    StaticJsonDocument<256> doc;
    
    // Add standard fields
    doc["success"] = success;
    doc["message"] = message;
    
    // If additional data was provided, add it to the response
    if (data != nullptr) {
        // Copy all key-value pairs from data to the response
        for (JsonPair kv : *data) {
            doc[kv.key()] = kv.value();
        }
    }
    
    // Convert to string
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Send the response
    request->send(200, "application/json", jsonString);
}