/**
 * =============================================================================
 * massiveSCARA - 5-Bar Parallel SCARA Robot Control Firmware
 * =============================================================================
 * 
 * Hardware: ESP32-C3 SuperMini
 * - 2x AS5600 magnetic encoders via TCA9548A I2C multiplexer
 * - 3x 28BYJ-48 stepper motors with ULN2003A drivers
 * - Closed-loop position control with PID
 * - WiFi + OTA for wireless updates
 * - Web interface for remote control
 * 
 * =============================================================================
 */

// =============================================================================
// INCLUDES
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Wire.h>

// =============================================================================
// CONFIGURATION
// =============================================================================
#define MOTOR_SPACING 50.0f
#define L1 70.0f
#define L2 100.0f
#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 3
#define MOTOR2_STEP_PIN 4
#define MOTOR2_DIR_PIN 5
#define MOTOR3_STEP_PIN 6
#define MOTOR3_DIR_PIN 7
#define TCA9548A_ADDRESS 0x70
#define ENCODER_LEFT_CHANNEL 0
#define ENCODER_RIGHT_CHANNEL 1
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define I2C_FREQUENCY 400000
#define PEN_STEPS_UP 400
#define MAX_SPEED_DEG_PER_SEC 180.0f
#define MAX_ACCELERATION_DEG_PER_SEC2 360.0f
#define PID_KP 1.0f
#define PID_KI 0.0f
#define PID_KD 0.1f
#define SERIAL_BAUD_RATE 115200

// =============================================================================
// PID CONTROLLER CLASS
// =============================================================================
class PIDController {
private:
    float _kp, _ki, _kd;
    float _target;
    float _lastInput;
    float _integral;
    float _lastError;
    unsigned long _lastTime;
    float _outputMin, _outputMax;
    unsigned long _sampleTimeMs;

    float constrainValue(float value, float minVal, float maxVal) {
        if (value < minVal) return minVal;
        if (value > maxVal) return maxVal;
        return value;
    }

public:
    PIDController() {
        _kp = _ki = _kd = 0.0f;
        _target = 0.0f;
        _lastInput = 0.0f;
        _integral = 0.0f;
        _lastError = 0.0f;
        _lastTime = 0;
        _outputMin = -1000.0f;
        _outputMax = 1000.0f;
        _sampleTimeMs = 10;
    }

    void setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }
    void setTarget(float targetAngle) { _target = targetAngle; }
    void setOutputLimits(float minVal, float maxVal) { _outputMin = minVal; _outputMax = maxVal; }
    void setSampleTime(unsigned long sampleTimeMs) { _sampleTimeMs = sampleTimeMs; }

    void reset() {
        _integral = 0.0f;
        _lastError = 0.0f;
        _lastTime = 0;
    }

    float update(float currentAngle) {
        unsigned long now = millis();
        unsigned long dt = now - _lastTime;

        if (dt >= _sampleTimeMs) {
            float error = _target - currentAngle;
            float p = _kp * error;
            
            _integral += error * (dt / 1000.0f);
            
            if (_ki != 0.0f) {
                float iContribution = _ki * _integral;
                if (iContribution > _outputMax - p) _integral = (_outputMax - p) / _ki;
                if (iContribution < _outputMin - p) _integral = (_outputMin - p) / _ki;
            }
            
            float i = _ki * _integral;
            float d = 0.0f;
            
            if (dt > 0) {
                float errorRate = (error - _lastError) / (dt / 1000.0f);
                d = _kd * errorRate;
            }
            
            float output = constrainValue(p + i + d, _outputMin, _outputMax);
            
            _lastError = error;
            _lastInput = currentAngle;
            _lastTime = now;
            
            return output;
        }
        return 0.0f;
    }
};

// =============================================================================
// ENCODER DRIVER CLASS
// =============================================================================
class EncoderDriver {
private:
    float _zeroOffset[2] = {0.0f, 0.0f};
    float _lastAngle[2] = {0.0f, 0.0f};
    bool _connected[2] = {false, false};
    unsigned long _lastErrorTime[2] = {0, 0};

    void selectChannel(uint8_t ch) {
        if (ch > 7) ch = 0;
        Wire.beginTransmission(TCA9548A_ADDRESS);
        Wire.write(1 << ch);
        Wire.endTransmission();
    }

    uint16_t readAS5600Angle() {
        uint16_t angle = 0;
        Wire.beginTransmission(0x36);
        Wire.write(0x0C);
        Wire.endTransmission(false);
        Wire.requestFrom(0x36, 1);
        if (Wire.available()) angle = Wire.read();
        
        Wire.beginTransmission(0x36);
        Wire.write(0x0D);
        Wire.endTransmission(false);
        Wire.requestFrom(0x36, 1);
        if (Wire.available()) angle |= (Wire.read() << 8);
        
        return angle & 0x0FFF;
    }

public:
    EncoderDriver() {}

    void begin() {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
        Wire.setClock(I2C_FREQUENCY);
        delay(10);
        
        for (uint8_t motorId = 0; motorId < 2; motorId++) {
            if (isConnected(motorId)) {
                Serial.print("Encoder ");
                Serial.print(motorId == 0 ? "LEFT" : "RIGHT");
                Serial.println(" detected!");
            } else {
                Serial.print("ERROR: Encoder ");
                Serial.print(motorId == 0 ? "LEFT" : "RIGHT");
                Serial.println(" NOT detected!");
            }
        }
    }

    bool isConnected(uint8_t motorId) {
        selectChannel(motorId == 0 ? ENCODER_LEFT_CHANNEL : ENCODER_RIGHT_CHANNEL);
        Wire.beginTransmission(0x36);
        Wire.write(0x0C);
        uint8_t error = Wire.endTransmission();
        _connected[motorId] = (error == 0);
        return _connected[motorId];
    }

    float readAngleDegrees(uint8_t motorId) {
        if (motorId > 1) motorId = 0;
        uint8_t channel = (motorId == 0) ? ENCODER_LEFT_CHANNEL : ENCODER_RIGHT_CHANNEL;
        selectChannel(channel);
        
        uint16_t rawAngle = readAS5600Angle();
        float angleDegrees = rawAngle * 360.0f / 4096.0f;
        
        if (rawAngle > 0) {
            _lastAngle[motorId] = angleDegrees;
            return angleDegrees;
        } else {
            if (_connected[motorId]) {
                unsigned long now = millis();
                if (now - _lastErrorTime[motorId] > 10000) {
                    Serial.print("ERROR: Encoder ");
                    Serial.print(motorId == 0 ? "LEFT" : "RIGHT");
                    Serial.println(" read failed!");
                    _lastErrorTime[motorId] = now;
                }
            }
            return _lastAngle[motorId];
        }
    }

    void setZeroOffset(uint8_t motorId, float offsetDegrees) {
        if (motorId <= 1) _zeroOffset[motorId] = offsetDegrees;
    }

    float getCalibratedAngle(uint8_t motorId) {
        if (motorId > 1) motorId = 0;
        float rawAngle = readAngleDegrees(motorId);
        float calibrated = rawAngle - _zeroOffset[motorId];
        while (calibrated < 0) calibrated += 360.0f;
        while (calibrated >= 360.0f) calibrated -= 360.0f;
        return calibrated;
    }
};

// =============================================================================
// MOTOR DRIVER CLASS
// =============================================================================
class MotorDriver {
private:
    int _stepPin, _dirPin;
    long _currentStep = 0, _targetStep = 0;
    unsigned long _delayMicros = 976;
    unsigned long _lastStepTime = 0;

    void stepMotor() {
        bool direction = _targetStep > _currentStep;
        digitalWrite(_dirPin, direction ? HIGH : LOW);
        delayMicroseconds(1);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(1);
        digitalWrite(_stepPin, LOW);
        _lastStepTime = micros();
        if (direction) _currentStep++; else _currentStep--;
    }

public:
    MotorDriver() {
        _stepPin = _dirPin = 0;
        _lastStepTime = 0;
    }

    void begin(int stepPin, int dirPin, int rpm) {
        _stepPin = stepPin;
        _dirPin = dirPin;
        pinMode(_stepPin, OUTPUT);
        pinMode(_dirPin, OUTPUT);
        digitalWrite(_stepPin, LOW);
        digitalWrite(_dirPin, LOW);
        setSpeed(rpm);
    }

    void setSpeed(int rpm) {
        if (rpm <= 0) rpm = 1;
        _delayMicros = 60000000UL / (rpm * 4096);
    }

    void moveTo(long targetSteps) {
        _targetStep = _currentStep + targetSteps;
    }

    bool isMoving() { return _currentStep != _targetStep; }
    long getCurrentStep() { return _currentStep; }

    void update() {
        if (_currentStep == _targetStep) return;
        unsigned long now = micros();
        if (now - _lastStepTime >= _delayMicros) stepMotor();
    }
};

// =============================================================================
// CLOSED LOOP MOTOR CLASS
// =============================================================================
class ClosedLoopMotor {
private:
    int _motorId = -1;
    int _stepPin, _dirPin;
    EncoderDriver* _encoder;
    float _targetAngle = 0.0f;
    float _currentAngle = 0.0f;
    bool _enabled = false;
    unsigned long _lastStepTime = 0;
    long _stepDelayMicro = 0;
    long _currentStepCount = 0;
    unsigned long _lastUpdateTime = 0;
    PIDController _pid;

    void stepMotor() {
        unsigned long now = micros();
        float output = _pid.update(_currentAngle);
        bool direction = output >= 0;
        
        digitalWrite(_dirPin, direction ? HIGH : LOW);
        delayMicroseconds(1);
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(1);
        digitalWrite(_stepPin, LOW);
        _lastStepTime = now;
        
        if (direction) _currentStepCount++; else _currentStepCount--;
    }

public:
    ClosedLoopMotor() {}

    void begin(int motorId, int stepPin, int dirPin, int rpm, EncoderDriver* encoderPtr) {
        _motorId = motorId;
        _stepPin = stepPin;
        _dirPin = dirPin;
        _encoder = encoderPtr;

        pinMode(_stepPin, OUTPUT);
        pinMode(_dirPin, OUTPUT);
        digitalWrite(_stepPin, LOW);
        digitalWrite(_dirPin, LOW);

        _pid.setGains(PID_KP, PID_KI, PID_KD);
        float maxStepsPerSec = rpm * 4096.0f / 60.0f;
        _pid.setOutputLimits(-maxStepsPerSec, maxStepsPerSec);
        _pid.setSampleTime(10);
        _pid.reset();
        
        _enabled = true;
        _lastUpdateTime = millis();
        _lastStepTime = micros();
    }

    void setTargetAngle(float angleDegrees) {
        _targetAngle = angleDegrees;
        _pid.setTarget(angleDegrees);
        _pid.reset();
    }

    float getCurrentAngle() {
        if (_encoder != nullptr) {
            _currentAngle = _encoder->getCalibratedAngle(_motorId);
        }
        return _currentAngle;
    }

    void update() {
        if (!_enabled) return;
        
        unsigned long now = millis();
        unsigned long dt = now - _lastUpdateTime;
        if (dt < 10) return;

        _currentAngle = getCurrentAngle();
        float output = _pid.update(_currentAngle);

        if (abs(output) < 1.0f) {
            _stepDelayMicro = 0;
        } else {
            _stepDelayMicro = 1000000.0f / abs(output);
        }

        unsigned long nowMicro = micros();
        unsigned long elapsed = nowMicro - _lastStepTime;
        if (_stepDelayMicro > 0 && elapsed >= _stepDelayMicro) {
            stepMotor();
        }

        _lastUpdateTime = now;
    }

    void enable() { _enabled = true; }
    void disable() { _enabled = false; _stepDelayMicro = 0; }
    
    bool isAtTarget(float tolerance) {
        float error = abs(_targetAngle - getCurrentAngle());
        return error <= tolerance;
    }

    void setPIDGains(float kp, float ki, float kd) { _pid.setGains(kp, ki, kd); }
};

// =============================================================================
// INVERSE KINEMATICS CLASS
// =============================================================================
class InverseKinematics {
private:
    static float _motorSpacing;
    static float _l1;
    static float _l2;

public:
    static void setGeometry(float motorSpacing, float l1, float l2) {
        _motorSpacing = motorSpacing;
        _l1 = l1;
        _l2 = l2;
    }

    static void getGeometry(float& motorSpacing, float& l1, float& l2) {
        motorSpacing = _motorSpacing;
        l1 = _l1;
        l2 = _l2;
    }

    static bool computeAngles(float x, float y, float& angleLeftDeg, float& angleRightDeg, bool elbowUp) {
        float dxLeft = x + _motorSpacing / 2.0f;
        float dyLeft = y;
        float distanceLeft = sqrt(dxLeft * dxLeft + dyLeft * dyLeft);

        float dxRight = x - _motorSpacing / 2.0f;
        float dyRight = y;
        float distanceRight = sqrt(dxRight * dxRight + dyRight * dyRight);

        float minReach = abs(_l1 - _l2) + 1.0f;
        float maxReach = _l1 + _l2 - 1.0f;

        if (distanceLeft < minReach || distanceLeft > maxReach || 
            distanceRight < minReach || distanceRight > maxReach) {
            return false;
        }

        float cosLeftElbow = (distanceLeft * distanceLeft + _l1 * _l1 - _l2 * _l2) / (2.0f * distanceLeft * _l1);
        cosLeftElbow = constrain(cosLeftElbow, -1.0f, 1.0f);
        float elbowAngleLeft = acos(cosLeftElbow);
        float shoulderAngleLeft = atan2(dyLeft, dxLeft);
        angleLeftDeg = degrees(shoulderAngleLeft - elbowAngleLeft);

        float cosRightElbow = (distanceRight * distanceRight + _l1 * _l1 - _l2 * _l2) / (2.0f * distanceRight * _l1);
        cosRightElbow = constrain(cosRightElbow, -1.0f, 1.0f);
        float elbowAngleRight = acos(cosRightElbow);
        float shoulderAngleRight = atan2(dyRight, dxRight);
        angleRightDeg = degrees(shoulderAngleRight + elbowAngleRight);

        return true;
    }
};

float InverseKinematics::_motorSpacing = 50.0f;
float InverseKinematics::_l1 = 70.0f;
float InverseKinematics::_l2 = 100.0f;

// =============================================================================
// MOTION PLANNER CLASS
// =============================================================================
class MotionPlanner {
private:
    bool _moving = false;
    float _startAngle1, _startAngle2;
    float _targetAngle1, _targetAngle2;
    float _maxSpeed, _maxAccel;
    float _currentTime = 0.0f;
    float _duration = 0.0f;

public:
    MotionPlanner() {}

    void plan(float start1, float start2, float target1, float target2, float maxSpeed, float maxAccel) {
        _startAngle1 = start1;
        _startAngle2 = start2;
        _targetAngle1 = target1;
        _targetAngle2 = target2;
        _maxSpeed = maxSpeed;
        _maxAccel = maxAccel;

        float distance1 = abs(target1 - start1);
        float distance2 = abs(target2 - start2);
        float maxDistance = max(distance1, distance2);

        float tCruise = maxDistance / maxSpeed;
        float tAccel = maxSpeed / maxAccel;
        float tDecel = maxSpeed / maxAccel;

        if (tCruise > tAccel + tDecel) {
            _duration = tCruise + tAccel;
        } else {
            _duration = 2.0f * sqrt(maxDistance / maxAccel);
        }

        _currentTime = 0.0f;
        _moving = true;
    }

    bool nextStep(float& angle1, float& angle2) {
        if (!_moving) return false;

        _currentTime += 0.01f;

        if (_currentTime >= _duration) {
            angle1 = _targetAngle1;
            angle2 = _targetAngle2;
            _moving = false;
            return false;
        }

        float t = _currentTime / _duration;
        float smooth = t * t * (3.0f - 2.0f * t);

        angle1 = _startAngle1 + (_targetAngle1 - _startAngle1) * smooth;
        angle2 = _startAngle2 + (_targetAngle2 - _startAngle2) * smooth;

        return true;
    }

    bool isMoving() { return _moving; }
    void reset() { _moving = false; }
};

// =============================================================================
// GCODE PARSER CLASS
// =============================================================================
class GcodeParser {
private:
    String _lastError;
    float _currentX = 0.0f, _currentY = 0.0f;
    bool _absoluteMode = true;

public:
    GcodeParser() {}

    void begin() { _lastError = ""; }

    bool parseLine(String line, float& x_target, float& y_target, bool& penDownCmd, bool& hasMovement, bool& hasPen) {
        line.trim();
        if (line.length() == 0) return false;

        int commentIndex = line.indexOf(';');
        if (commentIndex != -1) line = line.substring(0, commentIndex);
        line.trim();

        if (line.length() == 0) return false;

        char cmd = line.charAt(0);
        
        x_target = 0.0f; y_target = 0.0f;
        penDownCmd = false; hasMovement = false; hasPen = false;

        if (cmd == 'G') {
            int code = line.substring(1).toInt();
            
            if (code == 0 || code == 1) {
                hasMovement = true;
                int xIndex = line.indexOf('X');
                int yIndex = line.indexOf('Y');
                
                if (xIndex != -1) {
                    int spaceIndex = line.indexOf(' ', xIndex + 1);
                    String xStr = (spaceIndex != -1) ? line.substring(xIndex + 1, spaceIndex) : line.substring(xIndex + 1);
                    x_target = xStr.toFloat();
                }
                
                if (yIndex != -1) {
                    int spaceIndex = line.indexOf(' ', yIndex + 1);
                    String yStr = (spaceIndex != -1) ? line.substring(yIndex + 1, spaceIndex) : line.substring(yIndex + 1);
                    y_target = yStr.toFloat();
                }
            }
        } else if (cmd == 'M') {
            int code = line.substring(1).toInt();
            
            if (code == 3 || code == 4) { penDownCmd = true; hasPen = true; }
            else if (code == 5) { penDownCmd = false; hasPen = true; }
        }

        return true;
    }

    void setCurrentPosition(float x, float y) { _currentX = x; _currentY = y; }
    String getLastError() { return _lastError; }
};

// =============================================================================
// SCARA WEBSERVER CLASS
// =============================================================================
class SCARAWebServer {
private:
    WebServer _server;

public:
    void (*onMove)(float x, float y, bool relative) = nullptr;
    void (*onPen)(bool up) = nullptr;
    void (*onHome)() = nullptr;
    void (*onCalibrateEncoder)(uint8_t motorId) = nullptr;
    void (*onCalibratePID)(float kp, float ki, float kd) = nullptr;
    void (*onStatus)(JsonObject& response) = nullptr;
    void (*onJog)(const char* axis, int steps) = nullptr;
    void (*onMotorRaw)(uint8_t motorId, int steps) = nullptr;
    void (*onWorkspace)(JsonObject& response) = nullptr;

    SCARAWebServer() : _server(80) {}

    void begin() {
        Serial.println("Web Server Initializing...");

        if (!LittleFS.begin()) {
            Serial.println("Warning: LittleFS mount failed!");
        } else {
            Serial.println("LittleFS initialized.");
        }

        _server.on("/", HTTP_GET, [this]() {
            if (LittleFS.exists("/index.html")) {
                _server.send(200, "text/html", LittleFS.open("/index.html", "r").readString());
            } else {
                _server.send(200, "text/html", "<html><body><h1>SCARA Robot</h1><p>Web interface not found.</p></body></html>");
            }
        });

        _server.on("/status", HTTP_GET, [this]() {
            StaticJsonDocument<512> doc;
            JsonObject response = doc.to<JsonObject>();
            response["success"] = true;
            if (onStatus) onStatus(response);
            else {
                response["x"] = 0; response["y"] = 0;
                response["pen_up"] = true; response["homed"] = false;
            }
            String jsonString;
            serializeJson(doc, jsonString);
            _server.send(200, "application/json", jsonString);
        });

        _server.on("/move", HTTP_POST, [this]() {
            if (_server.hasArg("plain")) {
                String body = _server.arg("plain");
                StaticJsonDocument<256> doc;
                if (!deserializeJson(doc, body)) {
                    float x = doc["x"] | 0.0f;
                    float y = doc["y"] | 0.0f;
                    if (onMove) onMove(x, y, false);
                    _server.send(200, "application/json", "{\"success\":true,\"message\":\"OK\"}");
                    return;
                }
            }
            _server.send(400, "application/json", "{\"success\":false}");
        });

        _server.on("/pen", HTTP_POST, [this]() {
            if (_server.hasArg("plain")) {
                String body = _server.arg("plain");
                StaticJsonDocument<128> doc;
                if (!deserializeJson(doc, body)) {
                    bool up = doc["up"] | false;
                    if (onPen) onPen(up);
                    _server.send(200, "application/json", "{\"success\":true}");
                    return;
                }
            }
            _server.send(400, "application/json", "{\"success\":false}");
        });

        _server.on("/home", HTTP_POST, [this]() {
            if (onHome) onHome();
            _server.send(200, "application/json", "{\"success\":true}");
        });

        _server.onNotFound([this]() {
            _server.send(404, "application/json", "{\"success\":false,\"message\":\"Not found\"}");
        });

        _server.begin();
        Serial.println("Web Server Started!");
    }

    void handleClient() { _server.handleClient(); }
};

// =============================================================================
// OTA MANAGER CLASS
// =============================================================================
class OTAManager {
private:
    WiFiManager _wifiManager;
    String _hostname;
    bool _otaStarted = false;

public:
    OTAManager() {}

    void begin(const char* hostname) {
        _hostname = hostname;
        Serial.println("WiFi + OTA Manager Initializing...");

        _wifiManager.setTimeout(120);
        WiFi.setHostname(hostname);

        Serial.println("Attempting WiFi connection...");
        bool connected = _wifiManager.autoConnect("SCARA-Robot-Setup");

        if (connected) {
            Serial.println("Connected to WiFi!");
            Serial.print("IP: ");
            Serial.println(WiFi.localIP());
        }

        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.setHostname(hostname);
            ArduinoOTA.setPassword("scara123");

            ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
            ArduinoOTA.onProgress([](unsigned int p, unsigned int t) { 
                Serial.printf("Progress: %u%%\r", (p / (t / 100))); 
            });
            ArduinoOTA.onEnd([]() { Serial.println("\nOTA Complete!"); });
            ArduinoOTA.onError([](ota_error_t e) { 
                Serial.printf("Error: %u\n", e); 
            });

            ArduinoOTA.begin();
            _otaStarted = true;
            Serial.println("OTA Ready!");
        }
    }

    void handle() {
        if (_otaStarted) ArduinoOTA.handle();
    }
};

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================
EncoderDriver encoder;
ClosedLoopMotor leftMotor;
ClosedLoopMotor rightMotor;
MotorDriver penMotor;
OTAManager ota;
SCARAWebServer webServer;
InverseKinematics ik;
MotionPlanner planner;
GcodeParser parser;

// =============================================================================
// GLOBAL STATE
// =============================================================================
float currentX = 0.0f;
float currentY = 50.0f;
float currentLeftAngle = 0.0f;
float currentRightAngle = 0.0f;
bool penDown = false;
bool homed = true;
bool moving = false;

// =============================================================================
// CALLBACK FUNCTIONS
// =============================================================================
void handleMoveCallback(float x, float y, bool relative) {
    if (relative) {
        currentX += x;
        currentY += y;
    } else {
        currentX = x;
        currentY = y;
    }
}

void handlePenCallback(bool up) { penDown = !up; }

void handleHomeCallback() {
    currentX = 0.0f;
    currentY = 50.0f;
    currentLeftAngle = 0.0f;
    currentRightAngle = 0.0f;
    homed = true;
    encoder.setZeroOffset(0, currentLeftAngle);
    encoder.setZeroOffset(1, currentRightAngle);
    Serial.println("Homing complete!");
}

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

void handleStatusCallback(JsonObject& response) {
    response["x"] = currentX;
    response["y"] = currentY;
    response["angle_left"] = currentLeftAngle;
    response["angle_right"] = currentRightAngle;
    response["pen_up"] = !penDown;
    response["homed"] = homed;
}

void handleJogCallback(const char* axis, int steps) {
    Serial.print("Jog: ");
    Serial.print(axis);
    Serial.print(" ");
    Serial.println(steps);
}

void handleMotorRawCallback(uint8_t motorId, int steps) {
    Serial.print("Motor ");
    Serial.print(motorId);
    Serial.print(" raw steps: ");
    Serial.println(steps);
}

void handleWorkspaceCallback(JsonObject& response) {
    response["min_x"] = -120.0f;
    response["max_x"] = 120.0f;
    response["min_y"] = 0.0f;
    response["max_y"] = 170.0f;
    response["motor_spacing"] = MOTOR_SPACING;
    response["arm1_length"] = L1;
    response["arm2_length"] = L2;
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================
void moveToXY(float x, float y) {
    float targetLeftAngle, targetRightAngle;

    if (!InverseKinematics::computeAngles(x, y, targetLeftAngle, targetRightAngle, true)) {
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
    Serial.println(targetRightAngle);

    float startLeftAngle = leftMotor.getCurrentAngle();
    float startRightAngle = rightMotor.getCurrentAngle();

    planner.plan(startLeftAngle, startRightAngle, targetLeftAngle, targetRightAngle,
                 MAX_SPEED_DEG_PER_SEC, MAX_ACCELERATION_DEG_PER_SEC2);

    moving = true;
    currentX = x;
    currentY = y;
}

void updateMotors() {
    leftMotor.update();
    rightMotor.update();
    penMotor.update();
}

void processGcodeLine(String line) {
    float x_target, y_target;
    bool penDownCmd, hasMovement, hasPen;

    if (parser.parseLine(line, x_target, y_target, penDownCmd, hasMovement, hasPen)) {
        if (hasPen) penDown = penDownCmd;
        if (hasMovement) {
            moveToXY(x_target, y_target);
            parser.setCurrentPosition(x_target, y_target);
        }
    } else {
        Serial.print("Parse error: ");
        Serial.println(parser.getLastError());
    }
}

void setPen(bool down) {
    if (down != penDown) {
        penDown = down;
        if (penDown) {
            Serial.println("Pen DOWN");
            penMotor.moveTo(PEN_STEPS_UP);
        } else {
            Serial.println("Pen UP");
            penMotor.moveTo(0);
        }
    }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) delay(10);
    
    Serial.println();
    Serial.println("===========================================");
    Serial.println("  SCARA Robot Starting...");
    Serial.println("===========================================");

    Serial.println("Initializing encoders...");
    encoder.begin();

    Serial.println("Initializing motors...");
    leftMotor.begin(0, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 15, &encoder);
    rightMotor.begin(1, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 15, &encoder);
    leftMotor.setPIDGains(PID_KP, PID_KI, PID_KD);
    rightMotor.setPIDGains(PID_KP, PID_KI, PID_KD);

    penMotor.begin(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, 15);
    penMotor.moveTo(0);

    currentLeftAngle = encoder.getCalibratedAngle(0);
    currentRightAngle = encoder.getCalibratedAngle(1);
    leftMotor.setTargetAngle(currentLeftAngle);
    rightMotor.setTargetAngle(currentRightAngle);

    Serial.println("Initializing kinematics...");
    InverseKinematics::setGeometry(MOTOR_SPACING, L1, L2);

    parser.begin();

    Serial.println("Starting WiFi and OTA...");
    ota.begin("SCARA-Robot");

    Serial.println("Starting web server...");
    webServer.begin();
    webServer.onMove = handleMoveCallback;
    webServer.onPen = handlePenCallback;
    webServer.onHome = handleHomeCallback;
    webServer.onCalibrateEncoder = handleCalibrateEncoderCallback;
    webServer.onCalibratePID = handleCalibratePIDCallback;
    webServer.onStatus = handleStatusCallback;
    webServer.onJog = handleJogCallback;
    webServer.onMotorRaw = handleMotorRawCallback;
    webServer.onWorkspace = handleWorkspaceCallback;

    planner.reset();
    homed = true;

    Serial.println();
    Serial.println("===========================================");
    Serial.println("  SCARA Robot Ready!");
    Serial.println("===========================================");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  G0 X30 Y40   - Move to position");
    Serial.println("  M3           - Pen down");
    Serial.println("  M5           - Pen up");
    Serial.print("Web interface: http://");
    Serial.println(WiFi.localIP());
    Serial.println("===========================================");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    ota.handle();
    webServer.handleClient();
    updateMotors();

    currentLeftAngle = leftMotor.getCurrentAngle();
    currentRightAngle = rightMotor.getCurrentAngle();

    if (moving && planner.isMoving()) {
        float targetAngle1, targetAngle2;
        if (planner.nextStep(targetAngle1, targetAngle2)) {
            leftMotor.setTargetAngle(targetAngle1);
            rightMotor.setTargetAngle(targetAngle2);
        } else {
            moving = false;
            Serial.println("Motion complete!");
        }
    }

    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            Serial.print("> ");
            Serial.println(line);
            processGcodeLine(line);
        }
    }

    delay(1);
}