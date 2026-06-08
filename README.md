# 🤖 5-Bar Parallel SCARA Robot

![Status](https://img.shields.io/badge/status-ready-brightgreen)
![ESP32-C3](https://img.shields.io/badge/ESP32--C3-Supported-blue)
![License](https://img.shields.io/badge/license-MIT-green)

Complete **5-bar parallel SCARA robot** firmware with modular architecture for **ESP32-C3 SuperMini**. Features closed-loop motor control, inverse kinematics, web interface, and OTA updates.

---

## 📋 Quick Reference

| Item | Value |
|------|-------|
| Controller | ESP32-C3 SuperMini |
| Motors | 3× 28BYJ-48 + ULN2003A |
| Encoders | 2× AS5600 via TCA9548A |
| Max Reach | 170 mm |
| Serial | 115200 baud |

---

## ⚡ Quick Start

1. **Install Arduino IDE** + ESP32 board support
2. **Install Libraries**: WiFiManager, ESPAsyncWebServer, AsyncTCP, ArduinoJson
3. **Open** `SCARA_Robot.ino`
4. **Upload** via USB
5. **Connect**: WiFi AP "SCARA-Robot-Setup" or saved credentials
6. **Open**: `http://[ESP32-IP]/`

---

## 🔧 Hardware Pinout

```
Left Motor:   GPIO2 (step), GPIO3 (dir)
Right Motor:  GPIO4 (step), GPIO5 (dir)
Pen Motor:    GPIO6 (step), GPIO7 (dir)
I2C:          GPIO8 (SDA), GPIO9 (SCL)
Encoders:     TCA9548A ch0 (left), ch1 (right)
```

---

## 📁 Project Structure (25 files)

```
SCARA_Robot/
├── SCARA_Robot.ino              ← Main sketch
├── config/robot_config.h        ← Configuration
├── drivers/
│   ├── motor_driver.h/.cpp       ← Stepper control
│   └── encoder_driver.h/.cpp    ← AS5600 reading
├── control/
│   ├── pid_controller.h/.cpp    ← PID loop
│   └── closed_loop_motor.h/.cpp ← Motor+PID+Encoder
├── system/ota_manager.h/.cpp    ← WiFi + OTA
├── web/
│   ├── web_server.h/.cpp         ← REST API
│   └── data/ (index.html, style.css, script.js)
├── kinematics/inverse_kinematics.h/.cpp ← X,Y → angles
└── motion/
    ├── motion_planner.h/.cpp     ← Trajectories
    └── gcode_parser.h/.cpp       ← G-code parsing
```

---

## 🔌 Arduino IDE Setup

**Board Settings:**
| Setting | Value |
|---------|-------|
| Board | ESP32C3 Dev Module |
| Upload Speed | 921600 |
| CPU Frequency | 160 MHz |
| Flash Size | 4MB |
| USB CDC On Boot | Enabled |

**Libraries (Install via Library Manager):**
- WiFiManager by tzapu
- ESPAsyncWebServer (GitHub manual install)
- AsyncTCP (GitHub manual install)
- ArduinoJson by Benoit Blanchon

---

## 📤 Flashing Firmware

**USB Upload:**
1. Connect ESP32-C3 via USB
2. Tools → Port → COM port
3. Click Upload
4. If fails: Hold BOOT + press RST → release BOOT → upload

**OTA Upload (after first flash):**
1. Tools → Port → SCARA-Robot at 192.168.x.x (network)
2. Click Upload

**SPIFFS Upload (web files):**
1. Copy `web/data/` to sketch's `data/` folder
2. Tools → ESP32 Sketch Data Upload

---

## 🌐 Web Interface

Access at `http://[ESP32-IP]/`

| Panel | Features |
|-------|----------|
| Status | Real-time X, Y, angles |
| Pen Control | Up/Down buttons |
| Workspace | SVG visualizer |
| Jog | X±, Y± buttons |
| Calibration | Encoder zero, PID sliders |
| G-Code | File upload & send |

---

## ⌨️ Serial Commands

| Command | Description |
|---------|-------------|
| `G0 X30 Y40` | Move to (30, 40) |
| `G1 X50 Y60 F100` | Move with speed |
| `G90` | Absolute mode |
| `G91` | Incremental mode |
| `M3` | Pen down |
| `M5` | Pen up |

---

## 🌏 REST API

```bash
# Move
curl -X POST http://192.168.1.100/move \
  -H "Content-Type: application/json" \
  -d '{"x": 30, "y": 40}'

# Pen
curl -X POST http://192.168.1.100/pen \
  -H "Content-Type: application/json" \
  -d '{"state": "up"}'

# PID
curl -X POST http://192.168.1.100/calibrate/pid \
  -H "Content-Type: application/json" \
  -d '{"kp": 2.0, "ki": 0.5, "kd": 0.1}'

# Status
curl http://192.168.1.100/status
```

| Endpoint | Method | Body |
|----------|--------|------|
| `/move` | POST | `{"x": 30, "y": 40}` |
| `/moverel` | POST | `{"dx": 5, "dy": -2}` |
| `/pen` | POST | `{"state": "up"}` |
| `/home` | POST | - |
| `/calibrate/encoder` | POST | `{"motor": 0}` |
| `/calibrate/pid` | POST | `{"kp": 2, "ki": 0.5, "kd": 0.1}` |
| `/status` | GET | - |
| `/jog` | POST | `{"axis": "x", "steps": 10}` |
| `/motor_raw` | POST | `{"motor": 0, "steps": 100}` |
| `/workspace` | GET | - |

---

## ⚙️ Configuration

Edit `config/robot_config.h`:

```cpp
// Geometry
#define MOTOR_SPACING 50.0f
#define L1 70.0f
#define L2 100.0f

// PID Defaults
#define PID_KP 1.0f
#define PID_KI 0.0f
#define PID_KD 0.1f

// Limits
#define MAX_SPEED_DEG_PER_SEC 180.0f
#define MAX_ACCELERATION_DEG_PER_SEC2 360.0f
```

---

## 📐 Calibration

**Encoder Zero:**
1. Position arm at reference point
2. Web interface → "Set Left = 0°" or "Set Right = 0°"

**PID Tuning:**
1. Adjust Kp, Ki, Kd sliders
2. Click "Apply PID"
3. Tune until smooth movement

---

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| Upload fails | Hold BOOT → press RST → release BOOT |
| WiFi won't connect | Connect to AP "SCARA-Robot-Setup" |
| Motors not moving | Check power, pins, ULN2003A |
| Encoders not reading | Check I2C wiring, TCA9548A address |
| Web interface missing | Run ESP32 Sketch Data Upload |

---

## 📜 License

MIT License

---

**Project: ✅ Complete**

GitHub: https://github.com/ducsj/massiveSCARA