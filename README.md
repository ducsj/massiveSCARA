# massiveSCARA

# 5‑Bar Parallel SCARA Robot

![Robot Status](https://img.shields.io/badge/status-in%20development-yellow)
![ESP32](https://img.shields.io/badge/ESP32--C3-Supported-blue)
![License](https://img.shields.io/badge/license-MIT-green)

A **5‑bar parallel SCARA robot** built with low‑cost components, designed for drawing, writing, and light pick‑and‑place tasks.  
All software is modular, beginner‑friendly, and runs on an **ESP32‑C3 SuperMini** with **OTA updates** and a **web‑based control interface**.

---

## 📦 Hardware Inventory

| Component | Quantity | Purpose |
|-----------|----------|---------|
| ESP32‑C3 SuperMini | 1 | Main controller |
| ESP32‑C3 Expansion Board | 1 | Easier connections (optional) |
| 28BYJ‑48 stepper motor | 3 | Arm actuation + pen lift |
| ULN2003A driver | 3 | Motor drivers |
| AS5600 magnetic encoder | 2 | Closed‑loop feedback for arms |
| TCA9548A I2C multiplexer | 1 | Read multiple encoders on same I2C bus |
| 18650 battery (3000mAh) | 3 | Power source |
| 3S 4A charger board | 1 | Charge the battery pack |
| 3S 20A BMS | 1 | Battery protection |
| DC‑DC step‑up / step‑down | 2 | Regulate voltage for ESP32 & motors |
| 5V LED traffic light module | 1 | Status indication (optional) |
| 100µF capacitors | 5 | Noise filtering |
| Breadboard, jumpers, soldering iron | – | Assembly & debugging |

> **Note:** No TCRT5000 sensor is used – homing is done manually or via mechanical stops (future improvement).

---

## 📁 Software Architecture (Modular)

```

SCARA_Robot/
├── config/
│   └── robot_config.h          # All constants & pin definitions
├── drivers/
│   ├── motor_driver.h/.cpp     # Stepper motor control (open loop)
│   ├── encoder_driver.h/.cpp   # AS5600 via TCA9548A
├── control/
│   ├── pid_controller.h/.cpp   # Position PID
│   ├── closed_loop_motor.h/.cpp# Combines motor + encoder + PID
├── kinematics/
│   ├── inverse_kinematics.h/.cpp # 5‑bar geometry solver
│   ├── workspace.h/.cpp          # Reachability checks
├── motion/
│   ├── motion_planner.h/.cpp     # Trapezoidal motion profiles
│   ├── line_drawer.h/.cpp        # High‑level drawing commands
├── system/
│   ├── ota_manager.h/.cpp        # Over‑the‑air updates
│   ├── calibration.h/.cpp        # Encoder zero & PID tuning
│   ├── homing.h/.cpp             # Homing routine (manual/button)
├── web/
│   ├── web_server.h/.cpp         # AsyncWebServer + REST API
│   ├── data/
│   │   ├── index.html            # Web UI
│   │   ├── style.css
│   │   └── script.js
└── SCARA_Robot.ino               # Main entry point


Each module is **independent** – you can test them one by one.

---

## 🛠️ Setup Instructions

### 1. Install Arduino IDE & ESP32‑C3 Support

- Download [Arduino IDE](https://www.arduino.cc/en/software)
- Add ESP32 board URL:  
  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
- Install **esp32** via Boards Manager (version 2.0.14+)
- Select board: `ESP32C3 Dev Module`
- Set **USB CDC On Boot** → `Enabled` (so Serial.print works)

### 2. Install Required Libraries

| Library | Install via |
|---------|-------------|
| AS5600 | Library Manager (Rob Tillaart) |
| Adafruit TCA9548A | Library Manager |
| PID (Brett Beauregard) | Library Manager |
| ESPAsyncWebServer | [GitHub](https://github.com/me-no-dev/ESPAsyncWebServer) (manual) |
| AsyncTCP | [GitHub](https://github.com/me-no-dev/AsyncTCP) (manual) |
| ArduinoOTA | Built‑in (ESP32 core) |

### 3. First Flash (USB)

- Connect ESP32‑C3 via USB data cable
- Select the correct COM port
- Upload `SCARA_Robot.ino` (the main sketch)
- If upload fails, **hold BOOT** → press RST → release BOOT → upload again

### 4. Subsequent Updates (OTA)

After the first flash includes the OTA module, you can update over WiFi:

- Go to **Tools → Port → Network ports** → select your ESP32's IP address
- Click **Upload** – no USB cable needed!

> The ESP32 will appear as `SCARA-Robot` in the Arduino IDE network ports.

### 5. Web Interface

Once the robot is on your WiFi, open a browser and enter:

You will see a dashboard with:
- Real‑time position of both arms
- Pen up/down buttons
- Manual jogging (X/Y or individual motors)
- G‑code upload & execution
- Calibration tools (set encoder offsets, PID tuning)

---

## 🚀 Roadmap (12 Steps)

| # | Module | Status |
|---|--------|--------|
| 1 | `robot_config.h` | ✅ Completed |
| 2 | Motor Driver (open loop) | 🟡 In progress |
| 3 | Encoder Driver | ⏳ Pending |
| 4 | PID Controller | ⏳ Pending |
| 5 | Closed‑Loop Motor | ⏳ Pending |
| 6 | OTA Updates | ⏳ Pending |
| 7 | Web Server + REST API | ⏳ Pending |
| 8 | Web UI (HTML/CSS/JS) | ⏳ Pending |
| 9 | Inverse Kinematics | ⏳ Pending |
| 10 | Motion Planner | ⏳ Pending |
| 11 | G‑code Parser | ⏳ Pending |
| 12 | Full Integration & Testing | ⏳ Pending |

> Follow the instructions in this repo’s 

---

## 📝 Using OpenHands with This Repository




## ⚙️ Robot Geometry

| Parameter | Value |
|-----------|-------|
| Motor spacing | 50 mm |
| Upper arm length (L1) | 70 mm |
| Forearm length (L2) | 100 mm |
| Pen lift range | ~ 10 mm (mechanical, via motor steps) |

The inverse kinematics solve for **elbow‑up** configuration only (simpler for drawing).

}

