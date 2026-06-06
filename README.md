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

