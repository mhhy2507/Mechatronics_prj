# 🚗 ESP32 Line Following Delivery Robot

A line-following AGV (Automated Guided Vehicle) built on **ESP32** with dual DC motors, rotary encoders, line sensors, and loadcell for automatic delivery tasks.  
The robot can detect intersections, brake, turn left/right based on payload weight, and send telemetry via UART → WiFi → MQTT.

---

## ✨ Features

- **Line following** with 7x TCRT5000 analog sensors + Lyapunov controller
- **Motor control**: DC motors with encoders, closed-loop RPM PID control
- **Load detection**: HX711 + loadcell to determine delivery direction
- **Profiles**:
  - Multiple speed/control profiles (e.g., 0.3 m/s, 0.7 m/s)
  - Switchable via long-press button
- **Intersection logic**:
  - Brake at first intersection
  - Decide left/right turn at second intersection based on payload weight
  - Stop at delivery zone
- **Telemetry**:
  - JSON over UART
  - ESP32 #2 bridges UART → WiFi (MQTT) and beeps profile selection with a buzzer
- **Failsafes**:
  - Hard brake on lost line
  - Smooth ramp-up/down to avoid jerks
  - Profile auto-exit timeout

---

## 🛠 Hardware

- **Controller**: ESP32-WROOM
- **Motors**: 2x DC motors with encoders (L293D / MD10C driver)
- **Sensors**:
  - 7x TCRT5000 line sensors
  - HX711 + loadcell (weight-based delivery logic)
- **Power**: 4S 18650 Li-ion pack + buck converters
- **Other**:
  - Start/Profile select button
  - Buzzer (profile feedback on ESP32 #2)

---

## ⚙️ Software

### Main ESP32 (robot control)
- Reads line sensors
- Calculates error (TB, x)
- Lyapunov controller → setpoints
- PID closed-loop RPM control
- State machine for delivery logic
- Sends telemetry JSON via UART

### ESP32 #2 (telemetry + buzzer)
- Receives UART JSON
- Publishes telemetry to MQTT (HiveMQ Cloud)
- Beeps when profile changes (1 beep = Profile 1, 2 beeps = Profile 2, etc.)

---

## 📡 Telemetry JSON Example

```json
{
  "t": 123456,
  "type": "state",
  "mode": "RUN",
  "started": true,
  "sp": [100.0, 100.0],
  "rpm": [98.5, 97.8],
  "w": 1200,
  "s": [1234, 2230, 3100, 4000, 3000, 2100, 1500],
  "TB": -0.322,
  "x": -5.4,
  "ic": 1,
  "profile": 2,
  "e3": 0.00123
}
