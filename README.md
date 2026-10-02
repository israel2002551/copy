# AeroSync Pro — Smart IoT Climate & Fan Controller

A next-generation IoT Smart Fan and environmental telemetry monitoring station built with **ESP32**, **MQTT**, and a **Cyber/Glassmorphic Web Dashboard**.

---

## 🚀 Key Improvements & Logic Fixes

### 1. Embedded Firmware Logic (`smart.ino`)
- **Non-Blocking Wi-Fi Auto-Recovery**: Fixed critical bug where dropped Wi-Fi connections never reconnected. Added automatic periodic reconnection timer without stalling motor or sensor tasks.
- **DHT22 Transient Glitch Resilience**: Added consecutive failure filtering. Single transient bit-flips or checksum errors on DHT22 no longer cause the motor to jerk to a stop or LCD to flash error screens; safety halt triggers only after 3 persistent failures.
- **NVS Flash Wear Protection**: Implemented debounced parameter commits (3-second stabilization timer). Moving the speed slider no longer causes repeated flash writes, protecting ESP32 non-volatile memory from premature degradation.
- **Zero-Latency Remote Commands**: Incoming MQTT commands for mode or speed now immediately adjust PWM motor outputs and push a telemetry update, eliminating the previous 2-second command lag.
- **DC Motor Kickstart Pulse**: Added an automatic 40ms PWM pulse when starting from rest into low speeds (< 110 PWM) to overcome static friction and prevent motor humming/stalling.
- **Buzzer Mute & Remote Test**: Added MQTT remote command handling for `"buzzer": "test"` and `"buzzer": "mute"`.
- **Configurable Thermostat Setpoint**: The Auto cooling threshold temperature is now adjustable remotely via MQTT and stored in NVS.
- **Dual ESP32 Core & ArduinoJson Compatibility**: Seamless compilation on both **ESP32 Arduino Core 2.x and 3.x** (conditional `ledcAttach` vs `ledcSetup`) and **ArduinoJson v6 and v7** (`JsonDocument` / `StaticJsonDocument`).
- **Telemetry Enrichment**: Added relative humidity, Wi-Fi RSSI, uptime, and sensor diagnostic state.

### 2. Web Application (`index.html`)
- **Stunning Glassmorphism Design System**: Built with modern typography (*Outfit* & *Plus Jakarta Sans*), responsive grid, glowing status indicators, and clean CSS variables.
- **Dynamic 5-Blade Aerodynamic Fan Visualizer**: SVG fan rotor spinning smoothly with rotational velocity directly tied to the actual PWM throttle (stops smoothly when PWM is 0, glows turbo amber at full speed).
- **True Hardware Availability (LWT)**: Subscribes to `smartfan/{DEVICE_ID}/availability` to distinguish between broker connection and physical ESP32 power status.
- **Interactive Rolling Telemetry Charts**: Real-time Chart.js graph plotting Temperature, Humidity, Air Quality, and Fan Speed with individual toggleable datasets.
- **Full Metric Calculations**: Computes Heat Index and categorized air quality / comfort levels.
- **Settings Modal & Persistence**: Configurable Device ID, MQTT Broker URL, Temperature Units (°C / °F), and persistent state in `localStorage`.
- **Live Demo / Simulation Mode**: Built-in toggle to test the full UI, slider interactions, and animated fan even when hardware is offline.
- **Synthesized Audio Clicks & Alerts**: Web Audio API integration for tactile audio feedback without external audio files.

---

## 🔌 Hardware Pinout (ESP32)

| Component | ESP32 GPIO | Description |
| :--- | :--- | :--- |
| **DHT22 Sensor** | `GPIO 32` | Temperature & Relative Humidity (1-Wire) |
| **Air Quality Sensor (MQ-135)** | `GPIO 34` | ADC1 Channel 6 (Analog Input) |
| **Active Buzzer** | `GPIO 33` | Digital Output (High = Sound) |
| **Motor Driver ENA (PWM)** | `GPIO 25` | 5 kHz 8-bit PWM (0 - 255) |
| **Motor Driver IN1** | `GPIO 26` | Direction Pin 1 |
| **Motor Driver IN2** | `GPIO 27` | Direction Pin 2 |
| **I2C LCD SDA** | `GPIO 21` | I2C Data (16x2 LCD, Addr 0x27) |
| **I2C LCD SCL** | `GPIO 22` | I2C Clock |

---

## 📡 MQTT Topic Protocol

Default Device ID: `DEVICE123` *(configurable in smart.ino & website settings)*

| Topic | Direction | Payload Example / Description |
| :--- | :--- | :--- |
| `smartfan/DEVICE123/availability` | ESP32 &rarr; Broker | `"online"` (retained) / `"offline"` (LWT) |
| `smartfan/DEVICE123/status` | ESP32 &rarr; Broker | `{"temp":28.4,"humidity":52.1,"air":15,"mode":"AUTO","speed":180,"alarm":false,"target_temp":27.0,"sensor_ok":true,"wifi_rssi":-58,"uptime":340}` |
| `smartfan/DEVICE123/set` | Browser &rarr; ESP32 | Set Mode: `{"mode":"MANUAL"}` or `{"mode":"AUTO"}`<br>Set Speed: `{"mode":"MANUAL","speed":200}`<br>Set Auto Threshold: `{"auto_thresh":26.5}`<br>Buzzer Control: `{"buzzer":"test"}` or `{"buzzer":"mute"}` |

---

## 🖥️ Running & Testing the Web Dashboard

1. Double-click `index.html` in any modern web browser, or serve it locally:
   ```bash
   python -m http.server 8080
   ```
2. Navigate to `http://localhost:8080/index.html`.
3. Click **Settings** (top right) to change the Device ID or enable **Live Simulation Mode** to preview real-time fan animation and chart streaming without hardware.