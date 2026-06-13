# EspX: ESP32 Dual-Core Drone Controller

EspX is a high-performance, dual-core drone flight controller built for the ESP32 platform. Leveraging the power of FreeRTOS, it separates critical flight control logic from communication and system monitoring tasks to ensure maximum stability and responsiveness.

## 🚀 Features

- **Dual-Core Architecture**: 
  - **Core 1 (APP CPU)**: Dedicated to high-frequency PID control, IMU data processing, and motor output (250Hz).
  - **Core 0 (PRO CPU)**: Handles Wi-Fi communication, UDP RC command processing, LED status management, and battery monitoring (50Hz).
- **Advanced Sensor Fusion**: Utilizes Kalman filtering for accurate orientation estimation from IMU data.
- **Wi-Fi Control**: Receives RC commands via UDP, allowing for wireless control through a mobile app or custom controller.
- **Safety First**:
  - Integrated failsafe mechanism: Automatically disarms if the RC signal is lost for more than 100ms.
  - Real-time battery monitoring.
  - Visual status feedback via RGB LEDs.
- **Precision Timing**: Task execution is synchronized using FreeRTOS primitives (mutexes, queues) for jitter-free performance.

## 🛠 Hardware Setup

### Components
- **Microcontroller**: ESP32 DevKit V1
- **IMU**: MPU6050 (or similar I2C compatible sensor)
- **Motors**: 4x Brushless/Brushed Motors (controlled via ESCs or appropriate drivers)
- **LEDs**: RGB Status LED
- **Power**: LiPo Battery with voltage divider for monitoring

### Pin Mapping
| Component | ESP32 Pin | Description |
|-----------|-----------|-------------|
| **Motor 1** | GPIO 4 | Front Right |
| **Motor 2** | GPIO 33 | Back Right |
| **Motor 3** | GPIO 32 | Back Left |
| **Motor 4** | GPIO 25 | Front Left |
| **IMU SDA** | GPIO 21 | I2C Data |
| **IMU SCL** | GPIO 22 | I2C Clock |
| **Battery Sense** | GPIO 35 | Analog Input |
| **Red LED** | GPIO 5 | Status Indication |
| **Green LED** | GPIO 18 | Status Indication |
| **Blue LED** | GPIO 23 | Status Indication |

## 💻 Software Configuration

The project is developed using **PlatformIO** and the **Arduino Framework**.

### Dependencies
- ESP32 Board Support
- Wire (I2C)
- WiFi & WiFiUDP
- FreeRTOS (Built-in to ESP32 core)

### Installation
1. Install [PlatformIO](https://platformio.org/).
2. Clone this repository.
3. Open the project in VS Code with PlatformIO extension.
4. Build and upload to your ESP32.

## 🕹 Usage

### Status LED Indicators
- **Solid Green**: Connected & Disarmed (Ready to arm).
- **Solid Red**: **ARMED** (Motors active).
- **Blinking/Solid Yellow**: No RC Signal (Failsafe mode).

### Controls
The controller expects RC data via UDP. The default mapping follows standard AETR:
- **Aux 1**: Toggle to 2000 to **ARM** the drone.
- **Aux 2**: Toggle to 2000 for **Angle Mode** (Auto-leveling); otherwise, Rate Mode.

## ⚠️ Safety Warning
- **Always** remove propellers when testing on the bench.
- Ensure your battery voltage divider is correctly calibrated to avoid damaging the ESP32.
- Test in a clear, open area away from people and property.

---
*Developed with ❤️ for the ESP32 community.*
