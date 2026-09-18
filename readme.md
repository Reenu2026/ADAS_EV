# EV ADAS System: Real-Time Electric Vehicle Dashboard 🚗⚡

![Status: Active](https://img.shields.io/badge/Status-Active-brightgreen)
![Platform: STM32](https://img.shields.io/badge/Platform-STM32F103C8T6-blue)
![Environment: PICSimLab](https://img.shields.io/badge/Environment-PICSimLab-orange)

An embedded C and Python project simulating a Real-Time Electric Vehicle (EV) monitor and Advanced Driver Assistance System (ADAS). Built for the **STM32F103C8T6 (Blue Pill)**, this system processes real-time pedal inputs, monitors EV telemetry (Speed, SOC, Torque, Range), and uses ultrasonic sensors (HC-SR04) for collision avoidance and blind-spot detection. 

Data is streamed over UART at 10 Hz to a custom **Python GUI Dashboard**.

---

## ✨ Features

* **Real-Time EV Sensing:** Computes vehicle speed via a physics-based inertia model, estimates State of Charge (SOC), and calculates torque/power dynamically based on Drive Modes (ECO/NORMAL/SPORT).
* **ADAS Safety Engine:** 
  * **Forward Collision Warning:** Calculates Time To Collision (TTC) using front sensor data and vehicle speed. Triggers Warning/Critical alarms.
  * **Blind-Spot Detection:** Monitors left/right ultrasonic sensors to alert drivers of adjacent obstacles.
* **Deterministic State Machine:** Seamlessly transitions through states: `PARKED` → `READY` → `DRIVING` → `REGEN` → `FAULT`.
* **Live Telemetry & Dashboard:** Streams binary packets via UART DMA to a Python Matplotlib/PySerial dashboard showing live gauges, a bird-eye ADAS view, and alarm banners.
* **Interactive UART Shell:** Control the vehicle, inject faults, or override sensors directly via terminal commands.

---

## 🛠️ Hardware & Pin Mapping

This project is designed to run in **PICSimLab** (or on real hardware) using the STM32F103C8T6.

| Component | Pin | I/O | Description |
| :--- | :--- | :--- | :--- |
| **ADC: Accelerator** | `PA0` | Input | Potentiometer simulating pedal (0-100%) |
| **ADC: Brake** | `PA1` | Input | Potentiometer simulating brake (0-100%) |
| **ADC: Battery SOC** | `PA2` | Input | Potentiometer simulating SOC |
| **ADC: Motor Temp** | `PA3` | Input | Potentiometer simulating Motor Temp (NTC) |
| **UART TX (Telemetry)**| `PA9` | Output| USART1 TX (DMA stream to Dashboard) |
| **UART RX (Shell)** | `PA10`| Input | USART1 RX (Interrupt-based command shell) |
| **HC-SR04 Front** | `PB0` / `PB1` | Out/In| TRIG / ECHO |
| **HC-SR04 Left** | `PB2` / `PB3` | Out/In| TRIG / ECHO |
| **HC-SR04 Right** | `PB4` / `PB5` | Out/In| TRIG / ECHO |
| **LED: Collision** | `PB8` | Output| Red LED - Forward collision warning/critical |
| **LED: Blind Spot L**| `PB9` | Output| Yellow LED - Left blind spot active |
| **LED: Blind Spot R**| `PB10`| Output| Yellow LED - Right blind spot active |
| **LED: Fault** | `PB11`| Output| Red LED - System fault indicator |

---

## 📂 Software Architecture & Modules

The C firmware is modularized for strict real-time execution. The main control loop runs at **5 ms**, while sensor polling occurs at **100 ms**.

* **`ev_control.c / .h` (EV Controller)**
  * Handles the physics-based inertia model for speed calculation.
  * Manages State of Charge (SOC) deduction/regen based on power draw.
  * Adjusts torque scaling based on selected drive mode.
* **`adas.c / .h` (ADAS Engine)**
  * Evaluates sensor distances.
  * Calculates Time To Collision (TTC) = `front_cm / speed_cm_per_s`.
  * Triggers priority-based alarms (`P0-NONE`, `P1-CRITICAL`, `P2-WARNING`, `P3-ADVISORY`).
* **`ultrasonic.c / .h` (Ultrasonic Driver)**
  * Polling-based driver to sequentially trigger and read Echo lengths for the 3 HC-SR04 sensors. Clamps values between 2 cm and 400 cm.
* **`fault.c / .h` (Fault Manager)**
  * Monitors safety boundaries (Motor Temp > 90°C, SOC < 2%, Critical Collision).
  * Forces the state machine into `FAULT` state, cutting motor PWM.
* **`uart_shell.c / .h` (UART Telemetry & Shell)**
  * **TX:** Serializes telemetry into distinct EV (0x01) and ADAS (0x02) binary packets. Sends via DMA.
  * **RX:** Implements a ring buffer for processing incoming ASCII commands.
* **`main.c` (Scheduler)**
  * Initializes HAL, initializes state machines, and manages the main infinite loop/watchdog.

---

## 🚀 Installation & Setup

### 1. Prerequisites
* **STM32CubeIDE**: To compile the C code.
* **PICSimLab**: For hardware simulation.
* **Python 3.8+**: For the dashboard.
* **com0com (Windows) / tty0tty (Linux)**: Virtual serial port emulator to connect PICSimLab to the Python Dashboard.

### 2. Flashing the Firmware (Simulator)
1. Open the project in STM32CubeIDE.
2. Build the project (`Project -> Build All`) to generate the `.hex` or `.elf` file.
3. Open **PICSimLab**. Select `STM32` architecture and `STM32F103C8T6` board.
4. Load the generated `.hex` file.
5. In PICSimLab, open the **Modules** window and connect:
   * 4x Potentiometers (PA0-PA3)
   * 3x HC-SR04 Sensors (PB0-PB5)
   * 4x LEDs (PB8-PB11)
6. Configure the Serial Port in PICSimLab to route to your virtual COM port (e.g., `COM1`).

### 3. Running the Python Dashboard
Navigate to the `PythonDashBoard` directory and install dependencies:
```bash
pip install matplotlib pyserial
```
Run the dashboard (replace `COM2` with the other end of your virtual serial pair):
```bash
python ev_adas_dashboard.py --port COM2 --baud 115200
```

---

## 💻 Usage & UART Commands

While the system is running, you can interact with the vehicle using the UART shell. Send these commands via a serial terminal (like PuTTY or the PICSimLab Serial Terminal):

| Command | Example | Description |
| :--- | :--- | :--- |
| `mode <eco/normal/sport>` | `mode sport` | Switches drive mode, adjusting torque multiplier. |
| `speed set <kmh>` | `speed set 80` | Overrides physical model and injects speed (Testing). |
| `soc set <pct>` | `soc set 45` | Overrides current SOC percentage. |
| `obstacle <cm>` | `obstacle 15` | Forces the front distance to a specific value to trigger TTC/Collision alarms. |
| `blindspot <on/off>` | `blindspot on` | Simulates a vehicle in the blind spot. |
| `fault inject <type>` | `fault inject motor` | Triggers a motor overheat fault manually. |
| `fault clear` | `fault clear` | Clears current faults and resets vehicle to `PARKED` state. |
| `status` | `status` | Prints full system state and variables over UART. |

---

## 🚨 Fault Management & Alarms

The system enforces strict safety states:
* **P1 CRITICAL (Front < 20cm or TTC < 1.5s):** Rapid red LED flash, vehicle forced to `FAULT` state, dashboard flashes red.
* **P2 WARNING (Front < 50cm or TTC < 3.0s):** Steady red LED, amber dashboard warning.
* **P3 ADVISORY (Side < 30cm, Speed > 20km/h):** Steady yellow LED, bird-eye view shows amber side zone.
* **SYSTEM FAULT (Motor > 90°C or SOC < 2%):** Cuts motor power, dashboard shows `FAULT` banner. Must be recovered manually using the `fault clear` UART command. Hysteresis (300 ms) is implemented to prevent alarm bouncing on noisy sensor reads.