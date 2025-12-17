# MazeSolvingBot

MazeSolvingBot is an ESP32-based robot designed to solve mazes autonomously using the Left Wall Following (LSRB) algorithm and avoid obstacles. It features a web-based interface for remote control, monitoring, and configuration, served directly from the ESP32.

## Features

*   **Maze Solving:**
    *   Autonomous maze solving using the Left-Hand Rule (LSRB algorithm).
    *   PID controller for smooth wall following.
    *   Automatic path optimization (removing dead ends) after solving.
    *   "Fast Run" replay mode to execute the optimized path.
*   **Obstacle Avoidance:**
    *   Autonomous navigation avoiding obstacles using Ultrasonic and IR sensors.
*   **Web Interface:**
    *   Hosted on the ESP32 (SPIFFS).
    *   Real-time control (RC mode).
    *   Start/Stop maze solving and obstacle avoidance.
    *   Live tuning of PID parameters and speed.
    *   Maze solution management (save/load paths).
*   **Hardware Control:**
    *   Deep Sleep mode with touch sensor wake-up.
    *   Differential drive motor control.

## Hardware Requirements

*   **Microcontroller:** ESP32 DevKit V1
*   **Motor Driver:** L293D (or compatible H-Bridge)
*   **Sensors:**
    *   2x IR Sensors (Side Walls): Analog output
    *   1x IR Sensor (Front Wall): Analog output
    *   1x IR Sensor (Floor/End Detection): Digital output
    *   1x Ultrasonic Sensor (HC-SR04): Front obstacle detection
*   **Power:** Battery suitable for motors and ESP32 (e.g., 2x 18650 Li-ion)
*   **Chassis:** Differential drive robot chassis

### Pin Configuration

| Component | Pin Name | ESP32 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **Motors** | Left Motor IN3 | 17 | |
| | Left Motor IN4 | 16 | |
| | Left Motor ENB | 4 | PWM Speed Control |
| | Right Motor IN1 | 19 | |
| | Right Motor IN2 | 18 | |
| | Right Motor ENA | 23 | PWM Speed Control |
| **Sensors** | Left IR (IR_1) | 36 (VP) | Analog Input |
| | Right IR (IR_2) | 39 (VN) | Analog Input |
| | Front IR (IR_3) | 34 | Analog Input |
| | Floor IR (IR_4) | 35 | Digital Input |
| | Ultrasonic TRIG | 26 | |
| | Ultrasonic ECHO | 25 | |
| **Other** | Wake Button | 15 | Touch/Button for Wake-up |

## Software Dependencies

This project is built using [PlatformIO](https://platformio.org/). The following libraries are automatically handled by `platformio.ini`:

*   **NewPing:** For ultrasonic sensor control.
*   **L293D:** Custom/Local library for motor driver control.
*   **PID:** For proportional-integral-derivative control logic.
*   **WebSockets:** For real-time communication with the web UI.
*   **ArduinoJson:** For parsing JSON data (settings).

## Installation

1.  **Clone the repository:**
    ```bash
    git clone <repository_url>
    cd MazeSolvingBot
    ```

2.  **Open in PlatformIO:**
    Open the project folder in VS Code with the PlatformIO extension installed.

3.  **Build the Firmware:**
    Click the **Build** button (check mark icon) in the PlatformIO toolbar.

4.  **Upload Firmware:**
    Connect your ESP32 via USB and click the **Upload** button (arrow icon).

5.  **Upload File System (SPIFFS):**
    *   This uploads the web interface files located in the `data/` folder.
    *   In PlatformIO, go to **Project Tasks** -> **Platform** -> **Upload Filesystem Image**.

## Usage

1.  **Power On:** Turn on the robot.
2.  **Connect to WiFi:**
    *   SSID: `ESP32-Robot-Controller`
    *   Password: `1234567-8`
3.  **Open Web Interface:**
    *   Open a web browser and navigate to `http://192.168.4.1` (or the IP printed in the Serial Monitor).
4.  **Control:**
    *   **RC Mode:** Use the on-screen joystick or buttons to drive manually.
    *   **Maze Solving:** Enter a maze name and click "Start Solving".
    *   **Obstacle Avoidance:** Click "Start Obstacle Avoidance".
    *   **Settings:** Adjust PID values and Max Speed in real-time.

## WebSocket API

The robot communicates via WebSocket on port **81**.

| Command | Format | Description |
| :--- | :--- | :--- |
| **RC Control** | `rc:x,y` | Manual drive. `x` (turn), `y` (speed). |
| **Start Solving** | `start_solving:name` | Begins the maze solving task. |
| **Start Obstacle**| `start_obstacle` | Begins obstacle avoidance task. |
| **Load Maze** | `load_maze:path` | Loads a solution path string for replay. |
| **Abort** | `abort` | Stops all autonomous tasks and motors. |
| **Settings** | `settings:{json}` | Updates PID/Speed. JSON: `{"max_speed":.., "kp":.., "ki":.., "kd":..}` |
| **Power Off** | `power_off` | Puts the ESP32 into deep sleep. |
