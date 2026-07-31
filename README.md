# STM32 Smart AGV with Autonomous Navigation & Gesture Control

A FreeRTOS-based intelligent mobile robot built on the **STM32 Nucleo-F411RE** that supports both **autonomous maze navigation** and **wireless gesture-based manual control**.

The robot autonomously navigates using three ultrasonic sensors while avoiding obstacles. It can also be switched into manual mode, where an **ESP32 + MPU6050 motion controller** communicates with the robot through an **HC-05 Bluetooth module**, allowing intuitive hand-gesture control.

---

## Features

- FreeRTOS multitasking architecture
- Autonomous obstacle avoidance
- 3 Ultrasonic Sensors
- L298N motor driver
- 4 TT DC motors
- Bluetooth Manual Mode
- ESP32 Motion Controller
- MPU6050 Gesture Recognition
- Automatic Bluetooth Reconnection
- Manual Safety Override
- Front Collision Prevention
- Modular source code

---

## Modes

Blue USER Button cycles through:

```
STOP
   ↓
AUTO
   ↓
MANUAL
   ↓
STOP
```

### STOP

- Motors disabled

### AUTO

- Autonomous maze navigation
- Chooses safest direction using three ultrasonic sensors
- Differential steering

### MANUAL

Controlled wirelessly through an ESP32 hand controller.

The front ultrasonic sensor remains active as a collision safety feature.

---

# Hardware

## Robot

- STM32 Nucleo-F411RE
- L298N Dual H-Bridge
- 4× TT Gear Motors
- 3× HC-SR04 Ultrasonic Sensors
- HC-05 Bluetooth Module

## Hand Controller

- ESP32 Dev Board
- MPU6050
- Li-ion Battery

---

# System Architecture

```
              HAND CONTROLLER

      +-------------------------+
      | ESP32                   |
      | MPU6050                 |
      | Bluetooth Classic       |
      +------------+------------+
                   |
             HC-05 Bluetooth
                   |
                   |
      +------------v------------+
      | STM32 Nucleo F411RE     |
      | FreeRTOS                |
      +------------+------------+
                   |
          +--------+--------+
          |                 |
     AUTO MODE        MANUAL MODE
          |                 |
 Ultrasonic Maze      Bluetooth Commands
 Navigation                 |
          \                 /
           \               /
            +-------------+
            | Motor Driver|
            |   L298N      |
            +-------------+
                   |
              4 TT Motors
```

---

# Bluetooth Commands

The ESP32 transmits simple ASCII commands.

| Command | Action |
|----------|--------|
| F | Forward |
| B | Backward |
| L | Rotate Left |
| R | Rotate Right |
| FL | Forward Left |
| FR | Forward Right |
| BL | Backward Left |
| BR | Backward Right |
| S | Stop |

---

# Gesture Mapping

| Hand Motion | Robot Action |
|--------------|--------------|
| Flat | Stop |
| Tilt Forward | Forward |
| Tilt Backward | Backward |
| Tilt Left | Rotate Left |
| Tilt Right | Rotate Right |
| Forward + Left | Forward Left |
| Forward + Right | Forward Right |

---

# FreeRTOS Tasks

- ModeManagerTask
- UltrasonicTask
- AutoDriveTask
- ManualDriveTask
- BluetoothTask

---

# Autonomous Navigation

Three ultrasonic sensors continuously monitor the surroundings.

The robot:

- Detects frontal obstacles
- Compares left and right clearance
- Selects the direction with more available space
- Uses differential steering to navigate through unknown environments

---

# Manual Safety

Even in Manual Mode:

- Front obstacle detection remains active
- Unsafe forward commands are ignored
- Robot automatically stops before collision

---

# Project Structure

```
Core/
├── Inc/
│   ├── auto_drive.h
│   ├── bluetooth.h
│   ├── manual_drive.h
│   ├── mode_manager.h
│   ├── motor.h
│   └── ultrasonic.h
│
└── Src/
    ├── auto_drive.c
    ├── bluetooth.c
    ├── manual_drive.c
    ├── mode_manager.c
    ├── motor.c
    └── ultrasonic.c
```

---

# Future Improvements

- PID Wall Following
- IMU Sensor Fusion
- SLAM Mapping
- Camera Streaming
- Path Planning
- Firefighting Robot Conversion
- Wi-Fi Telemetry Dashboard
- Voice Commands
- Mobile Application
- ROS Integration

---

# Technologies

- STM32CubeIDE
- STM32 HAL
- FreeRTOS
- C
- ESP32 Arduino
- Bluetooth Classic
- MPU6050
- HC-SR04
- L298N

---

# Demonstration

Autonomous Navigation

⬜

Gesture Controlled Driving

⬜

---

## Author

Jacob Kwon

Built as a robotics and embedded systems project demonstrating autonomous navigation, real-time multitasking, wireless communication, and intuitive gesture-based robot control.
