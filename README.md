# CropDrop Bot – Autonomous Line Following & Color Sorting Robot

## Overview

CropDrop Bot is an autonomous mobile robot designed to navigate a warehouse-style arena, identify colored objects, and sort them into their respective zones. The robot follows a predefined path using a PID-based line-following algorithm, detects and classifies objects using a color sensor, and manipulates them using an electromagnet-based pickup mechanism.

The system is built around the **STM32F103RB Nucleo-64** development board and implemented using **STM32CubeIDE** and the **STM32 HAL** framework.

---

## Features

* PID-based line following using a 5-channel IR sensor array
* Real-time line position estimation using ADC + DMA
* Autonomous object pickup using an electromagnet
* RGB color detection using a TCS3200-style color sensor
* Automatic sorting of objects into matching colored bins
* Junction detection and navigation logic
* Differential drive motor control using PWM
* UART-based debugging and monitoring
* Task completion indication through RGB status LEDs

---

## System Architecture

```text
IR Sensor Array
       │
       ▼
 PID Controller
       │
       ▼
 Motor Control (PWM)
       │
       ▼
 Line Navigation
       │
       ├── Object Detection
       │
       ▼
 Electromagnet Pickup
       │
       ▼
 Color Detection
       │
       ▼
 Sorting Logic
       │
       ▼
 Correct Bin Placement
```

---

## Hardware Components

| Component                 | Purpose                   |
| ------------------------- | ------------------------- |
| STM32F103RB Nucleo-64     | Main Controller           |
| 5 IR Reflectance Sensors  | Line Tracking             |
| Dual DC Motors + H-Bridge | Differential Drive        |
| TCS3200 Color Sensor      | Object Color Detection    |
| Electromagnet             | Object Pickup and Release |
| Object Detection Sensor   | Presence Detection        |
| RGB Status LED            | Visual Feedback           |
| USART2 Interface          | Debug Communication       |

---

## Software Features

### PID Line Following

The robot continuously estimates the line position using five IR sensors and calculates the tracking error using a weighted-average approach.

The PID controller computes the correction required to keep the robot centered on the line:

```c
Kp = 1.5f;
Ki = 0.055f;
Kd = 0.4f;
```

This correction is applied as a differential adjustment to the left and right motor speeds.

---

### Object Pickup

When an object is detected:

1. Robot stops at the pickup point.
2. Electromagnet is energized.
3. Object is securely attached.
4. Robot resumes navigation.

---

### Color Classification

The TCS3200 sensor measures the reflected RGB components of the object.

Detected colors:

* Red
* Green
* Blue
* Unknown/Mixed

The detected color is displayed using the RGB status LED and stored for sorting decisions.

---

### Sorting Logic

While traversing the return path:

* Junctions are detected using the IR sensor array.
* The robot counts junction crossings.
* Based on the detected color, a predefined drop location is selected.
* The electromagnet is deactivated at the target bin.

---

## Peripheral Configuration

### ADC1 + DMA

* Continuous scan mode
* Five IR sensors sampled simultaneously
* DMA transfers sensor data into memory buffer

### TIM2

* PWM generation for motor control
* Differential drive operation

### TIM3

* Input Capture Mode
* Frequency measurement for color sensing

### USART2

* 115200 baud
* Debug and diagnostic messages

### GPIO

* Electromagnet Control
* RGB LED Control
* Color Sensor Filter Selection
* Object Detection Interrupt

---

## Project Structure

```text
Core/
├── Inc/
│   ├── Color.h
│   ├── main.h
│   └── stm32f1xx_*.h
│
├── Src/
│   ├── main.c
│   ├── Color.c
│   ├── stm32f1xx_it.c
│   └── stm32f1xx_hal_msp.c
│
Drivers/
├── CMSIS/
└── STM32 HAL Drivers/
```

---

## Development Tools

* STM32CubeIDE
* STM32 HAL Library
* STM32CubeMX
* ST-Link Debugger

---

## Results

* Reliable line tracking using PID control
* Accurate object pickup and placement
* Real-time color classification
* Autonomous sorting into designated bins
* Stable navigation through multiple junctions

---

## Future Improvements

* Dynamic PID auto-tuning
* Obstacle avoidance capability
* Wireless telemetry using ESP32
* Multiple-object scheduling and route optimization
* Machine vision-based object recognition

---

## Author

**Prajwal P Rao**
Electronics & Communication Engineering
Manipal Institute of Technology

---

## License

This project is intended for educational and research purposes.
