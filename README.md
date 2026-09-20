# Autonomous Lane-Change Robotic Vehicle

An ADAS-inspired autonomous robotic vehicle using ESP32, IR line tracking, ultrasonic obstacle detection, and MPU6050 gyro-guided lane change.

## Overview

This project is an autonomous robotic vehicle designed to follow a predefined lane, detect obstacles, and autonomously change to an alternate lane when the primary path is blocked.

The system combines multiple sensing and control methods on an ESP32 microcontroller:

- 5-channel IR sensor array for line tracking
- HC-SR04 ultrasonic sensor for obstacle detection
- MPU6050 IMU for gyro-guided turning
- L298N dual H-bridge motor driver
- Two TT geared DC motors with differential drive
- ESP32-WROOM as the main controller

The project explores concepts related to Advanced Driver Assistance Systems (ADAS), particularly path tracking, obstacle detection, and lane-change assistance, implemented on a small autonomous robotics platform.

## How It Works

During normal operation, the robot follows a black line using the IR sensor array.

When an obstacle is detected within the defined distance:

1. The robot stops.
2. It performs a gyro-integrated +90° turn.
3. It drives toward the alternate lane.
4. It performs a gyro-integrated -90° turn.
5. It searches for the alternate line.
6. Once the line is detected, autonomous line following resumes.

If the alternate line cannot be detected, the robot enters a safety stop state.

## Hardware

| Component | Purpose |
|---|---|
| ESP32-WROOM DevKit | Main controller |
| 5-channel IR sensor array | Line detection |
| HC-SR04 | Obstacle detection |
| MPU6050 | Gyroscope-based turning |
| L298N | Dual motor driver |
| 2× TT geared DC motors | Differential drive |
| 2×18650 battery pack | Power source |
| LM2596 buck converter | Voltage regulation |
| Caster wheel | Mechanical support |

## Pin Configuration

| Component | ESP32 GPIO |
|---|---|
| IR OUT1 | GPIO34 |
| IR OUT2 | GPIO35 |
| IR OUT3 | GPIO36 |
| IR OUT4 | GPIO4 |
| IR OUT5 | GPIO23 |
| L298N ENA | GPIO25 |
| L298N IN1 | GPIO26 |
| L298N IN2 | GPIO27 |
| L298N ENB | GPIO14 |
| L298N IN3 | GPIO32 |
| L298N IN4 | GPIO33 |
| HC-SR04 TRIG | GPIO18 |
| HC-SR04 ECHO | GPIO19 |
| MPU6050 SDA | GPIO21 |
| MPU6050 SCL | GPIO22 |

## Software

The firmware is written in C++ using the Arduino framework for ESP32.

### Main Control Functions

- IR-based line following
- Obstacle distance measurement
- Gyroscope bias calibration
- Gyro-integrated ±90° turning
- Alternate-lane line search
- Motor PWM control
- Line-loss recovery
- Safety stop

### Main Parameters

- Obstacle detection threshold: 20 cm
- Forward speed: 140 PWM
- Line turning speed: 180 PWM
- Lane-change drive speed: 200 PWM
- Gyro turn speed: 255 PWM
- Line-loss recovery time: 400 ms
- Alternate-line search time: up to 5 seconds

## Libraries

The project uses:

- Wire
- Adafruit MPU6050
- Adafruit Unified Sensor

## Author
Piyush Sinha

## GitHub
https://github.com/Piyushsinha-ece

## Portfolio
https://piyushsinha-ece.github.io

## LinkedIn
https://www.linkedin.com/in/piyushsinha-ece/

## Copyright
Copyright © 2026 Piyush Sinha. All rights reserved.
This source code is provided for viewing and educational reference only. Reuse, redistribution, modification or incorporation into other projects requires prior written permission from the author.
