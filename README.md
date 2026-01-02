# Autonomous Hovercraft Platform (ENGR 290)

**Team 3:**
* Juan Sebastian Holguin Corpas
* Mcwill Buikpor
* Niraj Patel
* Philippe Hadley Plancher
* Chrisjan Alejandro

## Project Overview
This repository contains the source code and design documentation for an autonomous hovercraft built for the ENGR 290 Capstone at Concordia University. The vehicle was designed to navigate a maze, clear obstacles up to 3mm in height, and stop autonomously under a horizontal bar.

**Key Technical Highlight:**
Unlike standard Arduino projects, this firmware is written in **Embedded C (AVR)** using direct register manipulation for the ATmega328P. It features a custom **PID controller** to stabilize the hovercraft's yaw (steering) using an IMU.

## The Hardware (Concept 1)
[cite_start]The design prioritizes weight efficiency (~685g total mass) using a "Concept 1" dual-fan configuration[cite: 314].

* **Microcontroller:** Arduino Nano (ATmega328P)
* [cite_start]**Lift Fan:** Delta AFB1212SH (12V, 6.36W) - Generates ~4mm hover height[cite: 231].
* [cite_start]**Thrust Fan:** Sunon MEC0251V1 (12V, 5.46W)[cite: 41].
* **Sensors:**
    * **IMU:** MPU-6050 (I2C) for gyroscopic stabilization.
    * **Distance:** HC-SR04 Ultrasonic & GP2Y0A21YK0F Infrared.
* [cite_start]**Power:** 2x Gens 450 mAh LiPo Batteries (Series)[cite: 45].

## Software Architecture
The code (`FinalHovercraftCode.c`) allows the system to bypass the Arduino abstraction layer for higher performance.

### Key Features Implemented:
* **PID Control Loop:** Corrects steering drift using Gyroscope Z-axis data (`yaw_deg`).
    * *Kp = 3.0, Ki = 0.1, Kd = 0.8*
* **Direct Register PWM:** Manually configured Timer0 and Timer1 for precise servo and fan speed control.
* **I2C Driver:** Custom implementation of the TWI (Two-Wire Interface) protocol to communicate with the MPU-6050 sensor.
* **State Machine:** Logic to handle "Cruise," "Turn," "Scan," and "Stop" states autonomously.

## Performance & Results
* [cite_start]**Thrust:** Generated 0.603 N of thrust, exceeding the kinetic friction threshold of 0.137 N[cite: 261].
* **Competition Result:** Successfully completed the first turn and one obstacle.
* **Challenges:** The primary failure point was sensor noise handling in the competition environment. [cite_start]The IR/Ultrasonic sensor placement was swapped late in the process, which introduced unhandled edge cases in the navigation logic[cite: 323].

## Gallery
*(Upload your photos from Page 17 of the report here)*
