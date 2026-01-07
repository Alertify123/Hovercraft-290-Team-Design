# Autonomous Hovercraft Platform (ENGR 290)

**Team 3:**
* Chrisjan Alejandro
* Juan Sebastian Holguin Corpas
* Mcwill Buikpor
* Niraj Patel
* Philippe Hadley Plancher


## Project Overview
This repository contains the source code and design documentation for our autonomous hovercraft built for the ENGR 290 Mini-Capstone at Concordia University. The vehicle was designed to navigate a maze, clear obstacles up to 3mm in height, and stop autonomously under a horizontal bar.

**Technical Highlight:**
Unlike standard Arduino projects, this firmware is written in **Embedded C (AVR)** using direct register manipulation for the ATmega328P microcontroller. It features a custom **PID controller** to stabilize the hovercraft's yaw steering using an IMU.

## The Hardware (Concept 1)
The design prioritizes weight efficiency (~685g total mass) using a "Concept 1" dual-fan configuration.

* **Microcontroller:** Arduino Nano (ATmega328P)
* **Lift Fan:** Delta AFB1212SH (12V, 6.36W) - Generates ~4mm hover height.
* **Thrust Fan:** Sunon MEC0251V1 (12V, 5.46W).
* **Sensors:**
    * **IMU:** MPU-6050 (I2C) for gyroscopic stabilization.
    * **Distance:** HC-SR04 Ultrasonic & GP2Y0A21YK0F Infrared.
* **Power:** 2x Gens 450 mAh LiPo Batteries (Series).

## Software Architecture
The code (`FinalHovercraftCode_290_TEAM3_FALL_2025.c`) allows the system to bypass the Arduino abstraction layer for higher performance.

### Key Features Implemented:
* **PID Control Loop:** Corrects steering drift using Gyroscope Z-axis data (`yaw_deg`).
    * *Kp = 3.0, Ki = 0.1, Kd = 0.8*
* **Direct Register PWM:** Manually configured Timer0 and Timer1 for precise servo and fan speed control.
* **I2C Driver:** Custom implementation of the TWI (Two-Wire Interface) protocol to communicate with the MPU-6050 sensor.
* **State Machine:** Logic to handle "Cruise," "Turn," "Scan," and "Stop" states autonomously.

## Performance & Results
* **Thrust:** Generated 0.603 N of thrust, exceeding the kinetic friction threshold of 0.137 N.
* **Competition Result:** Successfully completed the first turn and one obstacle.
* **Challenges:** The primary failure point was sensor noise handling in the competition environment. The IR/Ultrasonic sensor placement was swapped late in the process, which introduced unhandled edge cases in the navigation logic.

## Gallery
| Drawing | Model |
| :---: | :---: |
| ![Drawing](drawing.png) | ![Model](model.png) |

<br>

<h3>Competition Performance</h3>
<table>
  <tr>
    <th width="50%">The Setup</th>
    <th width="50%">Test Run</th>
  </tr>
  <tr>
    <td valign="top">
      <img src="final_comp.png" alt="Competition Setup" style="width: 100%;">
    </td>
    <td valign="top">
      <video src="https://github.com/user-attachments/assets/115affdc-731b-40f7-8c52-8d15a36279e1" controls="controls" style="max-width: 100%;">
      </video>
    </td>
  </tr>
</table>
