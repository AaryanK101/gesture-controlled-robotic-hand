# Gesture-Controlled Robotic Hand
A gesture-controlled robotic hand developed as part of the **Circuits and System Design** module at **Trinity College Dublin**

This project uses a flex-sensor glove to detect finger movement and map those gestures onto a servo-actuated robotic hand in real time. The system also includes an OLED interface and an interactive Rock-Paper-Scissors game mode to demonstrate gesture recognition in a more engaging way.

Youtube: https://youtu.be/TnD_lznMmZ8

## Project Overview

The aim of this project was to design and build a complete embedded system that could:

- read finger bend data from a wearable glove
- process and calibrate sensor input in real time
- drive a robotic hand using servo motors
- provide user feedback through an OLED display
- support both direct gesture mirroring and a Rock-Paper-Scissors game mode

What began as a simple gesture replication idea quickly became a broader hardware-software integration project involving calibration, noise filtering, servo control, button-based mode switching, and interface design.

## Key Features

- Real-time gesture replication from glove to robotic hand
- Five flex sensor inputs, one for each finger
- Servo-driven robotic finger actuation
- Runtime calibration for open and closed finger positions
- Smoothed sensor acquisition for more stable readings
- Hysteresis-based thresholding to reduce jitter
- OLED-based user interface
- Two operating modes:
  - Gesture Copy Mode
  - Rock-Paper-Scissors Game Mode

## System Architecture

The system consists of four main parts:

### 1. Sensing
A glove fitted with five flex sensors captures finger bend information. Each flex sensor is read through an analog voltage divider circuit.

### 2. Processing
An Arduino reads the sensor values, smooths the measurements, converts them into resistance estimates, and classifies each finger as open or closed based on calibrated thresholds.

### 3. Actuation
Five servo motors mounted inside the robotic hand reproduce the detected gesture by rotating each finger to its corresponding open or closed position.

### 4. User Interface
An OLED display provides calibration prompts, mode information, finger-state feedback, game status, and score updates.

## Operating Modes

### Gesture Copy Mode
In this mode, the robotic hand mirrors the user’s hand posture in real time. The OLED also displays the open or closed state of each finger.

### Rock-Paper-Scissors Mode
In this mode, the system detects one of three gestures from the glove input:
- Rock
- Paper
- Scissors

The robotic hand then plays against the user by randomly generating its own move, and the OLED displays the round result and updated score.

## Hardware Used

- Arduino Uno
- 5 flex sensors
- 5 servo motors
- Adafruit SSD1306 OLED display
- Breadboard and jumper wires
- Push buttons for mode selection and calibration confirmation
- External power supply for stable servo operation
- 3D-printed robotic hand structure

## Software and Libraries

The project was written in Arduino C/C++ and uses the following libraries:

- `SPI.h`
- `Wire.h`
- `Servo.h`
- `Adafruit_GFX.h`
- `Adafruit_SSD1306.h`

## Calibration and Signal Processing

A major part of the project was ensuring reliable sensor interpretation. Since each finger and sensor behaved slightly differently, runtime calibration was implemented to record open and closed reference values for every finger.

To improve reliability:
- multiple analog readings are averaged
- invalid readings are ignored
- hysteresis thresholds are used to prevent state flickering
- finger states are evaluated independently

This made the system more stable and reduced false gesture detection.

## Challenges and Learning Outcomes

This project highlighted a number of practical engineering challenges that do not usually appear in simulation-only work.

Some of the main lessons included:
- the importance of calibration in real sensor systems
- how noisy analog readings can affect control accuracy
- why hysteresis is useful in binary state detection
- the effect of power stability on servo behaviour
- the value of iterative debugging in embedded systems
- the challenge of integrating sensing, processing, actuation, and UI into one coherent system

Beyond building the final prototype, the project was a strong exercise in embedded design, hardware-software integration, real-time decision making, and system testing.
