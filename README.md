# EmbeddedComputing

This repository contains a collection of lab exercises and projects focused on embedded computing, with an emphasis on RTOS (Real-Time Operating Systems) and hardware interfacing. The projects are designed to explore various sensors, displays, communication protocols, and task scheduling techniques used in embedded systems.

## Contents

- **Lab2 - PIO Driver**: Introduction to Peripheral I/O drivers.
- **Lab3 - OLED Introduction**: Display data on an OLED screen.
- **Lab4 - Queues & Tasks**: Task scheduling and queue management in RTOS.
- **Lab5 - Distance Sensor**: Interface with a distance sensor using RTOS.
- **Lab6 - Gyroscope & Semaphores**: Use gyroscopes and semaphores for task synchronization.
- **Lab7 - Digital Clock**: Build a digital clock using an LCD display and RTOS.
- **Lab8 - Calendar with RTT**: Display a calendar using the RTT library on an OLED.
- **Lab9 - Temperature Sensor**: Interface a temperature sensor with an LCD using RTOS.
- **Lab10 - WiFi Connection (WINC1500)**: Connect to WiFi and send/receive data.
- **Project 1 - MIDI Controller**: Build a MIDI controller for musical applications.

## Requirements

- **Board**: ATMEL SAM E70
- **Programming Language**: C (91.9%)
- **Build System**: Makefile (8.1%)
- **Hardware**: Various sensors and peripheral devices (OLED, TFT-LCD, etc.)

## Board Pin Configuration

Below is the pin configuration for the **ATMEL SAM E70** board used in these projects and the complete datasheet!

![SAM E70 Pin Configuration](./img/images.jfif)
[SAM E70 Xplained Ultra Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/80000767C.pdf)

This image shows the pin assignments and peripheral interfaces on the SAM E70 Xplained Ultra Evaluation Kit, which is essential for hardware connections and debugging.

## Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/arthurfonsek/EmbeddedComputing.git
   cd EmbeddedComputing
   ```
