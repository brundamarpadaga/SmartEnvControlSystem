# Smart Environmental Control System



**Course**: ECE 544 - Embedded Systems Design, Winter 2025  
**Team**: Ashten Bontrager, Brunda Marpadaga, Nikolay Nikolov  
**Professor**: Roy Kravitz  
**TA**: Victoria Van Gaasbeck  

## About This Code

This repository contains the source code for an embedded system that controls temperature, humidity, and light in a small enclosure, simulating a greenhouse. Built for the Nexys A7 FPGA board with a MicroBlaze processor and FreeRTOS, it uses sensors to monitor conditions and PID logic to adjust a fan and LEDs. The code includes drivers for the TSL2561 light sensor, BME280 environmental sensor, and an OLED display, along with tasks to handle user inputs and display updates.

### Key Components
- **`main.c`**: Core logic with PID control for fan and LEDs.
- **`tsl2561.c`**: Driver for reading light levels.
- **`bme280.c`**: Driver for temperature and humidity data.
- **`lcd.c`**: Manages OLED display output.

### Purpose
Developed as an academic project for ECE 544 (Winter 2025) to demonstrate embedded system design and real-time control.

---

## Disclaimer

This is an academic project developed as part of ECE 544 at [Your University Name] for learning purposes only. It is not intended for commercial use, production environments, or real-world deployment beyond educational demonstration.

---

## Overview

The Smart Environmental Control System is an embedded solution built on the Nexys A7 FPGA board with a MicroBlaze soft processor. It regulates temperature, humidity, and light in a small enclosure, mimicking a greenhouse environment for plant growth. Using sensors, actuators, and PID control, the system provides real-time monitoring and dynamic adjustments, with feedback displayed via seven-segment displays and LEDs.

### Key Features
- **Monitoring**: Tracks temperature (°F), humidity (%), and ambient light (lux) using I2C and ADC sensors.
- **PID Control**: Maintains user-defined setpoints (e.g., 74°F, 50% humidity) with smooth fan speed adjustments.
- **User Interface**: Buttons adjust setpoints; switches toggle fan speed manually (25%, 50%, 75%).
- **Visual Feedback**: 
  - Seven-segment displays toggle setpoints and current conditions every 2 seconds.
  - PWM-driven LEDs indicate status (green: within setpoints, red: temp high, blue: humidity high).
- **Actuators**: Fan for cooling, white LED for light, infrared LED for heating simulation.

## Hardware Components

- **Nexys A7 FPGA Board**: Core platform with MicroBlaze, GPIO, and onboard displays/buttons/switches.
- **Sensors**:
  - TSL2561 (I2C): Ambient light sensor.
  - BME280 (I2C): Temperature, humidity, and pressure sensor.
- **Actuators**:
  - DC Fan: PWM-controlled for cooling.
  - RGB LEDs: Green, red, blue, white, infrared (PWM-driven).
- **Enclosure**: Small box simulating a greenhouse.

## Software Architecture

Developed in Xilinx Vivado and Vitis using FreeRTOS:
- **Drivers**:
  - `tsl2561.c`: Initializes and reads lux from the TSL2561 sensor.
  - `bme280.c`: Configures and compensates data from the BME280 sensor.
  - `lcd.c`: Manages OLED display updates for temp, humidity, and pressure.
- **PID Control** (`main.c`):
  - `PID_Task`: Calculates corrections for lux, temp, and humidity, adjusting actuators.
  - `pid_init`, `pid_funct`: Implements PID algorithm (P, I, D terms).
  - `correctedSignal`: Applies PID output to PWM duty cycles.
- **PID Control Details**:
  - **Structures**: Three PID structs (`pidLux`, `pidTemp`, `pidHum`) manage light, temperature, and humidity with initial setpoints (200 lux, 21°C, 35%) and gains (Kp=1, Ki=0.02, Kd=0.001).
  - **Algorithm**: `pid_funct` computes correction as `(P + I + D) / setpoint`, where:
    - Proportional (P): `Kp * error` adjusts based on current deviation.
    - Integral (I): `Ki * integral(error * delta_t)` accumulates error, clamped to ±512.
    - Derivative (D): `Kd * (delta_error / delta_t)` responds to change rate.
  - **Sampling**: Lux sampled every ~420ms (TSL2561), temp/humidity every ~1s (BME280).
  - **Output**: Correction percentage adjusts PWM duty cycles (e.g., fan: 25%-75%, LEDs: 0%-100%) via `correctedSignal`.
  - **Tuning**: Tested with hair dryer to smooth fan transitions (e.g., 70°F low, 90°F high).
- **Tasks**:
  - `Parse_Input_Task`: Processes button/switch inputs.
  - `Display_Task`: Updates seven-segment display.
  - `LCD_Task`: Drives OLED display.
- **Synchronization**: Uses semaphores (`i2c_sem`, `oled_sem`) and queues (`toPID`, `fromPID`) for task coordination.



