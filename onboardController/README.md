## Custom Flight Controller Firmware 
| File                   | Description                                                                                                                                                                |
| ---------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `main.cpp`             | Entry point for the firmware loop. Initializes subsystems, runs pre-flight checks, and manages sensor-actuator cycles during flight.                                       |
| `motorController.cpp`  | Sends PWM signals to each ESC via PCA9685, converting throttle and PID outputs into stable motor commands.                                                                 |
| `radioReceiver.cpp`    | Parses input from the radio receiver, translating PPM signals into throttle, roll, pitch, and yaw commands.                                                                |
| `sensorController.cpp` | Interfaces with the IMU (MPU6050), magnetometer (QMC5883), and barometer (BMP280) to provide fused orientation and altitude data. Handles PID control and safety triggers. |
| `telemetryRelay.cpp`   | Packages and transmits telemetry data (e.g., voltage, angles, error terms) over UART using a key–value system for live ground station updates.                             |

| File                 | Description                                                                                           |
| -------------------- | ----------------------------------------------------------------------------------------------------- |
| `motorController.h`  | Header for motor control logic; defines motor mapping and PWM signal control methods.                 |
| `radioReceiver.h`    | Header defining interface for parsing PPM receiver data into control channels with basic smoothing.   |
| `sensorController.h` | Declares sensor fusion, orientation estimation, PID tuning logic, safety checks, and telemetry hooks. |
| `telemetryRelay.h`   | Declares telemetry interface for buffering and relaying key-value telemetry messages over UART.       |


## Features 
Full PID flight control loop using gyroscope, accelerometer, magnetometer, and barometric pressure

Real-time sensor fusion with basic complementary filtering

Motor control via PWM outputs through PCA9685

UART telemetry relay with structured key-value updates

Voltage monitoring with critical warning triggers

Radio receiver input parsing

Failsafe disarming and sensor readiness checks

Written using the Raspberry Pi Pico SDK (CMake-based)

## Hardware 
MCU: Raspberry Pi Pico

*Sensors:* 
MPU6050 (Accelerometer + Gyroscope)

QMC5883 (Magnetometer)

BMP280 (Barometric Pressure)

*Motor Driver:* PCA9685 16-channel PWM controller

*Frame:* Holybro S500 V2 (salvaged)

*Additional:* Power cables braided from salvaged chargers (when needs must!)

## Safety Features 
Automatic disarming if:

Sensor initialization fails

Altitude drops unexpectedly

Roll or pitch exceeds ±30° (over-tilt safety)

Low-voltage detection with UART warnings and failsafe protocols

## Telemetry 
The TelemetryRelay system relays structured messages over UART. This allows real-time visualization or debugging via a connected ground station (e.g., a Raspberry Pi running a terminal app). Each frame contains telemetry values including: 

- Voltage 
- Orientation (roll, pitch, yaw)
- Altitude
- PID error terms and motor outputs
- Sensor error/reject counts

## Status  
- Untethered test flight completed
️- Crash occurred due to triggered over-tilt safety
️- Ground station (code named Amygdala) is under separate development
- No onboard logging yet; UART telemetry only

## Author 
Samuel Clucas 
Durham University (BSc, Biological Sciences, First Class) 
Incoming MRes student – Biomedical Data Science, Imperial College London

## Purpose 
This firmware was developed to understand the low-level dynamics of drone flight and embedded system design. No libraries or frameworks beyond the Raspberry Pi Pico SDK were used. Everything — from sensor calibration routines to UART telemetry and actuator control — was written from scratch to solidify embedded C++ fluency and systems engineering principles.
