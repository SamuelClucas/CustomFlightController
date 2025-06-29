# Engineering a Drone Flight Controller
I worked on this project between February to early-April 2025. I developed a fascination for embedded programming and set out to implement a perception-to-action flight model on a drone running my own flight controller firmware. While I only managed a single, brief, maiden flight in this time, I learned so much and deeply enjoyed doing so. Note: I had **very** limited resources for this project.

<img src="./lib/drone.jpg" width="300" height="300">

## Ground Control Station 

Terminal-based real-time telemetry parser and state monitor for autonomous flight experiments

### Overview
This module forms the ground control interface for a custom quadcopter drone project. Written in C++ and designed to run on a Raspberry Pi (or similar Linux-based system), it connects to the drone over UART and interprets live telemetry frames sent from the onboard flight controller (running on a Raspberry Pi Pico).

This implementation prioritizes minimal latency, code transparency, and terminal-friendly feedback. The code was part of a larger embedded systems engineering project that involved custom drone construction, real-time sensor fusion, and manual PCB-level integration.

### Features
UART serial communication using /dev/ttyUSB0

Real-time telemetry parsing and printing

Signal-aware loop with graceful SIGINT interrupt handling

Status-based output with ANSI terminal colouring

Modular Receiver class with persistent state

Simple multithread-safe control loop via std::atomic<bool>


### Running the Ground Station
Make sure your user has permission to access /dev/ttyUSB0 (or appropriate device).
You can build with:

g++ -std=c++17 -pthread main.cpp receiver.cpp -o groundstation
./groundstation

#### Interrupt with CTRL+C to shut down cleanly.

### Expected Output
If telemetry is flowing properly from the drone:

Ground control online...

Receiver state: ONLINE

Successfully opened file descriptor!

Parsing incoming signal...

roll=0.02,pitch=-0.01,yaw=0.03,...

The Receiver class parses key-value telemetry frames and prints them directly with terminal coloring.

### Integration with Drone Firmware
The [onboard flight controller](./onboardController/README.md) sends telemetry frames (e.g., roll, pitch, yaw, altitude, voltage, PID errors) over UART. This station listens, parses, and prints those updates in real time.

Future versions may include: 
- Logging
- Signal quality monitoring
- A minimal dashboard

## Maiden Flight 
The drone flew untethered during its maiden test before triggering an overtilt-safety cutoff. A short [telemetry log](./lib/maiden_flight.md) is included.

A video and photo of the flight are available in the lib/ directory (this was a special moment!).

[![Watch the maiden flight](lib/flight_thum.png)](lib/maiden_flight.mp4) 

(volume warning!)

## Author 
Samuel Clucas
Durham University – BSc Biological Sciences (First Class)
Incoming MRes – Biomedical Data Science, Imperial College London
