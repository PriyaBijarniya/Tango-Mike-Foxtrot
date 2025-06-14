# TMF Flight Controller — Core Firmware Architecture and Logic

---

## Overview

The main flight controller (FC) firmware is the heart of the Tango-Mike-Foxtrot drone, integrating propulsion control, stabilization, telemetry, and safety logic. Designed for real-time responsiveness, it manages complex electromagnetic propulsion via coil timing, plasma containment, and sensor fusion.

---

## System Architecture

- **Processor:** STM32F746ZG Cortex-M7 @216 MHz
- **RTOS:** FreeRTOS-based multi-threaded scheduler for task management
- **Modules:**
  - Sensor Fusion & State Estimation
  - Propulsion Coil Timing & Plasma Control
  - Flight Stabilization & Attitude Control
  - Power Management & Thermal Monitoring
  - Telemetry & Communication
  - Safety & Fail-Safe Handlers

---

## 1. Sensor Fusion & State Estimation

- **Sensors:**
  - IMU (BMI270): Accelerometer, Gyroscope, Magnetometer
  - Barometric Pressure Sensor (BMP388)
  - Temperature Sensors on coils (MAX31855)
- **Fusion Algorithm:**
  - Extended Kalman Filter (EKF) combining IMU + barometric altitude
  - Outputs: Position, Velocity, Orientation (Pitch, Roll, Yaw)
- **Update Rate:** 1 kHz sensor polling; 500 Hz EKF update loop

---

## 2. Propulsion Coil Timing & Plasma Control

- **Core Concepts:**
  - Plasma ignition and maintenance via ultra-high-frequency coil pulses (900kHz to 1.2MHz)
  - Phase-locked loop (PLL) controller to synchronize coil firing to plasma oscillation resonance
- **Hardware Interface:**
  - SPI-controlled DACs for precise coil current modulation
  - Temperature feedback loops for dynamic current adjustment
- **Control Loop:**
  - PID controller adjusts coil pulse width modulation (PWM) based on thermal and electromagnetic feedback
  - Safety cutoffs on overcurrent and thermal runaway
- **Startup Sequence:**
  - Precharge capacitors and supercapacitor bank
  - Initial low-power plasma ignition pulse
  - Gradual ramp-up to full coil current

---

## 3. Flight Stabilization & Attitude Control

- **Control Algorithms:**
  - Quaternion-based attitude representation
  - PID loops for pitch, roll, yaw stabilization using IMU data
- **Motor Outputs:**
  - Electronically switch coil phase offsets to vector thrust
  - Integrate small auxiliary fans only for attitude fine-tuning (emergency mode)
- **Failsafe:**
  - Auto-hover mode engages on sensor failure or loss of remote control
  - Emergency plasma shutdown on critical faults

---

## 4. Power Management & Thermal Monitoring

- **Monitoring:**
  - Real-time voltage/current sampling on power rails
  - Thermocouple data from coil mounts
- **Energy Optimization:**
  - Adaptive power draw management balancing flight duration and plasma field strength
  - Load shedding on non-critical systems if voltage drops below threshold
- **Thermal Protection:**
  - Alerts and throttles coil current when coil temps exceed 70°C
  - Forced plasma shutdown at 85°C to prevent damage

---

## 5. Telemetry & Communication

- **Data Streams:**
  - Flight status (orientation, velocity, battery)
  - Coil and plasma parameters (current, temp, power)
  - Error and diagnostic codes
- **Communication:**
  - 2.4 GHz custom FHSS radio link to VR controller (modified DJI FPV or similar)
  - Bluetooth LE for ground station telemetry
  - Firmware update channel via UART/USB

---

## 6. Safety & Fail-Safe Handlers

- Continuous health checks on sensor integrity, coil status, power systems
- Auto plasma shutdown and coil discharge on anomalies
- Emergency landing protocol triggered by manual override or critical system failure

---

## Development Notes

- Firmware written in C++17 with hardware abstraction layers (HAL)
- Unit tests for control algorithms simulated in MATLAB/Simulink prior to deployment
- Real-time trace logging enabled on debug builds for telemetry analysis
- Memory footprint: ~512KB flash, 150KB RAM used
- Modular architecture facilitates future expansions (e.g., AI-assisted flight)

---

## References & Standards

- IEEE Std 829 for software test documentation
- DO-178C guidelines for airborne software assurance (inspired compliance)
- STM32CubeMX middleware libraries
- FreeRTOS Kernel v10.4.6

---

*All code and algorithms will be provided in source files in `/firmware/src/` directory.*

