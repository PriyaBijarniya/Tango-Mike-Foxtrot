/*
 * coil_control.h - Public API for plasma coil driver
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Provides functions to initialize, control, and monitor
 * plasma coils used for Darkstar plasma shell propulsion.
 */

#ifndef COIL_CONTROL_H
#define COIL_CONTROL_H

#include <stdint.h>

// Initialize plasma coil control peripherals (DAC, PWM timers)
void CoilControl_Init(void);

// Coil control main task, runs periodic control loop (should be started as RTOS task)
void CoilControlTask(void *parameters);

// Set coil current setpoint in milliamps (0 to max safe current)
void SetCoilCurrentSetpoint(float milliamps);

// Get latest measured coil current in milliamps
float GetCoilCurrentMeasured(void);

// Get latest measured coil voltage in volts
float GetCoilVoltageMeasured(void);

// Set current setpoint for individual coils (index 0 to 3)
void CoilControl_SetCoilCurrent(uint8_t coilIndex, float current_mA);

#endif // COIL_CONTROL_H
