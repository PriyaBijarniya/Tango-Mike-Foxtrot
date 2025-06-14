/*
 * coil_control.h - Plasma coil thruster control interface for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Provides functions to initialize and control high-frequency plasma coils
 * used for propulsion and plasma shell generation in the TMF drone.
 */

#ifndef COIL_CONTROL_H
#define COIL_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// Initialize plasma coil hardware (GPIO, timers, DACs, etc.)
void CoilControl_Init(void);

// Set coil drive frequency in Hz (typical range: 10kHz - 1MHz)
// Returns true if frequency successfully set, false otherwise
bool CoilControl_SetFrequency(uint32_t frequency_hz);

// Set coil drive amplitude (0.0 to 1.0 normalized power output)
void CoilControl_SetAmplitude(float amplitude);

// Enable plasma coil drive signal output
void CoilControl_Enable(void);

// Disable plasma coil drive signal output
void CoilControl_Disable(void);

// Read back coil operational status (true if running, false if stopped)
bool CoilControl_IsActive(void);

#endif // COIL_CONTROL_H
