/*
 * flight_control.h - Public API for TMF drone flight control system
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Provides interfaces to initialize and run flight control loops,
 * set target attitude and altitude for autonomous stabilization,
 * and integrates with sensor fusion and coil thrust vectoring.
 */

#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdint.h>

// Initialize flight control system and reset PID controllers
void FlightControl_Init(void);

// Set desired attitude angles in degrees (roll, pitch, yaw)
void FlightControl_SetTargetAngles(float roll_deg, float pitch_deg, float yaw_deg);

// Set desired altitude in meters
void FlightControl_SetTargetAltitude(float altitude_m);

// Flight control loop task (to be run as RTOS thread/task)
void FlightControl_Loop(void *parameters);

#endif // FLIGHT_CONTROL_H
