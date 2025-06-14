/*
 * motor_control.h - Motor control interface for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Provides abstraction for throttle and yaw motor control
 * Supports ESC PWM output and yaw servo interface
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// Initialize motor control hardware interfaces (ESCs, servos, etc.)
void MotorControl_Init(void);

// Set overall throttle command (range -1000 to +1000, where 0 = no thrust)
// Positive values increase thrust; negative values allowed for reverse if hardware supports
// Typical ESC expects PWM 1000us (min) to 2000us (max)
void MotorControl_SetThrottle(float throttle_cmd);

// Set yaw control command (range -500 to +500 degrees per second, or normalized units)
// Positive values rotate clockwise, negative counterclockwise
void MotorControl_SetYaw(float yaw_cmd);

// Emergency stop all motors immediately
void MotorControl_EmergencyStop(void);

// Optional: Calibrate ESCs during startup
void MotorControl_CalibrateESC(void);

#endif // MOTOR_CONTROL_H
