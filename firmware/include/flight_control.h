/*
 * flight_control.h - Flight control interface for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Defines data structures and APIs for flight stabilization,
 * motor output control, and command processing.
 */

#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// PID controller data structure
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
    float output;
    float output_min;
    float output_max;
} PID_Controller_t;

// Flight attitude control data
typedef struct {
    float roll;      // Desired roll angle (degrees)
    float pitch;     // Desired pitch angle (degrees)
    float yaw;       // Desired yaw angle (degrees)
    float throttle;  // Throttle (0.0 - 1.0)
} Flight_Command_t;

// Motor outputs (typically 4 motors for quadcopter)
typedef struct {
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} Motor_Output_t;

// Initialize flight control subsystem
bool FlightControl_Init(void);

// Compute motor outputs based on desired commands and current attitude
void FlightControl_Update(const Flight_Command_t *cmd, 
                          float current_roll, float current_pitch, float current_yaw,
                          Motor_Output_t *motors);

// Reset all PID controllers
void FlightControl_Reset(void);

#endif // FLIGHT_CONTROL_H
