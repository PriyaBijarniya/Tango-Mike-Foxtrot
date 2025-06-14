/*
 * flight_control.h - Flight control interface for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Provides APIs to manage stabilization, PID control loops,
 * motor output mixing, and actuator commands.
 */

#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// Motor output range (PWM microseconds typical)
#define MOTOR_PWM_MIN 1000
#define MOTOR_PWM_MAX 2000
#define MOTOR_COUNT 4

// PID controller structure
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
    float output_min;
    float output_max;
} PID_Controller_t;

// Attitude setpoint structure (roll, pitch, yaw in degrees)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} AttitudeSetpoint_t;

// Sensor input data structure
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
} IMU_State_t;

// Initialize flight control module and PID parameters
bool FlightControl_Init(void);

// Set desired attitude setpoints from navigation
void FlightControl_SetAttitudeSetpoint(AttitudeSetpoint_t setpoint);

// Update flight control loop with current IMU sensor state
// Outputs motor commands to ESCs
void FlightControl_Update(IMU_State_t *imu_state);

// Get current motor outputs (PWM microseconds)
void FlightControl_GetMotorOutputs(uint16_t motor_outputs[MOTOR_COUNT]);

// Emergency motor shutdown
void FlightControl_MotorStop(void);

#endif // FLIGHT_CONTROL_H
