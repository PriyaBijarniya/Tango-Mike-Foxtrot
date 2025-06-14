/*
 * flight_control.cpp - Flight control stabilization and motor mixing for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Implements PID control loops for roll, pitch, yaw stabilization,
 * motor output mixing, and emergency shutdown.
 */

#include "flight_control.h"
#include <string.h>
#include <math.h>

// PID controllers for roll, pitch, yaw
static PID_Controller_t pid_roll;
static PID_Controller_t pid_pitch;
static PID_Controller_t pid_yaw;

// Current attitude setpoint
static AttitudeSetpoint_t attitude_setpoint;

// Latest motor outputs
static uint16_t motor_outputs[MOTOR_COUNT];

// Helper PID function prototypes
static float PID_Update(PID_Controller_t *pid, float setpoint, float measured, float dt);
static void MotorMix(float roll_output, float pitch_output, float yaw_output, uint16_t *motors);

bool FlightControl_Init(void) {
    // Initialize PID controllers with tuned constants (example values)
    pid_roll.kp = 6.0f; pid_roll.ki = 0.05f; pid_roll.kd = 0.3f;
    pid_pitch.kp = 6.0f; pid_pitch.ki = 0.05f; pid_pitch.kd = 0.3f;
    pid_yaw.kp = 4.0f; pid_yaw.ki = 0.02f; pid_yaw.kd = 0.15f;

    pid_roll.integrator = 0.0f; pid_pitch.integrator = 0.0f; pid_yaw.integrator = 0.0f;
    pid_roll.prev_error = 0.0f; pid_pitch.prev_error = 0.0f; pid_yaw.prev_error = 0.0f;

    pid_roll.output_min = -400.0f; pid_roll.output_max = 400.0f;
    pid_pitch.output_min = -400.0f; pid_pitch.output_max = 400.0f;
    pid_yaw.output_min = -400.0f; pid_yaw.output_max = 400.0f;

    memset(motor_outputs, MOTOR_PWM_MIN, sizeof(motor_outputs));

    attitude_setpoint.roll = 0.0f;
    attitude_setpoint.pitch = 0.0f;
    attitude_setpoint.yaw = 0.0f;

    return true;
}

void FlightControl_SetAttitudeSetpoint(AttitudeSetpoint_t setpoint) {
    attitude_setpoint = setpoint;
}

void FlightControl_Update(IMU_State_t *imu_state) {
    // Assuming fixed dt for simplicity (e.g. 0.01s update rate)
    float dt = 0.01f;

    // Calculate PID outputs for each axis
    float roll_output = PID_Update(&pid_roll, attitude_setpoint.roll, imu_state->roll, dt);
    float pitch_output = PID_Update(&pid_pitch, attitude_setpoint.pitch, imu_state->pitch, dt);
    float yaw_output = PID_Update(&pid_yaw, attitude_setpoint.yaw, imu_state->yaw, dt);

    // Mix PID outputs to motor signals
    MotorMix(roll_output, pitch_output, yaw_output, motor_outputs);
}

void FlightControl_GetMotorOutputs(uint16_t motor_out[MOTOR_COUNT]) {
    memcpy(motor_out, motor_outputs, sizeof(motor_outputs));
}

void FlightControl_MotorStop(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_outputs[i] = MOTOR_PWM_MIN;
    }
}

// --- PID helper function ---
static float PID_Update(PID_Controller_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integrator += error * dt;

    // Anti-windup
    if (pid->integrator > pid->output_max) pid->integrator = pid->output_max;
    else if (pid->integrator < pid->output_min) pid->integrator = pid->output_min;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integrator + pid->kd * derivative;

    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    return output;
}

// --- Motor mixing for quad X-configuration ---
// Motors order: 0=front-left, 1=front-right, 2=rear-right, 3=rear-left
static void MotorMix(float roll_output, float pitch_output, float yaw_output, uint16_t *motors) {
    // Base throttle at midpoint (1500), PID outputs add/subtract to each motor
    float base_throttle = 1500.0f;

    // Mixing matrix (typical quad X)
    // Motor 0: +roll, +pitch, -yaw
    // Motor 1: -roll, +pitch, +yaw
    // Motor 2: -roll, -pitch, -yaw
    // Motor 3: +roll, -pitch, +yaw

    float m0 = base_throttle + roll_output + pitch_output - yaw_output;
    float m1 = base_throttle - roll_output + pitch_output + yaw_output;
    float m2 = base_throttle - roll_output - pitch_output - yaw_output;
    float m3 = base_throttle + roll_output - pitch_output + yaw_output;

    // Clamp outputs
    motors[0] = (uint16_t)(m0 < MOTOR_PWM_MIN ? MOTOR_PWM_MIN : (m0 > MOTOR_PWM_MAX ? MOTOR_PWM_MAX : m0));
    motors[1] = (uint16_t)(m1 < MOTOR_PWM_MIN ? MOTOR_PWM_MIN : (m1 > MOTOR_PWM_MAX ? MOTOR_PWM_MAX : m1));
    motors[2] = (uint16_t)(m2 < MOTOR_PWM_MIN ? MOTOR_PWM_MIN : (m2 > MOTOR_PWM_MAX ? MOTOR_PWM_MAX : m2));
    motors[3] = (uint16_t)(m3 < MOTOR_PWM_MIN ? MOTOR_PWM_MIN : (m3 > MOTOR_PWM_MAX ? MOTOR_PWM_MAX : m3));
}
