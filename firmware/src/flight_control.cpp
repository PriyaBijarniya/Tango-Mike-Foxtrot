/*
 * flight_control.cpp - Flight stabilization and motor mixing for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Implements PID control loops for roll, pitch, yaw stabilization
 * and computes motor outputs accordingly.
 */

#include "flight_control.h"
#include <math.h>
#include <string.h>

static PID_Controller_t pid_roll;
static PID_Controller_t pid_pitch;
static PID_Controller_t pid_yaw;

static void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max);
static float PID_Update(PID_Controller_t *pid, float setpoint, float measured, float dt);

// Constants: tune these for your drone
#define PID_ROLL_KP  6.0f
#define PID_ROLL_KI  0.3f
#define PID_ROLL_KD  0.05f

#define PID_PITCH_KP  6.0f
#define PID_PITCH_KI  0.3f
#define PID_PITCH_KD  0.05f

#define PID_YAW_KP  4.0f
#define PID_YAW_KI  0.2f
#define PID_YAW_KD  0.02f

#define MOTOR_OUTPUT_MIN  0.0f
#define MOTOR_OUTPUT_MAX  1.0f

bool FlightControl_Init(void) {
    PID_Init(&pid_roll, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    PID_Init(&pid_pitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    PID_Init(&pid_yaw, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MAX);
    return true;
}

void FlightControl_Reset(void) {
    memset(&pid_roll, 0, sizeof(pid_roll));
    memset(&pid_pitch, 0, sizeof(pid_pitch));
    memset(&pid_yaw, 0, sizeof(pid_yaw));
}

void FlightControl_Update(const Flight_Command_t *cmd, 
                          float current_roll, float current_pitch, float current_yaw,
                          Motor_Output_t *motors) {
    // Assume dt is fixed timestep; adjust for actual control loop timing in real code
    const float dt = 0.01f; // 10ms typical control loop

    float roll_output = PID_Update(&pid_roll, cmd->roll, current_roll, dt);
    float pitch_output = PID_Update(&pid_pitch, cmd->pitch, current_pitch, dt);
    float yaw_output = PID_Update(&pid_yaw, cmd->yaw, current_yaw, dt);

    // Mix motor outputs for quadcopter (X configuration)
    // Motor layout:
    // motor1: front-left (CCW)
    // motor2: front-right (CW)
    // motor3: rear-right (CCW)
    // motor4: rear-left (CW)
    //
    // Adjust motor speeds by adding/subtracting roll, pitch, yaw PID outputs
    motors->motor1 = cmd->throttle + pitch_output + roll_output - yaw_output;
    motors->motor2 = cmd->throttle + pitch_output - roll_output + yaw_output;
    motors->motor3 = cmd->throttle - pitch_output - roll_output - yaw_output;
    motors->motor4 = cmd->throttle - pitch_output + roll_output + yaw_output;

    // Clamp motor outputs to allowed range
    motors->motor1 = fminf(fmaxf(motors->motor1, MOTOR_OUTPUT_MIN), MOTOR_OUTPUT_MAX);
    motors->motor2 = fminf(fmaxf(motors->motor2, MOTOR_OUTPUT_MIN), MOTOR_OUTPUT_MAX);
    motors->motor3 = fminf(fmaxf(motors->motor3, MOTOR_OUTPUT_MIN), MOTOR_OUTPUT_MAX);
    motors->motor4 = fminf(fmaxf(motors->motor4, MOTOR_OUTPUT_MIN), MOTOR_OUTPUT_MAX);
}

/* --- Internal PID functions --- */

static void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->output = 0.0f;
}

static float PID_Update(PID_Controller_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;

    // Prevent integral windup by clamping
    if (pid->integral > pid->output_max) pid->integral = pid->output_max;
    else if (pid->integral < pid->output_min) pid->integral = pid->output_min;

    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // Clamp output to limits
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    pid->output = output;
    return output;
}
