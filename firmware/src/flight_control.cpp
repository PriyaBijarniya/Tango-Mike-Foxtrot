/*
 * flight_control.cpp - TMF drone flight control implementation
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Implements PID controllers for roll, pitch, yaw stabilization
 * Integrates with IMU sensor fusion (e.g., MPU9250) for attitude estimation
 * Controls thrust vectoring via plasma coils and traditional ESC motors
 * Supports target angle and altitude hold with RTOS task scheduling
 */

#include "flight_control.h"
#include <math.h>
#include <stdbool.h>
#include "imu_sensor.h"
#include "coil_control.h"
#include "motor_control.h"

// PID parameters (tune these for best flight performance)
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_error;
} PIDController;

// PID Controllers for roll, pitch, yaw, altitude
static PIDController pid_roll = {4.0f, 0.01f, 0.1f, 0.0f, 0.0f};
static PIDController pid_pitch = {4.0f, 0.01f, 0.1f, 0.0f, 0.0f};
static PIDController pid_yaw = {3.0f, 0.005f, 0.05f, 0.0f, 0.0f};
static PIDController pid_altitude = {6.0f, 0.02f, 0.3f, 0.0f, 0.0f};

// Target setpoints
static float target_roll_deg = 0.0f;
static float target_pitch_deg = 0.0f;
static float target_yaw_deg = 0.0f;
static float target_altitude_m = 0.0f;

// Sampling time (seconds)
#define CONTROL_LOOP_DT 0.01f  // 100Hz control loop

// Helper PID function
static float PID_Update(PIDController *pid, float setpoint, float measured) {
    float error = setpoint - measured;
    pid->integral += error * CONTROL_LOOP_DT;
    float derivative = (error - pid->last_error) / CONTROL_LOOP_DT;
    pid->last_error = error;
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

// Initialize flight control system
void FlightControl_Init(void) {
    // Reset PID integrals and errors
    pid_roll.integral = 0.0f; pid_roll.last_error = 0.0f;
    pid_pitch.integral = 0.0f; pid_pitch.last_error = 0.0f;
    pid_yaw.integral = 0.0f; pid_yaw.last_error = 0.0f;
    pid_altitude.integral = 0.0f; pid_altitude.last_error = 0.0f;

    // Initialize sensors and motor interfaces
    IMU_Init();
    MotorControl_Init();
    CoilControl_Init();
}

void FlightControl_SetTargetAngles(float roll_deg, float pitch_deg, float yaw_deg) {
    target_roll_deg = roll_deg;
    target_pitch_deg = pitch_deg;
    target_yaw_deg = yaw_deg;
}

void FlightControl_SetTargetAltitude(float altitude_m) {
    target_altitude_m = altitude_m;
}

// Main flight control loop; called periodically by RTOS task (100Hz)
void FlightControl_Loop(void *parameters) {
    while (1) {
        // Read current attitude from IMU sensor fusion
        float current_roll, current_pitch, current_yaw;
        IMU_GetEulerAngles(&current_roll, &current_pitch, &current_yaw);

        // Read current altitude from barometer/altimeter (stub)
        float current_altitude = 0.0f; // TODO: integrate actual sensor

        // Compute PID corrections
        float roll_cmd = PID_Update(&pid_roll, target_roll_deg, current_roll);
        float pitch_cmd = PID_Update(&pid_pitch, target_pitch_deg, current_pitch);
        float yaw_cmd = PID_Update(&pid_yaw, target_yaw_deg, current_yaw);
        float altitude_cmd = PID_Update(&pid_altitude, target_altitude_m, current_altitude);

        // Map PID outputs to coil currents and motor throttle
        // Here, coil currents are used for fine attitude adjustments,
        // motors provide gross thrust and yaw control

        // Example: Set coil currents proportional to roll/pitch commands
        CoilControl_SetCoilCurrent(0, 1000.0f + roll_cmd * 10.0f - pitch_cmd * 10.0f); // Coil 0
        CoilControl_SetCoilCurrent(1, 1000.0f - roll_cmd * 10.0f - pitch_cmd * 10.0f); // Coil 1
        CoilControl_SetCoilCurrent(2, 1000.0f + roll_cmd * 10.0f + pitch_cmd * 10.0f); // Coil 2
        CoilControl_SetCoilCurrent(3, 1000.0f - roll_cmd * 10.0f + pitch_cmd * 10.0f); // Coil 3

        // Motor control: use altitude_cmd and yaw_cmd to set throttle and yaw motors
        MotorControl_SetThrottle(altitude_cmd);
        MotorControl_SetYaw(yaw_cmd);

        // Delay to maintain loop timing (RTOS delay or hardware timer)
        vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms delay for 100Hz loop
    }
}
