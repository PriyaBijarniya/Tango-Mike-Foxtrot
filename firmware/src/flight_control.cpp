/*
 * flight_control.cpp - TMF Drone Flight Control and Stabilization
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Implements sensor fusion (IMU + magnetometer + barometer),
 * PID control loops for attitude, altitude and velocity control,
 * and motor mixing for plasma coil thrust vectoring and auxiliary actuators.
 */

#include "flight_control.h"
#include "sensors.h"
#include "motor_control.h"
#include "coil_control.h"
#include "telemetry.h"
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846f

// PID control gains (tuned for TMF drone)
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Gains_t;

static PID_Gains_t pid_attitude_roll = {3.0f, 0.02f, 0.1f};
static PID_Gains_t pid_attitude_pitch = {3.0f, 0.02f, 0.1f};
static PID_Gains_t pid_attitude_yaw = {2.5f, 0.0f, 0.05f};
static PID_Gains_t pid_altitude = {4.5f, 0.03f, 0.1f};

// Internal state
typedef struct {
    float target_angle_roll;
    float target_angle_pitch;
    float target_angle_yaw;
    float target_altitude_m;
} FlightTarget_t;

static FlightTarget_t flightTarget = {0.0f, 0.0f, 0.0f, 0.0f};

// PID integrators and last errors for derivative
static float integrator_roll = 0.0f, last_error_roll = 0.0f;
static float integrator_pitch = 0.0f, last_error_pitch = 0.0f;
static float integrator_yaw = 0.0f, last_error_yaw = 0.0f;
static float integrator_altitude = 0.0f, last_error_altitude = 0.0f;

static float dt_sec = 0.01f;  // Control loop period ~10ms

// Forward declarations
static float PID_Compute(float setpoint, float measurement, float *integrator, float *last_error, PID_Gains_t gains);
static void ComputeMotorOutputs(float roll_out, float pitch_out, float yaw_out, float altitude_out);

void FlightControl_Init(void)
{
    flightTarget.target_angle_roll = 0.0f;
    flightTarget.target_angle_pitch = 0.0f;
    flightTarget.target_angle_yaw = 0.0f;
    flightTarget.target_altitude_m = 0.0f;

    integrator_roll = 0.0f; last_error_roll = 0.0f;
    integrator_pitch = 0.0f; last_error_pitch = 0.0f;
    integrator_yaw = 0.0f; last_error_yaw = 0.0f;
    integrator_altitude = 0.0f; last_error_altitude = 0.0f;
}

void FlightControl_SetTargetAngles(float roll_deg, float pitch_deg, float yaw_deg)
{
    flightTarget.target_angle_roll = roll_deg;
    flightTarget.target_angle_pitch = pitch_deg;
    flightTarget.target_angle_yaw = yaw_deg;
}

void FlightControl_SetTargetAltitude(float altitude_m)
{
    flightTarget.target_altitude_m = altitude_m;
}

void FlightControl_Loop(void *parameters)
{
    (void)parameters;

    while (1)
    {
        // Read current attitude (roll, pitch, yaw) in degrees from sensor fusion
        float roll_meas = Sensors_GetRoll();
        float pitch_meas = Sensors_GetPitch();
        float yaw_meas = Sensors_GetYaw();

        // Read altitude from barometer in meters
        float altitude_meas = Sensors_GetAltitude();

        // Compute PID outputs for attitude
        float roll_out = PID_Compute(flightTarget.target_angle_roll, roll_meas, &integrator_roll, &last_error_roll, pid_attitude_roll);
        float pitch_out = PID_Compute(flightTarget.target_angle_pitch, pitch_meas, &integrator_pitch, &last_error_pitch, pid_attitude_pitch);
        float yaw_out = PID_Compute(flightTarget.target_angle_yaw, yaw_meas, &integrator_yaw, &last_error_yaw, pid_attitude_yaw);

        // Compute PID output for altitude
        float altitude_out = PID_Compute(flightTarget.target_altitude_m, altitude_meas, &integrator_altitude, &last_error_altitude, pid_altitude);

        // Map PID outputs to motor commands including plasma coil thrust vectoring
        ComputeMotorOutputs(roll_out, pitch_out, yaw_out, altitude_out);

        // Telemetry send for logging and visualization
        Telemetry_SendFlightData(roll_meas, pitch_meas, yaw_meas, altitude_meas);

        vTaskDelay((TickType_t)(dt_sec * 1000)); // Delay for next control cycle
    }
}

static float PID_Compute(float setpoint, float measurement, float *integrator, float *last_error, PID_Gains_t gains)
{
    float error = setpoint - measurement;
    *integrator += error * dt_sec;

    // Anti-windup clamping
    if (*integrator > 100.0f) *integrator = 100.0f;
    if (*integrator < -100.0f) *integrator = -100.0f;

    float derivative = (error - *last_error) / dt_sec;
    *last_error = error;

    return gains.kp * error + gains.ki * (*integrator) + gains.kd * derivative;
}

static void ComputeMotorOutputs(float roll_out, float pitch_out, float yaw_out, float altitude_out)
{
    /*
     * TMF Drone motor mix logic:
     * Four plasma coils arranged in a quad configuration.
     * Control roll, pitch, yaw by differential thrust vectoring.
     * Altitude controlled by overall coil power output.
     *
     * For this design, we map control outputs to c*
