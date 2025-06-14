/*
 * navigation.h - Navigation control interface for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Provides APIs for autonomous flight control:
 * - Waypoint navigation
 * - Sensor fusion integration
 * - Flight path planning
 * - Velocity and attitude control
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>

// Position structure (latitude, longitude, altitude)
typedef struct {
    double latitude;   // Degrees, WGS84
    double longitude;  // Degrees, WGS84
    float altitude;    // Meters above sea level
} Position_t;

// Velocity structure (m/s in NED frame)
typedef struct {
    float north;   // Velocity north
    float east;    // Velocity east
    float down;    // Velocity down (positive down)
} Velocity_t;

// Attitude structure (Euler angles in degrees)
typedef struct {
    float roll;    // Rotation about X-axis
    float pitch;   // Rotation about Y-axis
    float yaw;     // Rotation about Z-axis (heading)
} Attitude_t;

// Waypoint structure
typedef struct {
    Position_t position;
    float hold_time;    // Seconds to hover at waypoint
} Waypoint_t;

#define MAX_WAYPOINTS 50

// Initialize navigation system and sensor fusion
bool Navigation_Init(void);

// Update navigation loop with sensor inputs and current state
void Navigation_Update(IMU_Data_t *imu, Barometer_Data_t *baro, Position_t *gps_pos);

// Set target waypoints for autonomous flight
bool Navigation_SetWaypoints(Waypoint_t *waypoints, uint8_t count);

// Get current desired velocity command
Velocity_t Navigation_GetVelocityCommand(void);

// Get current desired attitude command
Attitude_t Navigation_GetAttitudeCommand(void);

// Get current target waypoint index
uint8_t Navigation_GetCurrentWaypoint(void);

// Check if mission is complete
bool Navigation_IsMissionComplete(void);

// Abort mission and return control to manual pilot
void Navigation_AbortMission(void);

#endif // NAVIGATION_H
