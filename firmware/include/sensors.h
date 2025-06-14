/*
 * sensors.h - Sensor abstraction layer for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Defines data structures and interfaces for IMU, GPS,
 * barometer, and magnetometer sensor data acquisition.
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

// IMU sensor raw and processed data structure
typedef struct {
    float accel_x;  // Acceleration in m/s²
    float accel_y;
    float accel_z;
    float gyro_x;   // Angular velocity in deg/s
    float gyro_y;
    float gyro_z;
    float mag_x;    // Magnetometer readings in µT
    float mag_y;
    float mag_z;
    float roll;     // Computed attitude angles in degrees
    float pitch;
    float yaw;
} IMU_Data_t;

// GPS data structure
typedef struct {
    double latitude;      // degrees
    double longitude;     // degrees
    float altitude;       // meters above sea level
    float speed;          // meters per second
    uint8_t fix_type;     // 0 = no fix, 1 = 2D fix, 2 = 3D fix
} GPS_Data_t;

// Barometer data structure
typedef struct {
    float pressure;   // hPa
    float altitude;   // meters, derived from pressure
    float temperature;// Celsius
} Barometer_Data_t;

// Magnetometer data structure (if separate from IMU)
typedef struct {
    float x;  // microteslas
    float y;
    float z;
} Magnetometer_Data_t;

// Initialize all sensors, returns true if successful
bool Sensors_Init(void);

// Update IMU sensor data, returns true if data valid
bool Sensors_UpdateIMU(IMU_Data_t *imu_data);

// Update GPS sensor data, returns true if data valid
bool Sensors_UpdateGPS(GPS_Data_t *gps_data);

// Update barometer data, returns true if data valid
bool Sensors_UpdateBarometer(Barometer_Data_t *baro_data);

// Optional: Update magnetometer data separately if needed
bool Sensors_UpdateMagnetometer(Magnetometer_Data_t *mag_data);

#endif // SENSORS_H
