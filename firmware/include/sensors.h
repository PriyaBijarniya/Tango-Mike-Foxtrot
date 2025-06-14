/*
 * sensors.h - Sensor interface for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Provides abstraction layer for onboard sensors:
 * - IMU (gyro, accelerometer, magnetometer)
 * - Barometer/altimeter
 * - Temperature sensors
 * - Plasma ionization sensors (for propulsion feedback)
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

// Sensor data structures
typedef struct {
    float accel_x; // Acceleration in g
    float accel_y;
    float accel_z;
    float gyro_x;  // Angular velocity deg/s
    float gyro_y;
    float gyro_z;
    float mag_x;   // Magnetic field uT
    float mag_y;
    float mag_z;
} IMU_Data_t;

typedef struct {
    float pressure;     // Atmospheric pressure in hPa
    float altitude;     // Altitude in meters (calculated)
    float temperature;  // Temperature in °C
} Barometer_Data_t;

typedef struct {
    float temperature;  // Propulsion system temperature in °C
    float ionization;   // Ionization sensor output (arbitrary units)
} PlasmaSensor_Data_t;

// Initialize all sensors, I2C/SPI buses, interrupts, DMA, etc.
bool Sensors_Init(void);

// Read IMU data into provided structure, returns true if successful
bool Sensors_ReadIMU(IMU_Data_t *data);

// Read barometer data, returns true if successful
bool Sensors_ReadBarometer(Barometer_Data_t *data);

// Read plasma ionization sensor data, returns true if successful
bool Sensors_ReadPlasmaSensor(PlasmaSensor_Data_t *data);

#endif // SENSORS_H
