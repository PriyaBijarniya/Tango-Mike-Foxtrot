/*
 * sensors.cpp - Sensor hardware interface for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Interfaces with hardware sensors over SPI/I2C/UART
 * Provides raw and processed sensor data for flight control.
 */

#include "sensors.h"
#include <string.h>
#include <math.h>
#include "hardware_drivers.h" // Abstracts low-level SPI/I2C/UART

static IMU_Data_t imu_cache;
static GPS_Data_t gps_cache;
static Barometer_Data_t baro_cache;

// Internal helper prototypes
static bool IMU_ReadRaw(float *accel, float *gyro, float *mag);
static void IMU_ComputeEulerAngles(IMU_Data_t *imu);
static bool GPS_ParseData(char *nmea_sentence);
static bool Baro_ReadPressureTemp(float *pressure, float *temperature);

bool Sensors_Init(void) {
    bool imu_ok = IMU_Init();
    bool gps_ok = GPS_Init();
    bool baro_ok = Baro_Init();

    return imu_ok && gps_ok && baro_ok;
}

bool Sensors_UpdateIMU(IMU_Data_t *imu_data) {
    float accel[3], gyro[3], mag[3];

    if (!IMU_ReadRaw(accel, gyro, mag)) return false;

    imu_cache.accel_x = accel[0];
    imu_cache.accel_y = accel[1];
    imu_cache.accel_z = accel[2];
    imu_cache.gyro_x = gyro[0];
    imu_cache.gyro_y = gyro[1];
    imu_cache.gyro_z = gyro[2];
    imu_cache.mag_x = mag[0];
    imu_cache.mag_y = mag[1];
    imu_cache.mag_z = mag[2];

    IMU_ComputeEulerAngles(&imu_cache);

    if (imu_data) memcpy(imu_data, &imu_cache, sizeof(IMU_Data_t));
    return true;
}

bool Sensors_UpdateGPS(GPS_Data_t *gps_data) {
    char nmea_sentence[128];

    if (!GPS_ReadLine(nmea_sentence, sizeof(nmea_sentence))) return false;
    if (!GPS_ParseData(nmea_sentence)) return false;

    if (gps_data) memcpy(gps_data, &gps_cache, sizeof(GPS_Data_t));
    return true;
}

bool Sensors_UpdateBarometer(Barometer_Data_t *baro_data) {
    float pressure, temperature;

    if (!Baro_ReadPressureTemp(&pressure, &temperature)) return false;

    baro_cache.pressure = pressure;
    baro_cache.temperature = temperature;
    // Simplistic altitude calc assuming standard atmosphere
    baro_cache.altitude = 44330.0f * (1.0f - powf(pressure / 1013.25f, 0.1903f));

    if (baro_data) memcpy(baro_data, &baro_cache, sizeof(Barometer_Data_t));
    return true;
}

/* --- Internal hardware interface stubs --- */

static bool IMU_ReadRaw(float *accel, float *gyro, float *mag) {
    // Replace with actual hardware SPI/I2C reads for your IMU (e.g., BMI270 or similar)
    // Stub: return dummy stabilized values for development
    accel[0] = 0.0f; accel[1] = 0.0f; accel[2] = 9.81f; // gravity
    gyro[0] = 0.0f; gyro[1] = 0.0f; gyro[2] = 0.0f;
    mag[0] = 0.3f; mag[1] = 0.0f; mag[2] = 0.5f;
    return true;
}

static void IMU_ComputeEulerAngles(IMU_Data_t *imu) {
    // Simple complementary filter or Madgwick/Mahony could be implemented here
    // For demo, we'll calculate pitch and roll from accel only (radians)
    float ax = imu->accel_x;
    float ay = imu->accel_y;
    float az = imu->accel_z;

    imu->roll = atan2f(ay, az) * 57.2958f;  // degrees
    imu->pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
    // Yaw requires magnetometer + fusion; stub as zero
    imu->yaw = 0.0f;
}

static bool GPS_ParseData(char *nmea_sentence) {
    // Parse NMEA sentences to update gps_cache; simplified example:
    // Real implementation requires full NMEA parser
    // Here, we stub fixed coordinates for testing
    gps_cache.latitude = 37.7749;   // San Francisco approx
    gps_cache.longitude = -122.4194;
    gps_cache.altitude = 15.0f;
    gps_cache.speed = 0.0f;
    gps_cache.fix_type = 2;          // 3D fix assumed
    return true;
}

static bool Baro_ReadPressureTemp(float *pressure, float *temperature) {
    // Stub with fixed sea level pressure and room temp
    *pressure = 1013.25f;  // hPa
    *temperature = 25.0f;  // Celsius
    return true;
}
