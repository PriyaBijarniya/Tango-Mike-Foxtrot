/*
 * sensors.cpp - Sensor driver implementation for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Handles initialization and data acquisition from onboard sensors:
 * - IMU (MPU9250 or equivalent)
 * - Barometer (BMP280 or equivalent)
 * - Plasma ionization sensor (custom analog input)
 */

#include "sensors.h"
#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include <string.h>

// Peripheral handles (extern or static based on your MCU setup)
extern I2C_HandleTypeDef hi2c1;   // For IMU and Barometer
extern ADC_HandleTypeDef hadc1;   // For plasma sensor analog input

// IMU sensor I2C address (MPU9250 example)
#define IMU_I2C_ADDR 0x68 << 1

// Barometer I2C address (BMP280 example)
#define BARO_I2C_ADDR 0x76 << 1

// Plasma sensor ADC channel (example)
#define PLASMA_ADC_CHANNEL ADC_CHANNEL_5

// Internal functions for low-level sensor interaction
static bool IMU_Init(void);
static bool Barometer_Init(void);
static bool ReadIMURaw(uint8_t *buffer, size_t length);
static bool ReadBarometerRaw(uint8_t *buffer, size_t length);
static bool ReadPlasmaRaw(uint16_t *adc_value);

bool Sensors_Init(void) {
    bool imu_ok = IMU_Init();
    bool baro_ok = Barometer_Init();
    // ADC assumed initialized in main setup
    return imu_ok && baro_ok;
}

bool Sensors_ReadIMU(IMU_Data_t *data) {
    uint8_t raw[14];
    if (!ReadIMURaw(raw, sizeof(raw))) return false;

    // Convert raw data to physical units (assuming MPU9250 registers)
    int16_t ax = (raw[0] << 8) | raw[1];
    int16_t ay = (raw[2] << 8) | raw[3];
    int16_t az = (raw[4] << 8) | raw[5];
    int16_t gx = (raw[8] << 8) | raw[9];
    int16_t gy = (raw[10] << 8) | raw[11];
    int16_t gz = (raw[12] << 8) | raw[13];

    // MPU9250 scales: accel 16g range = 2048 LSB/g, gyro 2000 dps = 16.4 LSB/°/s
    data->accel_x = ((float)ax) / 2048.0f;
    data->accel_y = ((float)ay) / 2048.0f;
    data->accel_z = ((float)az) / 2048.0f;
    data->gyro_x = ((float)gx) / 16.4f;
    data->gyro_y = ((float)gy) / 16.4f;
    data->gyro_z = ((float)gz) / 16.4f;

    // Magnetometer omitted for brevity, but can be added similarly

    data->mag_x = 0.0f;
    data->mag_y = 0.0f;
    data->mag_z = 0.0f;

    return true;
}

bool Sensors_ReadBarometer(Barometer_Data_t *data) {
    uint8_t raw[6];
    if (!ReadBarometerRaw(raw, sizeof(raw))) return false;

    // Simple BMP280 pressure and temperature conversion (approximate)
    uint32_t pres_raw = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);
    uint32_t temp_raw = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);

    // Apply calibration here (omitted for brevity, use library or datasheet)
    // Placeholder conversions:
    data->pressure = (float)pres_raw / 256.0f;
    data->temperature = (float)temp_raw / 512.0f;

    // Calculate altitude from pressure (standard atmosphere)
    data->altitude = 44330.0f * (1.0f - powf(data->pressure / 1013.25f, 0.1903f));

    return true;
}

bool Sensors_ReadPlasmaSensor(PlasmaSensor_Data_t *data) {
    uint16_t adc_value = 0;
    if (!ReadPlasmaRaw(&adc_value)) return false;

    // Convert ADC value to temperature and ionization levels (calibration needed)
    data->temperature = ((float)adc_value / 4095.0f) * 100.0f; // dummy scale
    data->ionization = (float)adc_value;

    return true;
}

// Internal implementations
static bool IMU_Init(void) {
    // Example: write to IMU registers to wake up device and configure
    uint8_t data[2];
    data[0] = 0x6B; // PWR_MGMT_1 register
    data[1] = 0x00; // Clear sleep bit
    if (HAL_I2C_Master_Transmit(&hi2c1, IMU_I2C_ADDR, data, 2, 100) != HAL_OK) return false;

    // Additional configuration omitted for brevity

    return true;
}

static bool Barometer_Init(void) {
    // Example: write config to barometer registers
    uint8_t data[2];
    data[0] = 0xF4; // CTRL_MEAS register
    data[1] = 0x27; // Temp and pressure oversampling x1, normal mode
    if (HAL_I2C_Master_Transmit(&hi2c1, BARO_I2C_ADDR, data, 2, 100) != HAL_OK) return false;

    return true;
}

static bool ReadIMURaw(uint8_t *buffer, size_t length) {
    if (HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, buffer, length, 100) != HAL_OK) {
        return false;
    }
    return true;
}

static bool ReadBarometerRaw(uint8_t *buffer, size_t length) {
    if (HAL_I2C_Mem_Read(&hi2c1, BARO_I2C_ADDR, 0xF7, I2C_MEMADD_SIZE_8BIT, buffer, length, 100) != HAL_OK) {
        return false;
    }
    return true;
}

static bool ReadPlasmaRaw(uint16_t *adc_value) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = PLASMA_ADC_CHANNEL;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) return false;

    if (HAL_ADC_Start(&hadc1) != HAL_OK) return false;
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) return false;

    *adc_value = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);
    return true;
}
