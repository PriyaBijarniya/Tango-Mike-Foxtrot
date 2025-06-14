/*
 * coil_control.cpp - Plasma Coil Driver and PWM Control for TMF Drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Controls high-current plasma coils generating the Darkstar plasma shell layer.
 * Implements DAC and PWM control with safety interlocks and thermal management.
 */

#include "coil_control.h"
#include "stm32f7xx_hal.h"
#include "power_management.h"
#include "safety.h"
#include <math.h>

// Hardware peripheral handles (externally declared)
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;  // PWM timer for coil modulation

// Constants
#define COIL_MAX_CURRENT_mA 5000   // Max safe coil current in milliamps
#define COIL_PWM_FREQ_HZ 20000     // PWM frequency 20 kHz
#define COIL_VOLTAGE_MAX 24.0f     // Max coil voltage supply in volts

// Internal state
static float currentSetpoint_mA = 0.0f;
static float currentMeasured_mA = 0.0f;
static float voltageMeasured_V = 0.0f;
static float temperature_C = 0.0f;

// Coil PWM duty cycle [0.0 - 1.0]
static float pwmDutyCycle = 0.0f;

// Forward declarations
static void UpdatePWM(float duty);
static float ReadCurrent_mA(void);
static float ReadVoltage_V(void);
static void ThermalProtectionCheck(void);

void CoilControl_Init(void)
{
    // Initialize DAC for coil current setpoint
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    // Configure TIM1 PWM on coil control pin
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    currentSetpoint_mA = 0.0f;
    pwmDutyCycle = 0.0f;
}

void CoilControlTask(void *parameters)
{
    (void)parameters;

    while (1)
    {
        // Read coil current and voltage sensors
        currentMeasured_mA = ReadCurrent_mA();
        voltageMeasured_V = ReadVoltage_V();

        // Update temperature from sensor (via Sensors module)
        temperature_C = GetCoilTemperature();

        // Thermal protection: if too hot, reduce coil power or shut down
        ThermalProtectionCheck();

        // Simple current control loop (P controller)
        float error_mA = currentSetpoint_mA - currentMeasured_mA;
        float kp = 0.001f; // Proportional gain (tuned experimentally)
        pwmDutyCycle += kp * error_mA;

        // Clamp PWM duty cycle between 0 and 1
        if (pwmDutyCycle > 1.0f) pwmDutyCycle = 1.0f;
        if (pwmDutyCycle < 0.0f) pwmDutyCycle = 0.0f;

        // Update PWM output to coil
        UpdatePWM(pwmDutyCycle);

        // Safety checks: if current spikes above max, shut down coil power immediately
        if (currentMeasured_mA > COIL_MAX_CURRENT_mA)
        {
            pwmDutyCycle = 0.0f;
            UpdatePWM(pwmDutyCycle);
            SignalSafetyFault("Coil overcurrent");
        }

        // Control loop interval ~1 ms
        vTaskDelay(1);
    }
}

void SetCoilCurrentSetpoint(float milliamps)
{
    if (milliamps < 0) milliamps = 0;
    if (milliamps > COIL_MAX_CURRENT_mA) milliamps = COIL_MAX_CURRENT_mA;
    currentSetpoint_mA = milliamps;
}

float GetCoilCurrentMeasured(void)
{
    return currentMeasured_mA;
}

float GetCoilVoltageMeasured(void)
{
    return voltageMeasured_V;
}

static void UpdatePWM(float duty)
{
    // Convert duty cycle to timer compare value
    uint32_t period = htim1.Init.Period + 1;
    uint32_t pulse = (uint32_t)(duty * (float)(period - 1));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

static float ReadCurrent_mA(void)
{
    // Read ADC channel connected to current sensor
    uint32_t adc_val = HAL_ADC_GetValue(&hadc1); // assumed ADC handle
    // Convert ADC reading to current (mA) using calibration curve
    // For example, 12-bit ADC (0-4095), 3.3V ref, sensor sensitivity 100mV/A:
    // current (A) = ((adc_val / 4095.0)*3.3 - offset_voltage)/sensitivity
    // Convert to mA
    float voltage = ((float)adc_val / 4095.0f) * 3.3f;
    float current_A = (voltage - 0.5f) / 0.1f; // Example offset 0.5V, sensitivity 0.1V/A
    return current_A * 1000.0f; // mA
}

static float ReadVoltage_V(void)
{
    // Read ADC channel for coil voltage divider
    uint32_t adc_val = HAL_ADC_GetValue(&hadc2); // assumed second ADC handle
    // Convert ADC to voltage using voltage divider factor
    float voltage = ((float)adc_val / 4095.0f) * 3.3f * (11.0f); // 10k/1k divider example
    return voltage;
}

static void ThermalProtectionCheck(void)
{
    // If coil temperature exceeds threshold, reduce or cut coil power
    const float maxTempC = 80.0f;
    if (temperature_C > maxTempC)
    {
        pwmDutyCycle *= 0.8f; // ramp down
        if (temperature_C > 90.0f)
        {
            pwmDutyCycle = 0.0f;
            UpdatePWM(pwmDutyCycle);
            SignalSafetyFault("Coil overtemperature shutdown");
        }
    }
}

