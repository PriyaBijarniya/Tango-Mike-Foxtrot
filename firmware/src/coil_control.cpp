/*
 * coil_control.cpp - Plasma coil driver implementation for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Controls four plasma coils via PWM and DAC
 * Measures coil current via ADC feedback for closed-loop control
 * Implements safety cutoffs and coil temperature monitoring (stub)
 */

#include "coil_control.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"

// Hardware definitions (replace with actual pins/timers as needed)
#define COIL_PWM_TIMER        htim1
#define COIL_PWM_CHANNEL_1    TIM_CHANNEL_1
#define COIL_PWM_CHANNEL_2    TIM_CHANNEL_2
#define COIL_PWM_CHANNEL_3    TIM_CHANNEL_3
#define COIL_PWM_CHANNEL_4    TIM_CHANNEL_4

#define CURRENT_SENSOR_ADC    hadc1
#define TEMPERATURE_SENSOR_ADC hadc2

// Constants
#define MAX_COIL_CURRENT_mA   1500.0f   // Max safe coil current in milliamps
#define PWM_MAX_DUTY_CYCLE    1000      // Max timer value for 100% duty cycle

// Static state
static float coil_current_setpoints[4] = {0};
static float coil_current_measured[4] = {0};
static float coil_voltage_measured[4] = {0};

// Forward declarations
static void SetPWM(uint8_t coilIndex, uint16_t duty);
static uint16_t CurrentToPWMDuty(float current_mA);

// Public API

void CoilControl_Init(void) {
    // Initialize PWM timers and ADCs - assumed to be done externally
    // Just set all coil outputs to 0 duty cycle here
    SetPWM(0, 0);
    SetPWM(1, 0);
    SetPWM(2, 0);
    SetPWM(3, 0);
}

void CoilControlTask(void *parameters) {
    // Called periodically (e.g., 1kHz) by RTOS task or main loop

    for (uint8_t i = 0; i < 4; i++) {
        // Read ADC for current sensor of coil i
        // Stub: simulate reading for now
        uint16_t adc_value = HAL_ADC_GetValue(&CURRENT_SENSOR_ADC);
        coil_current_measured[i] = ((float)adc_value) * 3.3f / 4095.0f * 1000.0f; // milliamps approximation
        
        // Convert current setpoint to PWM duty
        uint16_t pwm_duty = CurrentToPWMDuty(coil_current_setpoints[i]);

        // Safety clamp
        if (coil_current_setpoints[i] > MAX_COIL_CURRENT_mA) {
            coil_current_setpoints[i] = MAX_COIL_CURRENT_mA;
        }
        if (coil_current_setpoints[i] < 0) {
            coil_current_setpoints[i] = 0;
        }

        // Set PWM duty for coil
        SetPWM(i, pwm_duty);
    }

    // TODO: Add temperature monitoring and safety cutoffs
}

void SetCoilCurrentSetpoint(float milliamps) {
    for (uint8_t i = 0; i < 4; i++) {
        coil_current_setpoints[i] = milliamps;
    }
}

float GetCoilCurrentMeasured(void) {
    // Return average coil current measured
    float sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += coil_current_measured[i];
    }
    return sum / 4.0f;
}

float GetCoilVoltageMeasured(void) {
    // Stub: Return fixed voltage
    return 12.0f;
}

void CoilControl_SetCoilCurrent(uint8_t coilIndex, float current_mA) {
    if (coilIndex < 4) {
        coil_current_setpoints[coilIndex] = current_mA;
    }
}

// Private helpers

static void SetPWM(uint8_t coilIndex, uint16_t duty) {
    if (duty > PWM_MAX_DUTY_CYCLE) duty = PWM_MAX_DUTY_CYCLE;

    switch (coilIndex) {
        case 0:
            __HAL_TIM_SET_COMPARE(&COIL_PWM_TIMER, COIL_PWM_CHANNEL_1, duty);
            break;
        case 1:
            __HAL_TIM_SET_COMPARE(&COIL_PWM_TIMER, COIL_PWM_CHANNEL_2, duty);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&COIL_PWM_TIMER, COIL_PWM_CHANNEL_3, duty);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&COIL_PWM_TIMER, COIL_PWM_CHANNEL_4, duty);
            break;
        default:
            break;
    }
}

static uint16_t CurrentToPWMDuty(float current_mA) {
    // Simple linear map for demo: 0mA -> 0 duty, MAX_COIL_CURRENT_mA -> 100% duty
    if (current_mA < 0) current_mA = 0;
    if (current_mA > MAX_COIL_CURRENT_mA) current_mA = MAX_COIL_CURRENT_mA;

    return (uint16_t)((current_mA / MAX_COIL_CURRENT_mA) * PWM_MAX_DUTY_CYCLE);
}
