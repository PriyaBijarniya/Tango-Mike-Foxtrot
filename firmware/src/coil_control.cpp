/*
 * coil_control.cpp - Plasma coil control implementation for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Controls high-frequency PWM or DAC output for plasma coil thruster drive.
 * Uses hardware timers and DAC channels configured for high-frequency signals.
 */

#include "coil_control.h"
#include "stm32f7xx_hal.h"
#include <math.h>

// Example hardware definitions (adjust to actual MCU pins and peripherals)
#define COIL_PWM_TIMER          htim2
#define COIL_PWM_CHANNEL        TIM_CHANNEL_1

extern TIM_HandleTypeDef COIL_PWM_TIMER;

// For amplitude control, using DAC channel (if available)
#define COIL_DAC_CHANNEL        DAC_CHANNEL_1
extern DAC_HandleTypeDef hdac;

// Internal state tracking
static uint32_t current_frequency = 0;
static float current_amplitude = 0.0f;
static bool coil_active = false;

// Helper: Calculate timer period for given frequency
static uint32_t FrequencyToTimerPeriod(uint32_t frequency) {
    // Assuming timer clock 100MHz (adjust as per clock config)
    // Timer period = TimerClock / frequency - 1
    if (frequency == 0) return 0xFFFFFFFF;
    return (100000000 / frequency) - 1;
}

void CoilControl_Init(void) {
    // Initialize PWM timer and DAC for coil control
    HAL_TIM_PWM_Start(&COIL_PWM_TIMER, COIL_PWM_CHANNEL);
    HAL_DAC_Start(&hdac, COIL_DAC_CHANNEL);
    coil_active = false;
}

bool CoilControl_SetFrequency(uint32_t frequency_hz) {
    if (frequency_hz < 10000 || frequency_hz > 1000000) {
        // Out of range
        return false;
    }

    uint32_t period = FrequencyToTimerPeriod(frequency_hz);
    if (period == 0xFFFFFFFF) return false;

    // Update timer period register to adjust PWM frequency
    COIL_PWM_TIMER.Instance->ARR = period;

    // Reset timer counter to avoid glitches
    COIL_PWM_TIMER.Instance->CNT = 0;

    current_frequency = frequency_hz;
    return true;
}

void CoilControl_SetAmplitude(float amplitude) {
    // Clamp amplitude between 0 and 1
    if (amplitude < 0.0f) amplitude = 0.0f;
    if (amplitude > 1.0f) amplitude = 1.0f;

    current_amplitude = amplitude;

    // Convert amplitude to DAC output value (12-bit resolution assumed)
    uint32_t dac_value = (uint32_t)(amplitude * 4095);

    // Set DAC output to modulate coil power
    HAL_DAC_SetValue(&hdac, COIL_DAC_CHANNEL, DAC_ALIGN_12B_R, dac_value);

    // Adjust PWM duty cycle proportional to amplitude
    uint32_t pulse = (uint32_t)(COIL_PWM_TIMER.Instance->ARR * amplitude);
    __HAL_TIM_SET_COMPARE(&COIL_PWM_TIMER, COIL_PWM_CHANNEL, pulse);
}

void CoilControl_Enable(void) {
    HAL_TIM_PWM_Start(&COIL_PWM_TIMER, COIL_PWM_CHANNEL);
    HAL_DAC_Start(&hdac, COIL_DAC_CHANNEL);
    coil_active = true;
}

void CoilControl_Disable(void) {
    HAL_TIM_PWM_Stop(&COIL_PWM_TIMER, COIL_PWM_CHANNEL);
    HAL_DAC_Stop(&hdac, COIL_DAC_CHANNEL);
    coil_active = false;
}

bool CoilControl_IsActive(void) {
    return coil_active;
}
