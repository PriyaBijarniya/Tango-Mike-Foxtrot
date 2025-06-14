/*
 * motor_control.cpp - Motor control implementation for TMF drone
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Controls ESC throttle and yaw servo using PWM output.
 * Assumes hardware timers configured for PWM signals.
 */

#include "motor_control.h"
#include "stm32f7xx_hal.h"

// Hardware-specific definitions for PWM channels and timers
// Adjust based on actual hardware timer and GPIO mapping

// Example PWM timers and channels for throttle and yaw
#define THROTTLE_PWM_TIMER        htim1
#define THROTTLE_PWM_CHANNEL      TIM_CHANNEL_1

#define YAW_PWM_TIMER             htim1
#define YAW_PWM_CHANNEL           TIM_CHANNEL_2

// External handles for timers (defined in main.c or hardware config)
extern TIM_HandleTypeDef THROTTLE_PWM_TIMER;
extern TIM_HandleTypeDef YAW_PWM_TIMER;

// ESC PWM pulse width limits (microseconds)
#define ESC_PWM_MIN 1000
#define ESC_PWM_MAX 2000

// Yaw servo PWM pulse width limits (microseconds)
#define YAW_PWM_MIN 1000
#define YAW_PWM_MAX 2000

// Converts microseconds to timer counts
static inline uint32_t PWM_us_to_counts(uint32_t microseconds) {
    // Assuming timer clock configured at 1MHz for 1us resolution
    return microseconds;
}

// Clamp utility function
static float clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

void MotorControl_Init(void) {
    // Initialize PWM timers, channels, GPIO pins for ESC and yaw servo control
    // HAL library assumed; hardware-specific setup must be done in main or HAL init
    HAL_TIM_PWM_Start(&THROTTLE_PWM_TIMER, THROTTLE_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&YAW_PWM_TIMER, YAW_PWM_CHANNEL);
}

void MotorControl_SetThrottle(float throttle_cmd) {
    // throttle_cmd expected range: -1000.0 to +1000.0 (normalized thrust command)
    // Map to ESC PWM pulse width 1000-2000us
    throttle_cmd = clamp(throttle_cmd, -1000.0f, 1000.0f);
    float normalized = (throttle_cmd + 1000.0f) / 2000.0f; // Map [-1000,1000] to [0,1]

    uint32_t pulse_width_us = (uint32_t)(ESC_PWM_MIN + normalized * (ESC_PWM_MAX - ESC_PWM_MIN));
    uint32_t compare_val = PWM_us_to_counts(pulse_width_us);

    __HAL_TIM_SET_COMPARE(&THROTTLE_PWM_TIMER, THROTTLE_PWM_CHANNEL, compare_val);
}

void MotorControl_SetYaw(float yaw_cmd) {
    // yaw_cmd expected range: -500.0 to +500.0 (degrees per second or normalized)
    // Map to servo PWM pulse width 1000-2000us centered at 1500us
    yaw_cmd = clamp(yaw_cmd, -500.0f, 500.0f);
    float normalized = (yaw_cmd + 500.0f) / 1000.0f; // Map [-500,500] to [0,1]

    uint32_t pulse_width_us = (uint32_t)(YAW_PWM_MIN + normalized * (YAW_PWM_MAX - YAW_PWM_MIN));
    uint32_t compare_val = PWM_us_to_counts(pulse_width_us);

    __HAL_TIM_SET_COMPARE(&YAW_PWM_TIMER, YAW_PWM_CHANNEL, compare_val);
}

void MotorControl_EmergencyStop(void) {
    // Immediately cut throttle to minimum
    __HAL_TIM_SET_COMPARE(&THROTTLE_PWM_TIMER, THROTTLE_PWM_CHANNEL, PWM_us_to_counts(ESC_PWM_MIN));
    // Center yaw servo to neutral
    __HAL_TIM_SET_COMPARE(&YAW_PWM_TIMER, YAW_PWM_CHANNEL, PWM_us_to_counts(1500));
}

void MotorControl_CalibrateESC(void) {
    // ESC calibration sequence (varies by ESC brand)
    // Typically max throttle, wait, then min throttle
    MotorControl_SetThrottle(1000.0f);
    HAL_Delay(3000); // 3 seconds max throttle
    MotorControl_SetThrottle(-1000.0f);
    HAL_Delay(3000); // 3 seconds min throttle
}

