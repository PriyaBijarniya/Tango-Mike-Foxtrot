/*
 * main.cpp - Entry point for TMF drone firmware
 * MCU: STM32F746ZG
 * Author: BryceWDesign
 * License: Apache-2.0
 *
 * Initializes all subsystems and runs the real-time flight control loop.
 * This version assumes test mode inputs are simulated.
 */

#include "flight_control.h"
#include "power_monitor.h"
#include "propulsion_driver.h"
#include "sensors.h"
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>

#define LOOP_INTERVAL_MS 10

int main() {
    printf("Initializing TMF Drone Firmware...\n");

    if (!PowerMonitor_Init()) {
        printf("Power monitor initialization failed.\n");
        return -1;
    }

    if (!Sensors_Init()) {
        printf("Sensor initialization failed.\n");
        return -1;
    }

    if (!FlightControl_Init()) {
        printf("Flight control initialization failed.\n");
        return -1;
    }

    if (!PropulsionDriver_Init()) {
        printf("Propulsion driver initialization failed.\n");
        return -1;
    }

    printf("Initialization complete. Entering control loop...\n");

    // Dummy command: can later be replaced by RC input, AI pilot, or BCI interface
    Flight_Command_t command = {
        .roll = 0.0f,
        .pitch = 0.0f,
        .yaw = 0.0f,
        .throttle = 0.6f
    };

    while (true) {
        float current_roll = 0.0f;
        float current_pitch = 0.0f;
        float current_yaw = 0.0f;

        if (!Sensors_ReadIMU(&current_roll, &current_pitch, &current_yaw)) {
            printf("Sensor read error. Skipping frame.\n");
            continue;
        }

        Motor_Output_t motors;
        FlightControl_Update(&command, current_roll, current_pitch, current_yaw, &motors);
        PropulsionDriver_SetOutputs(&motors);

        PowerMonitor_CheckHealth(); // Optional: alert/log on issues

        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_INTERVAL_MS));
    }

    return 0;
}
