#pragma once
#include <Arduino.h>

namespace config {
    constexpr uint32_t CONTROL_LOOP_FREQ_HZ = 100;
    constexpr uint32_t LOOP_PERIOD_MICROS   = 1000000 / CONTROL_LOOP_FREQ_HZ;
    constexpr uint32_t SERIAL_BAUD_RATE     = 115200;

    // --- MDDS30 #1 (LEFT SIDE) ---
    constexpr uint8_t PIN_M_FL_PWM = 4; // Front-Left
    constexpr uint8_t PIN_M_FL_DIR = 6;
    constexpr uint8_t PIN_M_RL_PWM = 5; // Rear-Left
    constexpr uint8_t PIN_M_RL_DIR = 7;

    // --- MDDS30 #2 (RIGHT SIDE) ---
    constexpr uint8_t PIN_M_FR_PWM = 8;  // Front-Right
    constexpr uint8_t PIN_M_FR_DIR = 10;
    constexpr uint8_t PIN_M_RR_PWM = 9;  // Rear-Right
    constexpr uint8_t PIN_M_RR_DIR = 11;

    // --- 4 INDEPENDENT ENCODERS ---
    constexpr uint8_t PIN_E_FL_A = 12;
    constexpr uint8_t PIN_E_FL_B = 13;
    constexpr uint8_t PIN_E_FR_A = 14;
    constexpr uint8_t PIN_E_FR_B = 15;
    constexpr uint8_t PIN_E_RL_A = 22; 
    constexpr uint8_t PIN_E_RL_B = 23;
    constexpr uint8_t PIN_E_RR_A = 24; 
    constexpr uint8_t PIN_E_RR_B = 25;
    constexpr uint32_t ENCODER_CPR = 4096;
    
    constexpr bool INVERT_ENC_FL = false;
    constexpr bool INVERT_ENC_FR = true;
    constexpr bool INVERT_ENC_RL = false;
    constexpr bool INVERT_ENC_RR = true;

    // --- TUNING PARAMETERS ---
    constexpr float MOTOR_DEADBAND = 0.05f;
    // SOFT START: Max change in throttle per second.
    constexpr float MOTOR_SLEW_RATE = 2.0f; 

    constexpr float MAX_MOTOR_RPM = 300.0f;

    // Kp: 1.0 full effort / 300 RPM max error. Yields 100% effort only when error is at absolute max.
    constexpr float PID_KP = 1.0f / MAX_MOTOR_RPM; // ~0.00333f
    
    // Ki: Integral builds over time. Using ~10% of Kp means an error of 100 RPM held for 1 second adds ~0.033 to the effort.
    constexpr float PID_KI = PID_KP * 0.1f;
    
    // Kd: Dampens aggressive changes. Using ~1% of Kp as a baseline for derivation over the 10ms dt loop.
    constexpr float PID_KD = PID_KP * 0.01f;
    
    constexpr float PID_MAX_INT = 0.5f;

    // --- ADVANCED FEATURES PARAMETERS ---
    constexpr uint32_t WATCHDOG_TIMEOUT_MICROS = 500000; // 500ms Watchdog
    constexpr float ENCODER_HEALTH_TIMEOUT_SEC = 0.5f;   // 500ms Encoder Dead Time
    constexpr float ENCODER_MIN_RPM_THRESH = 1.0f;       // Below 1 RPM is considered 0
    constexpr float MOTOR_MIN_EFFORT_THRESH = 0.20f;     // Commanded effort > 20%
    constexpr float SLIP_RPM_THRESH = 5.0f;              // RPM diff for slip detection
    constexpr float CMD_LPF_ALPHA = 0.1f;                // Alpha for Target Damping LPF
    
    // --- BATTERY & FEEDFORWARD ---
    constexpr uint8_t PIN_BATTERY_SENSE = A12;
    constexpr float NOMINAL_VOLTAGE = 24.0f;
    constexpr float VOLTAGE_DIVIDER_RATIO = 11.0f;       // Assuming a 100k / 10k divider
    constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
    constexpr float ADC_MAX_VALUE = 1023.0f;
    
    constexpr uint8_t LED = 17;
}