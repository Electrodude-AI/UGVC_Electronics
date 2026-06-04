#include <Arduino.h>
#include "config.h"
#include "SerialComm.h"
#include "WheelEncoder.h"
#include "Motor.h"
#include "PID.h"

// --- INSTANTIATE SERIAL PROTOCOL ---
SerialComm jetsonComm;

// --- INSTANTIATE 4 INDEPENDENT WHEELS ---
WheelEncoder  encFL(config::PIN_E_FL_A, config::PIN_E_FL_B, config::ENCODER_CPR, config::INVERT_ENC_FL);
MotorDriver   motFL(config::PIN_M_FL_PWM, config::PIN_M_FL_DIR, config::MOTOR_DEADBAND);
PIDController pidFL(config::PID_KP, config::PID_KI, config::PID_KD);

WheelEncoder  encFR(config::PIN_E_FR_A, config::PIN_E_FR_B, config::ENCODER_CPR, config::INVERT_ENC_FR);
MotorDriver   motFR(config::PIN_M_FR_PWM, config::PIN_M_FR_DIR, config::MOTOR_DEADBAND);
PIDController pidFR(config::PID_KP, config::PID_KI, config::PID_KD);

WheelEncoder  encRL(config::PIN_E_RL_A, config::PIN_E_RL_B, config::ENCODER_CPR, config::INVERT_ENC_RL);
MotorDriver   motRL(config::PIN_M_RL_PWM, config::PIN_M_RL_DIR, config::MOTOR_DEADBAND);
PIDController pidRL(config::PID_KP, config::PID_KI, config::PID_KD);

WheelEncoder  encRR(config::PIN_E_RR_A, config::PIN_E_RR_B, config::ENCODER_CPR, config::INVERT_ENC_RR);
MotorDriver   motRR(config::PIN_M_RR_PWM, config::PIN_M_RR_DIR, config::MOTOR_DEADBAND);
PIDController pidRR(config::PID_KP, config::PID_KI, config::PID_KD);

void setup() {
    pinMode(config::LED, OUTPUT);
    digitalWrite(config::LED, HIGH);

    analogReadResolution(10);

    jetsonComm.begin(config::SERIAL_BAUD_RATE);

    motFL.begin(); motFR.begin(); motRL.begin(); motRR.begin();
    encFL.reset(); encFR.reset(); encRL.reset(); encRR.reset();

    pidFL.setLimits(-1.0f, 1.0f, config::PID_MAX_INT);
    pidFR.setLimits(-1.0f, 1.0f, config::PID_MAX_INT);
    pidRL.setLimits(-1.0f, 1.0f, config::PID_MAX_INT);
    pidRR.setLimits(-1.0f, 1.0f, config::PID_MAX_INT);
}

float getFilteredBatteryVoltage() {
    static float filteredVoltage = config::NOMINAL_VOLTAGE; // Initialize at nominal
    
    int rawAdc = analogRead(config::PIN_BATTERY_SENSE);
    float pinVoltage = (static_cast<float>(rawAdc) / config::ADC_MAX_VALUE) * config::ADC_REFERENCE_VOLTAGE;
    float actualVoltage = pinVoltage * config::VOLTAGE_DIVIDER_RATIO;
    
    // LPF to prevent noise spikes from violently oscillating motor speeds
    filteredVoltage += 0.01f * (actualVoltage - filteredVoltage);
    
    // Safety clamp to prevent div by zero or extreme multipliers if unplugged
    if (filteredVoltage < 10.0f) return 10.0f; 
    
    return filteredVoltage;
}

void loop() {
    static uint32_t lastLoopTime = micros();
    uint32_t currentTime = micros();

    // Static variables for LPF Command Damping
    static float filteredTargetFL = 0.0f;
    static float filteredTargetFR = 0.0f;
    static float filteredTargetRL = 0.0f;
    static float filteredTargetRR = 0.0f;
    const float alpha = 0.05f;

    // Continuously process incoming Jetson serial data
    jetsonComm.spin();

    // The 100Hz Gatekeeper
    if ((currentTime - lastLoopTime) >= config::LOOP_PERIOD_MICROS) {
        float dt = static_cast<float>(currentTime - lastLoopTime) / 1000000.0f;
        lastLoopTime = currentTime;

        // (We continue applying the Serial internal LPF to extract the latest raw string parses)
        jetsonComm.applyCommandDamping(dt);

        // 1. Pull 4 Independent Targets from Jetson
        float tFL = jetsonComm.getTargetFL();
        float tFR = jetsonComm.getTargetFR();
        float tRL = jetsonComm.getTargetRL();
        float tRR = jetsonComm.getTargetRR();

        // Apply explicit Low-Pass Filter directly in main loop
        filteredTargetFL += alpha * (tFL - filteredTargetFL);
        filteredTargetFR += alpha * (tFR - filteredTargetFR);
        filteredTargetRL += alpha * (tRL - filteredTargetRL);
        filteredTargetRR += alpha * (tRR - filteredTargetRR);

        // 2. Measure 4 Independent Realities
        float mFL = encFL.getRPM(dt);
        float mFR = encFR.getRPM(dt);
        float mRL = encRL.getRPM(dt);
        float mRR = encRR.getRPM(dt);

        // 3. Compute 4 Independent Errors using Filtered Targets
        float cmdFL = pidFL.compute(filteredTargetFL, mFL, dt);
        float cmdFR = pidFR.compute(filteredTargetFR, mFR, dt);
        float cmdRL = pidRL.compute(filteredTargetRL, mRL, dt);
        float cmdRR = pidRR.compute(filteredTargetRR, mRR, dt);

        // 4. Advanced Health Checks
        encFL.updateHealth(cmdFL, dt);
        encFR.updateHealth(cmdFR, dt);
        encRL.updateHealth(cmdRL, dt);
        encRR.updateHealth(cmdRR, dt);

        bool wdgOk = jetsonComm.isWatchdogOK();
        bool encOk = encFL.isHealthy() && encFR.isHealthy() && encRL.isHealthy() && encRR.isHealthy();

        // Failsafe condition
        if (!wdgOk || !encOk) {
            cmdFL = 0.0f; cmdFR = 0.0f; cmdRL = 0.0f; cmdRR = 0.0f;
            pidFL.reset(); pidFR.reset(); pidRL.reset(); pidRR.reset();
            
            // Activate hard dynamic braking on all Cytrons
            motFL.brake();
            motFR.brake();
            motRL.brake();
            motRR.brake();
        } else {
            // --- VOLTAGE FEEDFORWARD COMPENSATION ---
            float currentVoltage = getFilteredBatteryVoltage();
            float feedforwardRatio = config::NOMINAL_VOLTAGE / currentVoltage;
            
            cmdFL *= feedforwardRatio;
            cmdFR *= feedforwardRatio;
            cmdRL *= feedforwardRatio;
            cmdRR *= feedforwardRatio;

            // 6. Actuate 4 Motors (INSTANTLY, no slew rate here)
            motFL.setSpeed(cmdFL);
            motFR.setSpeed(cmdFR);
            motRL.setSpeed(cmdRL);
            motRR.setSpeed(cmdRR);
        }

        // 7. Publish Telemetry at 50Hz (every other loop)
        static int telemetryCounter = 0;
        telemetryCounter++;
        if (telemetryCounter >= 2) {
            jetsonComm.publishFeedback(filteredTargetFL, filteredTargetFR, filteredTargetRL, filteredTargetRR, mFL, mFR, mRL, mRR, wdgOk, encOk, slipLeft, slipRight);
            telemetryCounter = 0;
        }
    }
}