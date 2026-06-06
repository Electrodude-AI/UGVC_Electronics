#include <Arduino.h>

#include "config.h"
#include "receiver.h"
#include "encoder_manager.h"
#include "pid.h"
#include "motor_driver.h"
#include "watchdog.h"
#include "safety_monitor.h"
#include "slip_detector.h"

static unsigned long lastLoop = 0;

#ifdef ENABLE_TELEMETRY
static unsigned long lastTelemetry = 0;
static unsigned long loopCount = 0;
#endif

void setup()
{
    Serial.begin(115200);

    Serial.print("MAX_CPS COMPUTED AS: ");
    Serial.println(MAX_CPS);
    if (abs(MAX_CPS - 12481.9f) > 2.0f) {
        Serial.println("HALT: MAX_CPS does not match 9 km/h specification!");
        while(1);
    }

    receiverInit();
    encoderInit();
    motorInit();
    watchdogInit();
    safetyInit();
}

void loop()
{
    watchdogCheck();

    unsigned long now = millis();

    if(now - lastLoop < loopTime)
        return;

    lastLoop = now;

#ifdef MOTOR_TEST_MODE

    if(motorTestLoop())
    {
        Serial.println("MOTOR TEST COMPLETE");
        while(1);
    }

    watchdogFeed();
    return;

#endif

    if(!receiverUpdate())
    {
        stopAllMotors();
        resetPID();
        safetyReset();
        slipReset();
        watchdogFeed();
        return;
    }

    MotorTargets targets = getReceiverTargets();
    WheelSpeeds  speeds  = getWheelSpeeds();

    safetyUpdate(targets, speeds);

    if(safetyCriticalFault())
    {
        stopAllMotors();
        while(1);
    }

    MotorCommands cmds = pidComputeAll(targets, speeds, safetyFaultMask());

    slipDetectUpdate(targets, speeds, cmds, now, loopTime / 1000.0f);
    slipApplyResponse(cmds);

    motorApply(cmds);

#ifdef SAFE_MODE
    static unsigned long lastPrint = 0;
    if (now - lastPrint > 100) // Print 10 times a second
    {
        Serial.print("Receiver -> L: ");
        Serial.print(targets.fl);
        Serial.print("  R: ");
        Serial.print(targets.fr);
        Serial.print("  | PWM -> L: ");
        Serial.print(cmds.fl);
        Serial.print("  R: ");
        Serial.println(cmds.fr);
        lastPrint = now;
    }
#endif

    watchdogFeed();

#ifdef ENABLE_TELEMETRY
    loopCount++;

    if(now - lastTelemetry >= 200)
    {
        unsigned long elapsed = now - lastTelemetry;
        float freq = (float)loopCount / (elapsed / 1000.0f);

        Serial.print("LOOP=");
        Serial.print(freq, 1);
        Serial.print("Hz FL=");
        Serial.print(speeds.fl, 1);
        Serial.print(" RL=");
        Serial.print(speeds.rl, 1);
        Serial.print(" FR=");
        Serial.print(speeds.fr, 1);
        Serial.print(" RR=");
        Serial.print(speeds.rr, 1);
        Serial.print(" SLIP=");
        Serial.print(slipIsSlipping(0)); Serial.print(slipIsSlipping(1));
        Serial.print(slipIsSlipping(2)); Serial.println(slipIsSlipping(3));

        loopCount = 0;
        lastTelemetry = now;
    }
#endif
}
