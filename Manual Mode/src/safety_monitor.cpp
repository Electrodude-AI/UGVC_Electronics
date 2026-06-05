#include "safety_monitor.h"
#include "config.h"
#include "motor_driver.h"
#include "pid.h"

#include <stdint.h>

enum FaultState : uint8_t
{
    HEALTH_NORMAL  = 0,
    HEALTH_SUSPECT = 1,
    HEALTH_FAULT   = 2
};

struct WheelHealth
{
    FaultState     state;
    unsigned long  timer;
    uint8_t        reason;
};

static WheelHealth health[4] = {
    {HEALTH_NORMAL, 0, 0}, {HEALTH_NORMAL, 0, 0},
    {HEALTH_NORMAL, 0, 0}, {HEALTH_NORMAL, 0, 0}
};

static const char* wheelLabels[4] = {"FL", "RL", "FR", "RR"};

void safetyInit()
{
}

void safetyUpdate(
    const MotorTargets& targets,
    const WheelSpeeds&  speeds)
{
#ifdef SAFE_MODE
    (void)targets;
    (void)speeds;
#else

    const float tArr[4] = {targets.fl, targets.rl, targets.fr, targets.rr};
    const float sArr[4] = {speeds.fl,  speeds.rl,  speeds.fr,  speeds.rr};

    for(int i = 0; i < 4; i++)
    {
        float st = tArr[i];
        float sp = sArr[i];

        bool runaway  = (fabs(st) < HEALTH_TARGET_NEAR_ZERO) && (fabs(sp) > HEALTH_SPEED_RUNAWAY);
        bool stuck    = (fabs(st) > HEALTH_TARGET_ACTIVE)    && (fabs(sp) < HEALTH_SPEED_STUCK);
        bool faultNow = runaway || stuck;

        switch(health[i].state)
        {
            case HEALTH_NORMAL:
                if(faultNow)
                {
                    health[i].state = HEALTH_SUSPECT;
                    health[i].timer = millis();
                }
                break;

            case HEALTH_SUSPECT:
                if(faultNow)
                {
                    if(millis() - health[i].timer >= HEALTH_SUSPECT_MS)
                    {
                        health[i].state  = HEALTH_FAULT;
                        health[i].reason = runaway ? 1 : 2;
                        motorSetEnabled(i, false);
                        resetMotorPID(i);
                        Serial.print("FAULT_");
                        Serial.print(wheelLabels[i]);
                        Serial.print(" ");
                        Serial.println(health[i].reason == 1 ? "RUNAWAY" : "STUCK");
                    }
                }
                else
                {
                    health[i].state = HEALTH_NORMAL;
                }
                break;

            case HEALTH_FAULT:
                if(faultNow)
                {
                    health[i].timer = millis();
                }
                else
                {
                    if(millis() - health[i].timer >= HEALTH_CLEAR_MS)
                    {
                        health[i].state  = HEALTH_NORMAL;
                        health[i].reason = 0;
                        motorSetEnabled(i, true);
                        Serial.print("OK_");
                        Serial.println(wheelLabels[i]);
                    }
                }
                break;
        }
    }

#endif // ndef SAFE_MODE
}

bool safetyCriticalFault()
{
    int count = 0;

    for(int i = 0; i < 4; i++)
        if(health[i].state == HEALTH_FAULT)
            count++;

    if(count >= 2)
    {
        Serial.println("CRITICAL: 2+ wheel faults — system halted");
        return true;
    }

    return false;
}

const bool* safetyFaultMask()
{
    static bool mask[4];

    for(int i = 0; i < 4; i++)
        mask[i] = (health[i].state == HEALTH_FAULT);

    return mask;
}

void safetyReset()
{
    for(int i = 0; i < 4; i++)
    {
        health[i].state  = HEALTH_NORMAL;
        health[i].timer  = 0;
        health[i].reason = 0;
        motorSetEnabled(i, true);
    }
}
