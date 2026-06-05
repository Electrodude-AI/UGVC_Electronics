#include "pid.h"
#include "config.h"

float integral[4]     = {0, 0, 0, 0};
float prevActual[4]   = {0, 0, 0, 0};
unsigned long prevPidTime[4] = {0, 0, 0, 0};
bool initialized[4]   = {false, false, false, false};

float calculatePID(
    float target,
    float actual,
    int motor)
{
    float error = target - actual;

    // DT CALCULATION

    unsigned long now = micros();
    float dt = (now - prevPidTime[motor]) / 1000000.0f;

    if(dt < 0.001f)
        dt = 0.001f;

    if(dt > 0.1f)
        dt = 0.02f;

    prevPidTime[motor] = now;

    // DIRECTION REVERSAL INTEGRAL RESET & DECAY

    static float prevTarget[4] = {0, 0, 0, 0};

    if ((target > 0.0f && prevTarget[motor] <= 0.0f) ||
        (target < 0.0f && prevTarget[motor] >= 0.0f))
    {
        if (target != prevTarget[motor])
            integral[motor] = 0;
    }
    
    prevTarget[motor] = target;

    if(abs(target) < 5.0f)
    {
        integral[motor] *= 0.90f;
    }

    // DERIVATIVE ON MEASUREMENT
    //
    // Suppress derivative on first call (prevActual not yet valid).

    float derivative = 0;

    if(!initialized[motor])
    {
        initialized[motor] = true;
        prevActual[motor] = actual;
    }
    else
    {
        derivative =
            -(actual - prevActual[motor]) / dt;
    }

    prevActual[motor] = actual;

    float pTerm = Kp * error;
    float iTerm = Ki * integral[motor];
    float dTerm = Kd * derivative;
    float ff    = (target != 0.0f) ? target * kF : 0.0f;

    // Two-stage saturation check:
    //
    //   Stage 1: base = P + D + FF (no integral contribution)
    //     → If base alone saturates, freeze integral at current value.
    //       No windup occurred (I never entered the computation).
    //
    //   Stage 2: base + I saturates but base alone did not
    //     → I pushed us into saturation. Back-calculate the excess.
    //
    //   Neither → normal integration.

    float base   = pTerm + dTerm + ff;
    float output = 0.0f;

    if(base > 255.0f || base < -255.0f)
    {
        output = constrain(base, -255.0f, 255.0f);
    }
    else
    {
        output = base + iTerm;

        if(output > 255.0f || output < -255.0f)
        {
            // I pushed us into saturation: back-calculate
            float excess = output - constrain(output, -255.0f, 255.0f);
            if(error * excess > 0.0f)
                integral[motor] -= excess * ANTI_WINDUP_GAIN * dt;
            output = constrain(output, -255.0f, 255.0f);
        }
        else
        {
            // Not saturated: normal integration
            integral[motor] += error * dt;
        }
    }

    integral[motor] =
        constrain(integral[motor],
                  -INTEGRAL_LIMIT,
                   INTEGRAL_LIMIT);

    return output;
}

MotorCommands pidComputeAll(
    const MotorTargets& targets,
    const WheelSpeeds&  speeds,
    const bool*         faultMask)
{
    MotorCommands cmds = {0, 0, 0, 0};

#ifdef SAFE_MODE

    (void)speeds;
    (void)faultMask;

    cmds.fl = constrain((targets.fl / (float)SAFE_MAX_TARGET) * SAFE_PWM_LIMIT,
                        -SAFE_PWM_LIMIT, SAFE_PWM_LIMIT);
    cmds.rl = constrain((targets.rl / (float)SAFE_MAX_TARGET) * SAFE_PWM_LIMIT,
                        -SAFE_PWM_LIMIT, SAFE_PWM_LIMIT);
    cmds.fr = constrain((targets.fr / (float)SAFE_MAX_TARGET) * SAFE_PWM_LIMIT,
                        -SAFE_PWM_LIMIT, SAFE_PWM_LIMIT);
    cmds.rr = constrain((targets.rr / (float)SAFE_MAX_TARGET) * SAFE_PWM_LIMIT,
                        -SAFE_PWM_LIMIT, SAFE_PWM_LIMIT);

#else

    if(!faultMask[0])
    {
        float target = targets.fl;
        target = constrain(target, -MAX_CPS, MAX_CPS);
        cmds.fl = calculatePID(target, speeds.fl, 0);
    }

    if(!faultMask[1])
    {
        float target = targets.rl;
        target = constrain(target, -MAX_CPS, MAX_CPS);
        cmds.rl = calculatePID(target, speeds.rl, 1);
    }

    if(!faultMask[2])
    {
        float target = targets.fr;
        target = constrain(target, -MAX_CPS, MAX_CPS);
        cmds.fr = calculatePID(target, speeds.fr, 2);
    }

    if(!faultMask[3])
    {
        float target = targets.rr;
        target = constrain(target, -MAX_CPS, MAX_CPS);
        cmds.rr = calculatePID(target, speeds.rr, 3);
    }

#endif

    return cmds;
}

void resetMotorPID(int motor)
{
    if(motor >= 0 && motor < 4)
    {
        integral[motor] = 0;
        prevActual[motor] = 0;
        prevPidTime[motor] = 0;
        initialized[motor] = false;
    }
}

void resetPID()
{
    for(int i = 0; i < 4; i++)
    {
        resetMotorPID(i);
    }
}
