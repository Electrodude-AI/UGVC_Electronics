#include "slip_detector.h"
#include "config.h"

#define COUNTS_PER_RPM (CPR / 60.0f)   // 79.73 cps per RPM (was 100 with 6000 CPR)
#define MOTOR_MAX_RPM  200.0f
#define MOTOR_TAU      0.05f

static float  filtered[4]        = {0, 0, 0, 0};
static int    confirmCount[4]    = {0, 0, 0, 0};
static int    clearCount[4]      = {0, 0, 0, 0};
static bool   slipping[4]        = {false, false, false, false};
static float  prevSpeeds[4]      = {0, 0, 0, 0};
static unsigned long lastTargetChange = 0;
static unsigned long lastRev[4]        = {0, 0, 0, 0};
static float  prevTargets[4]     = {0, 0, 0, 0};
static bool   initialized        = false;

void slipDetectUpdate(
    const MotorTargets&  targets,
    const WheelSpeeds&   speeds,
    const MotorCommands& cmds,
    unsigned long        nowMs,
    float                dt)
{
    const float* t = &targets.fl;
    const float* s = &speeds.fl;
    const float* c = &cmds.fl;

    // Track target changes for ramp suppression
    for(int i = 0; i < 4; i++)
    {
        if(fabs(t[i] - prevTargets[i]) > 1.0f)
            lastTargetChange = nowMs;

        if((t[i] > 0 && prevTargets[i] < 0) ||
           (t[i] < 0 && prevTargets[i] > 0))
            lastRev[i] = nowMs;

        prevTargets[i] = t[i];
    }

    // Side averages
    float avgLeft  = (s[0] + s[1]) * 0.5f;
    float avgRight = (s[2] + s[3]) * 0.5f;

    // Surface-adaptive threshold
    float maxT = max(max(fabs(t[0]), fabs(t[1])),
                     max(fabs(t[2]), fabs(t[3])));
    float threshold = (maxT < 300.0f) ? SLIP_THRESH_LIGHT : SLIP_THRESH_MOD;

    bool inRamp = (nowMs - lastTargetChange < SLIP_RAMP_SUPPRESS);
    bool startup = (nowMs < 1000);

    for(int i = 0; i < 4; i++)
    {
        bool inRev = (nowMs - lastRev[i] < SLIP_REV_SUPPRESS);

        // Fix #1: Steady-state-only slip evaluation to prevent false triggers during transients
        if (inRamp || inRev || startup || fabs(t[i]) < 50.0f)
        {
            filtered[i] = 0;
            confirmCount[i] = 0;
            clearCount[i] = 0;
            slipping[i] = false;
            prevSpeeds[i] = s[i];
            continue;
        }

        // Skip accel error on first frame (prevSpeeds not yet valid)
        if(!initialized)
            prevSpeeds[i] = s[i];

        // Divergence from side-mate average
        float sideAvg = (i < 2) ? avgLeft : avgRight;
        float div = (fabs(sideAvg) > 100.0f)
            ? fabs(s[i] - sideAvg) / fabs(sideAvg)
            : 0.0f;

        // Acceleration error: how well actual accel matches motor model
        float accel = (s[i] - prevSpeeds[i]) / max(dt, 0.001f);
        float pwmNorm = fabs(c[i]) / 255.0f;
        float speedRPM = s[i] / COUNTS_PER_RPM;
        float expAccelRPM = (pwmNorm * MOTOR_MAX_RPM - fabs(speedRPM)) / MOTOR_TAU;
        float expAccelCounts = expAccelRPM * COUNTS_PER_RPM;
        float accelErr = fabs(accel - expAccelCounts);
        accelErr = constrain(accelErr / 5000.0f, 0.0f, 1.0f);

        // Kinematic consistency (when near-straight)
        float kin = 0.0f;
        bool nearStraight = (fabs(t[0] - t[2]) < 100.0f &&
                             fabs(t[1] - t[3]) < 100.0f);
        if(nearStraight && avgLeft > 500.0f && avgRight > 500.0f)
        {
            float ratio = (avgLeft + avgRight) / max(avgLeft, avgRight);
            kin = 1.0f - constrain(ratio, 0.0f, 1.0f);
        }

        // Composite slip index
        float raw = div   * SLIP_WEIGHT_DIV
                  + accelErr * SLIP_WEIGHT_ACCEL
                  + kin     * SLIP_WEIGHT_KIN;

        // EMA filter
        filtered[i] = SLIP_ALPHA * raw + (1.0f - SLIP_ALPHA) * filtered[i];

        // State machine
        bool pwmActive = fabs(c[i]) > SLIP_PWM_MIN;

        if(pwmActive && filtered[i] > threshold)
        {
            confirmCount[i]++;
            clearCount[i] = 0;
            if(confirmCount[i] * (int)(dt * 1000) >= SLIP_CONFIRM_MS)
                slipping[i] = true;
        }
        else
        {
            confirmCount[i] = 0;
            if(slipping[i])
            {
                clearCount[i]++;
                if(clearCount[i] * (int)(dt * 1000) >= SLIP_CLEAR_MS)
                    slipping[i] = false;
            }
        }

        prevSpeeds[i] = s[i];
    }

    initialized = true;
}

void slipApplyResponse(MotorCommands& cmds)
{
    float* c = &cmds.fl;
    for(int i = 0; i < 4; i++)
    {
        if(slipping[i])
            c[i] *= SLIP_PWM_CAP;
    }
}

bool slipIsSlipping(int wheel)
{
    if(wheel < 0 || wheel > 3) return false;
    return slipping[wheel];
}

bool slipAnySlipping()
{
    return slipping[0] || slipping[1] ||
           slipping[2] || slipping[3];
}

float slipGetIndex(int wheel)
{
    if(wheel < 0 || wheel > 3) return 0.0f;
    return filtered[wheel];
}

void slipReset()
{
    for(int i = 0; i < 4; i++)
    {
        filtered[i]     = 0;
        confirmCount[i] = 0;
        clearCount[i]   = 0;
        slipping[i]     = false;
        prevSpeeds[i]   = 0;
        prevTargets[i]  = 0;
        lastRev[i]      = 0;
    }
    lastTargetChange = 0;
    initialized = false;
}
