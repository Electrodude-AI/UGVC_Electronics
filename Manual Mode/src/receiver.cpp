#include "receiver.h"
#include "config.h"

// ---------------------------------------------------------------------------
// Interrupt-based PWM capture
//
// attachInterrupt(CHANGE) fires on every rising and falling edge.
// Each channel tracks its last rising-edge timestamp and the resulting
// pulse width.  receiverUpdate() reads the latest captured values without
// blocking — no pulseIn, no busy-wait, no variable loop jitter.
// ---------------------------------------------------------------------------

struct PwmChannel
{
    volatile unsigned long riseTime;
    volatile unsigned long pulseWidth;
    volatile unsigned long lastEdge;
};

static PwmChannel chY = {0, 0, 0};
static PwmChannel chX = {0, 0, 0};

// digitalReadFast requires a compile-time constant pin number,
// so each channel gets its own ISR with the pin hardcoded.

static void isrChY()
{
    if(digitalReadFast(CH_Y))
    {
        chY.riseTime = micros();
    }
    else
    {
        chY.pulseWidth = micros() - chY.riseTime;
        chY.lastEdge   = micros();
    }
}

static void isrChX()
{
    if(digitalReadFast(CH_X))
    {
        chX.riseTime = micros();
    }
    else
    {
        chX.pulseWidth = micros() - chX.riseTime;
        chX.lastEdge   = micros();
    }
}

// ---------------------------------------------------------------------------
// Ramp
// ---------------------------------------------------------------------------

static MotorTargets targets;
static float currentV     = 0;
static float currentOmega = 0;

static float rampValue(float current, float target, int step)
{
    if(current < target)
    {
        current += step;

        if(current > target)
            current = target;
    }
    else if(current > target)
    {
        current -= step;

        if(current < target)
            current = target;
    }

    return current;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void receiverInit()
{
    pinMode(CH_X, INPUT);
    pinMode(CH_Y, INPUT);

    unsigned long now = micros();

    chY.riseTime  = now;
    chX.riseTime  = now;
    chY.lastEdge  = now;
    chX.lastEdge  = now;

    attachInterrupt(digitalPinToInterrupt(CH_Y), isrChY, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH_X), isrChX, CHANGE);
}

bool receiverUpdate()
{
    unsigned long now = micros();

    // Signal-loss detection: 100 ms without ANY edge on either channel

    if(now - chY.lastEdge > 100000)
        return false;

    if(now - chX.lastEdge > 100000)
        return false;

    // Read latest captured pulse widths (atomic 32-bit on Cortex-M7)

    int rawY = chY.pulseWidth;
    int rawX = chX.pulseWidth;

    if(rawY < 900 || rawY > 2100)
        return false;

    if(rawX < 900 || rawX > 2100)
        return false;

    // Lightweight receiver input smoothing (EMA filter)

    static float filtY = 1500.0f;
    static float filtX = 1500.0f;
    const float RC_ALPHA = 0.30f;

    filtY += RC_ALPHA * ((float)rawY - filtY);
    filtX += RC_ALPHA * ((float)rawX - filtX);

    int useY = (int)(filtY + 0.5f);
    int useX = (int)(filtX + 0.5f);

    // Map to target range

    float throttle =
        map(useY, 1000, 2000,
#ifdef SAFE_MODE
            -SAFE_MAX_TARGET, SAFE_MAX_TARGET
#else
            -(long)MAX_CPS, (long)MAX_CPS
#endif
        );

    float steering =
        map(useX, 1000, 2000,
#ifdef SAFE_MODE
            -SAFE_MAX_TARGET, SAFE_MAX_TARGET
#else
            -(long)MAX_CPS, (long)MAX_CPS
#endif
        );

    // Deadband

    if(abs(throttle) < deadband)
        throttle = 0;

    if(abs(steering) < deadband)
        steering = 0;

    // Prevent ramp windup beyond the speed limiter's active limit
    if (throttle > MAX_CPS) throttle = MAX_CPS;
    if (throttle < -MAX_CPS) throttle = -MAX_CPS;
    if (steering > MAX_CPS) steering = MAX_CPS;
    if (steering < -MAX_CPS) steering = -MAX_CPS;

    // Chassis-level skid-steer mixing with proportional normalization

    currentV     = rampValue(currentV,     throttle, rampStep);
    currentOmega = rampValue(currentOmega, steering, rampStepOmega);

    float left  = currentV + currentOmega * WHEELBASE_FACTOR;
    float right = currentV - currentOmega * WHEELBASE_FACTOR;

    float maxMag = fabs(left);
    if(fabs(right) > maxMag) maxMag = fabs(right);

    if(maxMag > MAX_CPS)
    {
        float scale = MAX_CPS / maxMag;
        left  *= scale;
        right *= scale;
    }

    targets.fl = left;
    targets.rl = left;

    targets.fr = right;
    targets.rr = right;

    return true;
}

MotorTargets getReceiverTargets()
{
    return targets;
}
