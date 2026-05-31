#include "receiver.h"
#include "config.h"

static MotorTargets targets;

static float currentLeft = 0;
static float currentRight = 0;

void receiverInit()
{
    pinMode(CH_X, INPUT);
    pinMode(CH_Y, INPUT);
}

float rampValue(float current, float target)
{
    if(current < target)
    {
        current += rampStep;

        if(current > target)
            current = target;
    }
    else if(current > target)
    {
        current -= rampStep;

        if(current < target)
            current = target;
    }

    return current;
}

bool receiverUpdate()
{
    int chY = pulseIn(CH_Y,HIGH,25000);
    int chX = pulseIn(CH_X,HIGH,25000);

    if(chY < 900 || chY > 2100)
        return false;

    if(chX < 900 || chX > 2100)
        return false;

    int throttle =
        map(chY,1000,2000,-maxTarget,maxTarget);

    int steering =
        map(chX,1000,2000,-maxTarget,maxTarget);

    if(abs(throttle) < deadband)
        throttle = 0;

    if(abs(steering) < deadband)
        steering = 0;

    int leftTarget =
        throttle + steering;

    int rightTarget =
        throttle - steering;

    leftTarget =
        constrain(leftTarget,
                  -maxTarget,
                  maxTarget);

    rightTarget =
        constrain(rightTarget,
                  -maxTarget,
                  maxTarget);

    currentLeft =
        rampValue(currentLeft,leftTarget);

    currentRight =
        rampValue(currentRight,rightTarget);

    targets.fl = currentLeft;
    targets.rl = currentLeft;

    targets.fr = currentRight;
    targets.rr = currentRight;

    return true;
}

MotorTargets getTargets()
{
    return targets;
}