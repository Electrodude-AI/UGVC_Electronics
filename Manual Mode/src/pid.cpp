#include "pid.h"
#include "config.h"

float integral[4];
float prevError[4];

float calculatePID(
    float target,
    float actual,
    int motor)
{
    float error =
        target - actual;

    integral[motor] += error;

    float derivative =
        error - prevError[motor];

    float output =
        (Kp * error) +
        (Ki * integral[motor]) +
        (Kd * derivative);

    prevError[motor] = error;

    output =
        constrain(output,-255,255);

    return output;
}

void resetPID()
{
    for(int i=0;i<4;i++)
    {
        integral[i]=0;
        prevError[i]=0;
    }
}