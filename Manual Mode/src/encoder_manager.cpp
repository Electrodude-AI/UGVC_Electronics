#include <Encoder.h>

#include "encoder_manager.h"
#include "config.h"

Encoder encFL(FL_ENC_A,FL_ENC_B);
Encoder encRL(RL_ENC_A,RL_ENC_B);
Encoder encFR(FR_ENC_A,FR_ENC_B);
Encoder encRR(RR_ENC_A,RR_ENC_B);

long prevFL = 0;
long prevRL = 0;
long prevFR = 0;
long prevRR = 0;

void encoderInit()
{
}

float getFLSpeed()
{
    long now = encFL.read();

    float speed = now - prevFL;

    prevFL = now;

    return speed;
}

float getRLSpeed()
{
    long now = encRL.read();

    float speed = now - prevRL;

    prevRL = now;

    return speed;
}

float getFRSpeed()
{
    long now = encFR.read();

    float speed = now - prevFR;

    prevFR = now;

    return speed;
}

float getRRSpeed()
{
    long now = encRR.read();

    float speed = now - prevRR;

    prevRR = now;

    return speed;
}