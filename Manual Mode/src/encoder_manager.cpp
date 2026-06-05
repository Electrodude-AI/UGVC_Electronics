#include <Encoder.h>

#include "encoder_manager.h"
#include "config.h"

Encoder encFL(FL_ENC_A, FL_ENC_B);
Encoder encRL(RL_ENC_A, RL_ENC_B);
Encoder encFR(FR_ENC_A, FR_ENC_B);
Encoder encRR(RR_ENC_A, RR_ENC_B);

long prevFL = 0;
long prevRL = 0;
long prevFR = 0;
long prevRR = 0;

unsigned long prevTimeFL = 0;
unsigned long prevTimeRL = 0;
unsigned long prevTimeFR = 0;
unsigned long prevTimeRR = 0;

void encoderInit()
{
    prevFL = encFL.read();
    prevRL = encRL.read();
    prevFR = encFR.read();
    prevRR = encRR.read();

    unsigned long now_us = micros();
    prevTimeFL = now_us;
    prevTimeRL = now_us;
    prevTimeFR = now_us;
    prevTimeRR = now_us;
}

static float computeSpeed(long& prevCount, unsigned long& prevTime, Encoder& enc)
{
    long now = enc.read();
    long delta = now - prevCount;
    prevCount = now;

    unsigned long now_us = micros();
    unsigned long dt_us = now_us - prevTime;
    prevTime = now_us;

    if(dt_us < 1)
        dt_us = 1;

    float dt_sec = dt_us / 1000000.0f;

    return delta / dt_sec;
}

WheelSpeeds getWheelSpeeds()
{
    WheelSpeeds ws;
    ws.fl = computeSpeed(prevFL, prevTimeFL, encFL);
    ws.rl = computeSpeed(prevRL, prevTimeRL, encRL);
    ws.fr = computeSpeed(prevFR, prevTimeFR, encFR);
    ws.rr = computeSpeed(prevRR, prevTimeRR, encRR);
    return ws;
}
