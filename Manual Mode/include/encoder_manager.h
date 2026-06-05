#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

struct WheelSpeeds
{
    float fl;
    float rl;
    float fr;
    float rr;
};

void encoderInit();

WheelSpeeds getWheelSpeeds();

#endif
