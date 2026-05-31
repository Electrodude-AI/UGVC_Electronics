#ifndef RECEIVER_H
#define RECEIVER_H

struct MotorTargets
{
    float fl;
    float rl;
    float fr;
    float rr;
};

void receiverInit();

bool receiverUpdate();

MotorTargets getTargets();

#endif