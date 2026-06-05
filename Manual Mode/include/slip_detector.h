#ifndef SLIP_DETECTOR_H
#define SLIP_DETECTOR_H

#include "receiver.h"
#include "encoder_manager.h"
#include "motor_driver.h"

void slipDetectUpdate(
    const MotorTargets&  targets,
    const WheelSpeeds&   speeds,
    const MotorCommands& cmds,
    unsigned long        nowMs,
    float                dt);

void slipApplyResponse(MotorCommands& cmds);

bool  slipIsSlipping(int wheel);
bool  slipAnySlipping();
float slipGetIndex(int wheel);
void  slipReset();

#endif
