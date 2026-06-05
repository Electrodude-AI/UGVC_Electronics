#ifndef PID_H
#define PID_H

#include "receiver.h"
#include "encoder_manager.h"
#include "motor_driver.h"

float calculatePID(
    float target,
    float actual,
    int motor);

MotorCommands pidComputeAll(
    const MotorTargets& targets,
    const WheelSpeeds&  speeds,
    const bool*         faultMask);

void resetPID();
void resetMotorPID(int motor);

#endif
