#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "receiver.h"
#include "encoder_manager.h"

void safetyInit();

void safetyUpdate(
    const MotorTargets& targets,
    const WheelSpeeds&  speeds);

bool safetyCriticalFault();

const bool* safetyFaultMask();

void safetyReset();

#endif
