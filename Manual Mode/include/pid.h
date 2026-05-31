#ifndef PID_H
#define PID_H

float calculatePID(
    float target,
    float actual,
    int motor);

void resetPID();

#endif