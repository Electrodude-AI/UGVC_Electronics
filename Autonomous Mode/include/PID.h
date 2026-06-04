#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    void setLimits(float minOut, float maxOut, float maxInt);
    float compute(float target, float measured, float dt);
    void reset();

private:
    float _kp, _ki, _kd;
    float _minOut, _maxOut, _maxInt;
    float _integral, _prevError;
};