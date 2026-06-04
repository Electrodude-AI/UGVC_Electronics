#include "PID.h"

PIDController::PIDController(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd), _minOut(-1.0f), _maxOut(1.0f), _maxInt(1.0f), _integral(0.0f), _prevError(0.0f) {}

void PIDController::setLimits(float minOut, float maxOut, float maxInt) {
    _minOut = minOut; _maxOut = maxOut; _maxInt = maxInt;
}

void PIDController::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
}

float PIDController::compute(float target, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = target - measured;
    float pOut = _kp * error;

    _integral += (_ki * error * dt);
    if (_integral > _maxInt) _integral = _maxInt;
    else if (_integral < -_maxInt) _integral = -_maxInt;

    float dOut = _kd * ((error - _prevError) / dt);
    _prevError = error;

    float output = pOut + _integral + dOut;
    if (output > _maxOut) output = _maxOut;
    else if (output < _minOut) output = _minOut;

    return output;
}