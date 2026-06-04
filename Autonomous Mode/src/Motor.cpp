#include "Motor.h"

MotorDriver::MotorDriver(uint8_t pwmPin, uint8_t dirPin, float deadband)
    : _pwmPin(pwmPin), _dirPin(dirPin), _deadband(deadband), _lastPwm(-1), _lastDir(-1) {}

void MotorDriver::begin() {
    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    analogWriteResolution(8);
    setSpeed(0.0f);
}

void MotorDriver::setSpeed(float targetValue) {
    // 1. Strict Clamping (-1.0 to 1.0)
    if (targetValue > 1.0f) targetValue = 1.0f;
    else if (targetValue < -1.0f) targetValue = -1.0f;

    // 2. Deadband Check
    if (abs(targetValue) < _deadband) {
        analogWrite(_pwmPin, 0);
        _lastPwm = 0;
        return;
    }

    // 3. Cytron Hardware Application (INSTANT - No Soft Start Delay)
    uint8_t newDir = (targetValue >= 0.0f) ? HIGH : LOW;
    uint8_t newPwm = static_cast<uint8_t>(abs(targetValue) * 255.0f);

    if (_lastDir != newDir) { digitalWrite(_dirPin, newDir); _lastDir = newDir; }
    if (_lastPwm != newPwm) { analogWrite(_pwmPin, newPwm); _lastPwm = newPwm; }
}

void MotorDriver::brake() {
    // Dynamic Braking: short the motor leads to ground by sending 0 PWM and pulling DIR low
    analogWrite(_pwmPin, 0);
    digitalWrite(_dirPin, LOW);
    _lastPwm = 0;
    _lastDir = LOW;
}