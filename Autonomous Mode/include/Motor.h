#pragma once
#include <Arduino.h>

class MotorDriver {
public:
    MotorDriver(uint8_t pwmPin, uint8_t dirPin, float deadband);
    void begin();
    
    // Instantly applies effort to the motor (no slew rate)
    void setSpeed(float targetValue); 
    
    // Hard dynamic brake
    void brake();

private:
    uint8_t _pwmPin;
    uint8_t _dirPin;
    float _deadband;
    
    int16_t _lastPwm;
    int8_t _lastDir;
};