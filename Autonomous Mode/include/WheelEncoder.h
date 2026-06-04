#pragma once
#include <Arduino.h>
#include <Encoder.h>

class WheelEncoder {
public:
    WheelEncoder(uint8_t pinA, uint8_t pinB, uint32_t cpr, bool invert = false);
    void reset();
    int32_t getCount();
    float getRPM(float dt);

    void updateHealth(float effort, float dt);
    bool isHealthy();

private:
    Encoder _hwEncoder;
    uint32_t _cpr;
    int32_t _lastCount;
    float _filteredVelocity;
    float _lpfAlpha;
    
    float _deadTime;
    bool _isDead;
    bool _invert;
};