#include "WheelEncoder.h"
#include "config.h"

WheelEncoder::WheelEncoder(uint8_t pinA, uint8_t pinB, uint32_t cpr, bool invert)
    : _hwEncoder(pinA, pinB), _cpr(cpr), _lastCount(0), 
      _filteredVelocity(0.0f), _lpfAlpha(0.20f), _deadTime(0.0f), _isDead(false), _invert(invert) {}

void WheelEncoder::reset() {
    _hwEncoder.write(0);
    _lastCount = 0;
    _filteredVelocity = 0.0f;
    _deadTime = 0.0f;
    _isDead = false;
}

int32_t WheelEncoder::getCount() {
    return _hwEncoder.read();
}

float WheelEncoder::getRPM(float dt) {
    if (dt <= 0.0f) return _filteredVelocity;

    int32_t currentCount = getCount();
    int32_t deltaTicks = currentCount - _lastCount;
    _lastCount = currentCount;

    float currentRPM = (static_cast<float>(deltaTicks) / static_cast<float>(_cpr)) * (60.0f / dt);
    
    if (_invert) {
        currentRPM = -currentRPM;
    }
    
    _filteredVelocity = (_lpfAlpha * currentRPM) + ((1.0f - _lpfAlpha) * _filteredVelocity);
    
    return _filteredVelocity;
}

void WheelEncoder::updateHealth(float effort, float dt) {
    if (_isDead) return;

    if (abs(effort) > config::MOTOR_MIN_EFFORT_THRESH && abs(_filteredVelocity) < config::ENCODER_MIN_RPM_THRESH) {
        _deadTime += dt;
        if (_deadTime >= config::ENCODER_HEALTH_TIMEOUT_SEC) {
            _isDead = true;
        }
    } else {
        _deadTime = 0.0f;
    }
}

bool WheelEncoder::isHealthy() {
    return !_isDead;
}