#pragma once
#include <Arduino.h>

class SerialComm {
public:
    SerialComm();
    void begin(long baudRate);
    
    // Reads incoming PySerial data
    void spin(); 
    
    // Applies low-pass filter to raw targets
    void applyCommandDamping(float dt);
    
    // Watchdog check
    bool isWatchdogOK();

    // Sends the advanced telemetry back to PySerial
    void publishFeedback(float tFL, float tFR, float tRL, float tRR,
                         float mFL, float mFR, float mRL, float mRR,
                         bool wdgOk, bool encOk, bool slipL, bool slipR);

    // Getters for the 4 damped targets
    float getTargetFL(); float getTargetFR();
    float getTargetRL(); float getTargetRR();

private:
    void parseData();
    
    float _rawFL, _rawFR, _rawRL, _rawRR;
    float _tgtFL, _tgtFR, _tgtRL, _tgtRR;
    
    // Non-blocking serial buffer variables
    char _buffer[64];
    uint8_t _bufIdx;
    bool _isReceiving;
    
    uint32_t _lastReceiveTime;
};