#include "SerialComm.h"
#include "config.h"

SerialComm::SerialComm() : _rawFL(0), _rawFR(0), _rawRL(0), _rawRR(0), 
                           _tgtFL(0), _tgtFR(0), _tgtRL(0), _tgtRR(0), 
                           _bufIdx(0), _isReceiving(false), _lastReceiveTime(0) {}

void SerialComm::begin(long baudRate) {
    Serial.begin(baudRate);
    _lastReceiveTime = micros();
}

void SerialComm::spin() {
    // Read serial very fast without freezing the Teensy (Non-blocking)
    while (Serial.available() > 0) {
        char rc = Serial.read();

        if (rc == '<') { // Start marker
            _isReceiving = true;
            _bufIdx = 0;
        } 
        else if (rc == '>') { // End marker
            _isReceiving = false;
            _buffer[_bufIdx] = '\0'; // Terminate string
            parseData();
            _lastReceiveTime = micros(); // Update Watchdog
        } 
        else if (_isReceiving) {
            _buffer[_bufIdx] = rc;
            _bufIdx++;
            if (_bufIdx >= 64) _bufIdx = 63; // Prevent memory crash
        }
    }
}

void SerialComm::parseData() {
    // Splits the string into 4 float variables
    char* strtokIndx;
    
    strtokIndx = strtok(_buffer, ",");
    if(strtokIndx != NULL) _rawFL = atof(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    if(strtokIndx != NULL) _rawFR = atof(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    if(strtokIndx != NULL) _rawRL = atof(strtokIndx);
    
    strtokIndx = strtok(NULL, ",");
    if(strtokIndx != NULL) _rawRR = atof(strtokIndx);
}

void SerialComm::applyCommandDamping(float dt) {
    float alpha = config::CMD_LPF_ALPHA;
    _tgtFL += alpha * (_rawFL - _tgtFL);
    _tgtFR += alpha * (_rawFR - _tgtFR);
    _tgtRL += alpha * (_rawRL - _tgtRL);
    _tgtRR += alpha * (_rawRR - _tgtRR);
}

bool SerialComm::isWatchdogOK() {
    return (micros() - _lastReceiveTime) <= config::WATCHDOG_TIMEOUT_MICROS;
}

void SerialComm::publishFeedback(float tFL, float tFR, float tRL, float tRR,
                                 float mFL, float mFR, float mRL, float mRR,
                                 bool wdgOk, bool encOk, bool slipL, bool slipR) {
    // Send back formatted string: Target_FL, Target_FR, Target_RL, Target_RR, Meas_FL, Meas_FR, Meas_RL, Meas_RR, Watchdog_OK_Bool, Encoders_OK_Bool, Slip_Left_Bool, Slip_Right_Bool
    Serial.print(tFL); Serial.print(",");
    Serial.print(tFR); Serial.print(",");
    Serial.print(tRL); Serial.print(",");
    Serial.print(tRR); Serial.print(",");
    Serial.print(mFL); Serial.print(",");
    Serial.print(mFR); Serial.print(",");
    Serial.print(mRL); Serial.print(",");
    Serial.print(mRR); Serial.print(",");
    Serial.print(wdgOk ? 1 : 0); Serial.print(",");
    Serial.print(encOk ? 1 : 0); Serial.print(",");
    Serial.print(slipL ? 1 : 0); Serial.print(",");
    Serial.println(slipR ? 1 : 0); // println adds '\n' required by PySerial
}

float SerialComm::getTargetFL() { return _tgtFL; }
float SerialComm::getTargetFR() { return _tgtFR; }
float SerialComm::getTargetRL() { return _tgtRL; }
float SerialComm::getTargetRR() { return _tgtRR; }