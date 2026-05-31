#include <Arduino.h>
#include "config.h"
#include "receiver.h"
#include "encoder_manager.h"
#include "pid.h"
#include "motor_driver.h"

unsigned long lastLoop = 0;

void setup()
{
    Serial.begin(115200);

    receiverInit();
    encoderInit();
    motorInit();
}

void loop()
{
    if(millis() - lastLoop < loopTime)
        return;

    lastLoop = millis();

    if(!receiverUpdate())
    {
        stopAllMotors();
        resetPID();
        return;
    }

    MotorTargets t = getTargets();

    float flPWM =
        calculatePID(
            t.fl,
            getFLSpeed(),
            0);

    float rlPWM =
        calculatePID(
            t.rl,
            getRLSpeed(),
            1);

    float frPWM =
        calculatePID(
            t.fr,
            getFRSpeed(),
            2);

    float rrPWM =
        calculatePID(
            t.rr,
            getRRSpeed(),
            3);

    driveFL(flPWM);
    driveRL(rlPWM);

    driveFR(frPWM);
    driveRR(rrPWM);
}