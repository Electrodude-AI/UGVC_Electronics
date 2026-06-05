#include "motor_driver.h"
#include "config.h"

static int  prevMotorDir[4]  = {0, 0, 0, 0};
static bool motorEnabled[4]  = {true, true, true, true};

static int motorIndex(int pwmPin)
{
    if(pwmPin == FL_PWM) return 0;
    if(pwmPin == RL_PWM) return 1;
    if(pwmPin == FR_PWM) return 2;
    return 3;
}

void motorInit()
{
    pinMode(FL_PWM, OUTPUT);
    pinMode(FL_DIR, OUTPUT);

    pinMode(RL_PWM, OUTPUT);
    pinMode(RL_DIR, OUTPUT);

    pinMode(FR_PWM, OUTPUT);
    pinMode(FR_DIR, OUTPUT);

    pinMode(RR_PWM, OUTPUT);
    pinMode(RR_DIR, OUTPUT);
}

void motorSetEnabled(int wheel, bool enabled)
{
    if(wheel < 0 || wheel > 3)
        return;

    motorEnabled[wheel] = enabled;

    if(!enabled)
    {
        static const int pwmPins[4] = {FL_PWM, RL_PWM, FR_PWM, RR_PWM};
        analogWrite(pwmPins[wheel], 0);
    }
}

static void setMotor(
    int pwmPin,
    int dirPin,
    float pwm,
    bool reverse)
{
    if(reverse)
        pwm = -pwm;

    int idx = motorIndex(pwmPin);

    if(!motorEnabled[idx])
    {
        analogWrite(pwmPin, 0);
        return;
    }

    int newDir = (pwm >= 0) ? 1 : -1;

    if(newDir != prevMotorDir[idx] && prevMotorDir[idx] != 0)
    {
        analogWrite(pwmPin, 0);
        delayMicroseconds(100);
    }

    if(pwm >= 0)
    {
        digitalWrite(dirPin, HIGH);
        analogWrite(pwmPin, pwm);
    }
    else
    {
        digitalWrite(dirPin, LOW);
        analogWrite(pwmPin, -pwm);
    }

    prevMotorDir[idx] = newDir;
}

void driveFL(float pwm)
{
    setMotor(FL_PWM, FL_DIR, pwm, false);
}

void driveRL(float pwm)
{
    setMotor(RL_PWM, RL_DIR, pwm, false);
}

void driveFR(float pwm)
{
    setMotor(FR_PWM, FR_DIR, pwm, true);
}

void driveRR(float pwm)
{
    setMotor(RR_PWM, RR_DIR, pwm, true);
}

void motorApply(const MotorCommands& cmds)
{
    driveFL(cmds.fl);
    driveRL(cmds.rl);
    driveFR(cmds.fr);
    driveRR(cmds.rr);
}

void stopAllMotors()
{
    analogWrite(FL_PWM, 0);
    analogWrite(RL_PWM, 0);
    analogWrite(FR_PWM, 0);
    analogWrite(RR_PWM, 0);
}

#ifdef MOTOR_TEST_MODE

#include "encoder_manager.h"

static int testWheel;
static int testDir;
static int testStep;
static bool testDone;
static const char* testWheelTag[4] = {"FL","RL","FR","RR"};

void motorTestInit()
{
    testWheel = 0;
    testDir   = 0;
    testStep  = 0;
    testDone  = false;
    stopAllMotors();
    Serial.println("=== MOTOR TEST MODE ===");
    Serial.println("FL fwd -> FL rev -> RL fwd -> RL rev -> FR fwd -> FR rev -> RR fwd -> RR rev");
}

bool motorTestLoop()
{
    if(testDone)
        return true;

    float t;
    if(testStep < TEST_HALF_CYCLES)
        t = (float)testStep / (TEST_HALF_CYCLES - 1);
    else
        t = (float)((2 * TEST_HALF_CYCLES - 1) - testStep) / (TEST_HALF_CYCLES - 1);

    float pwm = t * TEST_MAX_PWM;
    if(testDir == 1)
        pwm = -pwm;

    switch(testWheel)
    {
        case 0: driveFL(pwm); break;
        case 1: driveRL(pwm); break;
        case 2: driveFR(pwm); break;
        case 3: driveRR(pwm); break;
    }

    if(testStep % 10 == 0 || testStep == 0 || testStep == 2 * TEST_HALF_CYCLES - 1)
    {
        WheelSpeeds ws = getWheelSpeeds();
        float speed = 0;
        switch(testWheel)
        {
            case 0: speed = ws.fl; break;
            case 1: speed = ws.rl; break;
            case 2: speed = ws.fr; break;
            case 3: speed = ws.rr; break;
        }
        const char* dirTag = (testDir == 0) ? "FWD" : "REV";
        Serial.print(testWheelTag[testWheel]);
        Serial.print(" ");
        Serial.print(dirTag);
        Serial.print(" step=");
        Serial.print(testStep);
        Serial.print(" PWM=");
        Serial.print(pwm, 0);
        Serial.print(" enc=");
        Serial.println(speed, 1);
    }

    testStep++;

    if(testStep >= 2 * TEST_HALF_CYCLES)
    {
        testStep = 0;
        testDir++;

        if(testDir >= 2)
        {
            testDir = 0;
            testWheel++;

            if(testWheel >= 4)
            {
                testDone = true;
                stopAllMotors();
                Serial.println("=== MOTOR TEST COMPLETE ===");
                return true;
            }

            Serial.print("=== ");
            Serial.print(testWheelTag[testWheel]);
            Serial.println(" forward ===");
        }
        else
        {
            Serial.print("=== ");
            Serial.print(testWheelTag[testWheel]);
            Serial.println(" reverse ===");
        }
    }

    return false;
}

#endif
