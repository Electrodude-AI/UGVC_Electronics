#include "motor_driver.h"
#include "config.h"

void motorInit()
{
    pinMode(FL_PWM,OUTPUT);
    pinMode(FL_DIR,OUTPUT);

    pinMode(RL_PWM,OUTPUT);
    pinMode(RL_DIR,OUTPUT);

    pinMode(FR_PWM,OUTPUT);
    pinMode(FR_DIR,OUTPUT);

    pinMode(RR_PWM,OUTPUT);
    pinMode(RR_DIR,OUTPUT);
}

void setMotor(
    int pwmPin,
    int dirPin,
    float pwm,
    bool reverse)
{
    if(reverse)
        pwm = -pwm;

    if(pwm >= 0)
    {
        digitalWrite(dirPin,HIGH);
        analogWrite(pwmPin,pwm);
    }
    else
    {
        digitalWrite(dirPin,LOW);
        analogWrite(pwmPin,-pwm);
    }
}

void driveFL(float pwm)
{
    setMotor(FL_PWM,FL_DIR,pwm,false);
}

void driveRL(float pwm)
{
    setMotor(RL_PWM,RL_DIR,pwm,false);
}

void driveFR(float pwm)
{
    setMotor(FR_PWM,FR_DIR,pwm,true);
}

void driveRR(float pwm)
{
    setMotor(RR_PWM,RR_DIR,pwm,true);
}

void stopAllMotors()
{
    analogWrite(FL_PWM,0);
    analogWrite(RL_PWM,0);
    analogWrite(FR_PWM,0);
    analogWrite(RR_PWM,0);
}