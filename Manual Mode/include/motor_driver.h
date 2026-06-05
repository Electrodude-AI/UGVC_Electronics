#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

struct MotorCommands
{
    float fl;
    float rl;
    float fr;
    float rr;
};

void motorInit();

void driveFL(float pwm);
void driveRL(float pwm);
void driveFR(float pwm);
void driveRR(float pwm);

void motorApply(const MotorCommands& cmds);

void stopAllMotors();

void motorSetEnabled(int wheel, bool enabled);

#ifdef MOTOR_TEST_MODE
void motorTestInit();
bool motorTestLoop();
#endif

#endif
