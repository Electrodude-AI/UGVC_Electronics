#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

void motorInit();

void driveFL(float pwm);
void driveRL(float pwm);
void driveFR(float pwm);
void driveRR(float pwm);

void stopAllMotors();

#endif