#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* RECEIVER */

#define CH_Y 14
#define CH_X 15

/* FRONT LEFT */

#define FL_PWM 2
#define FL_DIR 3
#define FL_ENC_A 4
#define FL_ENC_B 5

/* REAR LEFT */

#define RL_PWM 6
#define RL_DIR 7
#define RL_ENC_A 8
#define RL_ENC_B 9

/* FRONT RIGHT */

#define FR_PWM 10
#define FR_DIR 11
#define FR_ENC_A 12
#define FR_ENC_B 24

/* REAR RIGHT */

#define RR_PWM 25
#define RR_DIR 26
#define RR_ENC_A 27
#define RR_ENC_B 28

/* CONTROL */

const int deadband = 30;
const int maxTarget = 400;
const int rampStep = 10;
const int loopTime = 20;

/* PID */

const float Kp = 1.5;
const float Ki = 0.05;
const float Kd = 0.02;

#endif