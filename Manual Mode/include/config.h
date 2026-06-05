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

/* ENCODER */

const float CPR = 4784.0f;       // counts per revolution (quadrature)

/* SPEED LIMITER — 9.0 km/h hard cap */

const float MAX_SPEED_KMH = 9.0f;
const float MAX_SPEED_MS = MAX_SPEED_KMH / 3.6f;
const float WHEEL_DIAMETER_M = 0.305f;
const float WHEEL_CIRCUMFERENCE_M = 3.14159265359f * WHEEL_DIAMETER_M;

const float MAX_WHEEL_RPM = (MAX_SPEED_MS / WHEEL_CIRCUMFERENCE_M) * 60.0f;
const float MAX_CPS = MAX_WHEEL_RPM * (CPR / 60.0f);

/* CONTROL */

const int deadband = 30;
const float rampStep = 500.0f;       // CPS per loop (~1.0s 0-max)
const float rampStepOmega = 250.0f;  // CPS per loop
const int loopTime = 20;

// Skid-steer: omega contribution scaling (1.0 = same range as v)
const float WHEELBASE_FACTOR = 1.0f;

/* PID */

const float Kp = 0.015f;
const float Ki = 0.008f;        // Ti = Kp/Ki = 1.875s. Range: 0.005 (3s) to 0.012 (1.25s)
const float Kd = 0.0f;

// Feedforward gain — provides steady-state baseline, not saturation demand.
// Motor: 200 RPM / 255 PWM = 0.784 RPM/PWM
// PWM_per_RPM = 1 / 0.784 = 1.275 PWM/RPM
// 1 RPM = CPR/60 = 79.73 counts/sec
// PWM_per_counts = 1.275 / 79.73 = 0.01599
// FF_ratio = 0.75 caps FF at 75% of steady-state PWM, leaving 25% headroom for PID.
// kF = 0.01599 * 0.75 = 0.0120
// At 156.5 RPM (12482 cps): FF = 150 PWM, PID headroom = 105 PWM
const float kF = 0.0120f;

// Anti-windup
const float INTEGRAL_LIMIT = 10000.0f;

// Back-calculation gain (higher = faster unwind during saturation)
//   integral -= (output - 255) * ANTI_WINDUP_GAIN * dt   when saturated
// At 4.0: full clamp unwinds in ~1s at max saturation.
const float ANTI_WINDUP_GAIN = 4.0f;

// Encoder health (counts-per-second thresholds)
const float HEALTH_TARGET_NEAR_ZERO = 50.0f;
const float HEALTH_SPEED_RUNAWAY   = 200.0f;
const float HEALTH_TARGET_ACTIVE   = 500.0f;
const float HEALTH_SPEED_STUCK     = 5.0f;
const int   HEALTH_SUSPECT_MS      = 500;
const int   HEALTH_CLEAR_MS        = 1000;

// Slip detection
const float SLIP_ALPHA              = 0.15f;
const float SLIP_THRESH_LIGHT       = 0.25f;
const float SLIP_THRESH_MOD         = 0.40f;
const float SLIP_PWM_MIN            = 100.0f;
const int   SLIP_CONFIRM_MS         = 100;
const int   SLIP_CLEAR_MS           = 200;
const int   SLIP_RAMP_SUPPRESS      = 500;
const int   SLIP_REV_SUPPRESS       = 300;
const float SLIP_WEIGHT_DIV         = 0.6f;
const float SLIP_WEIGHT_ACCEL       = 0.3f;
const float SLIP_WEIGHT_KIN         = 0.1f;
const float SLIP_PWM_CAP            = 0.60f;

// Telemetry
// #define ENABLE_TELEMETRY

// SAFE_MODE — open-loop bring-up (10% max PWM, PID bypassed)
// #define SAFE_MODE

#ifdef SAFE_MODE
  const int   SAFE_MAX_TARGET = 40;
  const float SAFE_PWM_LIMIT  = 25.5f;
#endif

// MOTOR_TEST_MODE — sequential per-wheel test (30% max PWM, 3s per wheel)
// #define MOTOR_TEST_MODE

#ifdef MOTOR_TEST_MODE
  const float TEST_MAX_PWM     = 76.5f;
  const int   TEST_HALF_CYCLES = 75;
#endif

#endif
