#ifndef PROBLEM_Q2_H
#define PROBLEM_Q2_H

#include "control/pendulum_control.h"
#include <stdbool.h>
#include <stdint.h>

/* State and reason values share the existing trial telemetry encoding. */
typedef enum {
  PROBLEM_Q2_IDLE = 0, PROBLEM_Q2_RUNNING, PROBLEM_Q2_FINISHED,
  PROBLEM_Q2_STOPPED, PROBLEM_Q2_FAULT
} ProblemQ2_State;

typedef enum {
  PROBLEM_Q2_NONE = 0, PROBLEM_Q2_TIME_UP, PROBLEM_Q2_USER_STOP,
  PROBLEM_Q2_SAMPLE_STALE, PROBLEM_Q2_ANGLE_LIMIT, PROBLEM_Q2_RESERVED,
  PROBLEM_Q2_EN_LOW, PROBLEM_Q2_NOT_CENTERED, PROBLEM_Q2_INVALID_CONFIG
} ProblemQ2_Reason;

typedef struct {
  float line_length_m;
  float ground_height_m;
  uint32_t period_ms;
  uint32_t amplitude_ramp_ms;
  uint32_t duration_ms; /* Bench window ending is not a qualification verdict. */
  uint16_t max_pwm_permille;
  PID_Config roll_pid;
  PID_Config pitch_pid;
} ProblemQ2_Config;

typedef struct {
  ProblemQ2_State state;
  ProblemQ2_Reason reason;
  ProblemQ2_Config config;
  PendulumControl controller;
  PendulumControl_Output control_output;
  uint32_t started_ms;
  uint32_t last_step_ms;
  uint32_t run_ms;
  uint32_t switches; /* Actual applied PWM polarity changes, not measured turns. */
  int last_pwm_sign;
  uint16_t pwm[4];
  float target_roll_deg;
  float target_pitch_deg;
  float target_pitch_rate_dps;
  float target_position_m;
  float target_velocity_mps;
  float left_peak_deg;
  float right_peak_deg;
  bool pitch_integrating;
} ProblemQ2;

ProblemQ2_Config ProblemQ2_DefaultConfig(void);
bool ProblemQ2_Validate(const ProblemQ2_Config *config);
bool ProblemQ2_Start(ProblemQ2 *problem, const ProblemQ2_Config *config,
                      uint32_t now_ms, float roll_deg, float pitch_deg);
void ProblemQ2_Stop(ProblemQ2 *problem, ProblemQ2_Reason reason);
void ProblemQ2_Step(ProblemQ2 *problem, uint32_t now_ms,
                     float roll_deg, float pitch_deg,
                     float roll_rate_dps, float pitch_rate_dps,
                     bool sample_valid, bool stop_requested);

#endif
