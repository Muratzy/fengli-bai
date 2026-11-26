#ifndef Q1_SWING_H
#define Q1_SWING_H

#include "control/pendulum_control.h"
#include "problems/problem_q1.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  Q1_SWING_IDLE = 0,
  Q1_SWING_RUNNING,
  Q1_SWING_FINISHED,
  Q1_SWING_STOPPED,
  Q1_SWING_FAULT
} Q1Swing_State;

typedef enum {
  Q1_SWING_NONE = 0,
  Q1_SWING_TIME_UP,
  Q1_SWING_USER_STOP,
  Q1_SWING_SAMPLE_STALE,
  Q1_SWING_ANGLE_LIMIT,
  Q1_SWING_NO_TURN,
  Q1_SWING_EN_LOW,
  Q1_SWING_NOT_CENTERED,
  Q1_SWING_INVALID_CONFIG,
  Q4_BRAKE_SETTLED,
  Q4_BRAKE_NO_RELEASE,
  Q4_BRAKE_NOT_SETTLED
} Q1Swing_Reason;

typedef enum {
  Q1_SWING_CONTROL_PID = 0,
  Q1_SWING_CONTROL_BLEND,
  Q1_SWING_CONTROL_AMPLITUDE
} Q1Swing_ControlPhase;

typedef struct {
  float startup_kp;
  float startup_kd;
  float amplitude_kp;
  float amplitude_ki;
  float amplitude_kd;
  float damping_compensation;
  float target_amplitude_deg;
  uint32_t duration_ms; /* Zero selects the default window. */
} Q1Swing_Tuning;

typedef struct {
  Q1Swing_State state;
  Q1Swing_Reason reason;
  ProblemQ1 reference;
  PendulumControl controller;
  Q1Swing_Tuning tuning;
  PID_Controller amplitude_pid;
  PendulumControl_Output control_output;
  uint32_t started_ms;
  uint32_t last_step_ms;
  uint32_t run_ms;
  uint32_t switches;
  int last_pwm_sign;
  bool pitch_integrating;
  bool amplitude_mode_enabled;
  bool amplitude_threshold_tracking;
  Q1Swing_ControlPhase control_phase;
  uint32_t amplitude_threshold_since_ms;
  uint32_t amplitude_blend_started_ms;
  uint16_t amplitude_blend_permille;
  float peak_deg;
  float left_peak_deg;
  float right_peak_deg;
  float amplitude_estimate_deg;
  float target_amplitude_deg;
  float amplitude_error_deg;
  float amplitude_feedback_output;
  float amplitude_compensation_output;
  float predicted_peak_deg;
  float target_roll_deg;
  float target_pitch_deg;
  float target_pitch_rate_dps;
  float target_position_m;
  float target_velocity_mps;
  int8_t leg_direction;
  bool have_previous_turn;
  uint32_t last_turn_ms;
  uint32_t last_center_update_ms;
  float leg_peak_deg;
  float previous_turn_deg;
  float center_target_deg;
  float center_estimate_deg;
  float center_output;
  uint16_t pwm[4];
} Q1Swing;

Q1Swing_Tuning Q1Swing_DefaultTuning(void);
bool Q1Swing_StartTuned(Q1Swing *swing, const Q1Swing_Tuning *tuning,
                        uint32_t now_ms, float roll_deg, float pitch_deg,
                        float pitch_rate_dps);
void Q1Swing_Stop(Q1Swing *swing, Q1Swing_Reason reason);
bool Q1Swing_Start(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                    float pitch_deg, float pitch_rate_dps);
bool Q1Swing_StartQ2(Q1Swing *swing, uint32_t length_mm, uint32_t now_ms,
                      float roll_deg, float pitch_deg, float pitch_rate_dps);
void Q1Swing_Step(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                   float pitch_deg, float pitch_rate_dps,
                   bool sample_valid, bool stop_requested);

#endif
