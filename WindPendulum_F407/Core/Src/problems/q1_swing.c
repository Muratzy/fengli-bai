#include "problems/q1_swing.h"
#include "app_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define Q1_MAX_SAMPLE_GAP_MS 20U

static PID_Config Q1Swing_PidConfig(void)
{
  const PID_Config config = {
      .kp = APP_Q1_PITCH_KP,
      .ki = APP_Q1_PITCH_KI,
      .kd = APP_Q1_PITCH_KD,
      .proportional_setpoint_weight = 1.0F,
      .derivative_filter_tau_s = APP_Q1_RATE_FILTER_TAU_S,
      .integral_min = -APP_Q1_INTEGRAL_LIMIT,
      .integral_max = APP_Q1_INTEGRAL_LIMIT,
      .output_min = -(float)APP_Q1_PWM_MAX_PERMILLE,
      .output_max = (float)APP_Q1_PWM_MAX_PERMILLE};
  return config;
}

void Q1Swing_Stop(Q1Swing *swing, Q1Swing_Reason reason)
{
  if (swing == NULL) return;
  if (reason != Q1_SWING_TIME_UP) ProblemQ1_Cancel(&swing->reference);
  swing->reason = reason;
  swing->state = (reason == Q1_SWING_TIME_UP) ? Q1_SWING_FINISHED :
      (reason == Q1_SWING_USER_STOP) ? Q1_SWING_STOPPED : Q1_SWING_FAULT;
  swing->pitch_integrating = false;
  memset(swing->pwm, 0, sizeof(swing->pwm));
  memset(&swing->control_output, 0, sizeof(swing->control_output));
  PendulumControl_Reset(&swing->controller, 0.0F, 0.0F);
}

bool Q1Swing_Start(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                    float pitch_deg, float pitch_rate_dps)
{
  if (swing == NULL || swing->state == Q1_SWING_RUNNING) return false;
  *swing = (Q1Swing){0};
  if (!isfinite(roll_deg) || !isfinite(pitch_deg) ||
      !isfinite(pitch_rate_dps) ||
      fabsf(roll_deg) > APP_Q1_START_ANGLE_DEG ||
      fabsf(pitch_deg) > APP_Q1_START_ANGLE_DEG) {
    Q1Swing_Stop(swing, Q1_SWING_NOT_CENTERED);
    return false;
  }

  const ProblemQ1_Config reference_config = ProblemQ1_DefaultConfig();
  if (ProblemQ1_Start(&swing->reference, &reference_config, now_ms,
                      true, true) != PROBLEM_Q1_OK) {
    Q1Swing_Stop(swing, Q1_SWING_INVALID_CONFIG);
    return false;
  }

  const PID_Config pid = Q1Swing_PidConfig();
  const PendulumControl_Config control_config = {
      .roll_pid = pid,
      .pitch_pid = pid,
      /* Positive Pitch is right; PWM1 moves right and PWM2 moves left. */
      .fan_mix = {{0.0F, 1.0F}, {0.0F, -1.0F},
                  {0.0F, 0.0F}, {0.0F, 0.0F}},
      .max_duty_permille = APP_Q1_PWM_MAX_PERMILLE,
      .mixer_calibrated = true,
      .disabled_axis_mask = PENDULUM_DISABLE_ROLL};
  if (!PendulumControl_Init(&swing->controller, &control_config)) {
    Q1Swing_Stop(swing, Q1_SWING_INVALID_CONFIG);
    return false;
  }

  PendulumControl_Reset(&swing->controller, roll_deg, pitch_deg);
  swing->state = Q1_SWING_RUNNING;
  swing->started_ms = now_ms;
  swing->last_step_ms = now_ms;
  return true;
}

static void Q1Swing_UpdatePwm(Q1Swing *swing)
{
  memcpy(swing->pwm, swing->control_output.pwm_permille, sizeof(swing->pwm));
  const int sign = (swing->pwm[0] > 0U) ? 1 :
      (swing->pwm[1] > 0U) ? -1 : 0;
  if (sign != 0) {
    if (swing->last_pwm_sign != 0 && sign != swing->last_pwm_sign) {
      swing->switches++;
    }
    swing->last_pwm_sign = sign;
  }
}

static void Q1Swing_UpdateTurnMonitor(Q1Swing *swing, float pitch_deg)
{
  if (pitch_deg > swing->right_peak_deg) swing->right_peak_deg = pitch_deg;
  if (-pitch_deg > swing->left_peak_deg) swing->left_peak_deg = -pitch_deg;
  swing->peak_deg = fmaxf(swing->right_peak_deg, swing->left_peak_deg);
}

void Q1Swing_Step(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                   float pitch_deg, float pitch_rate_dps,
                   bool sample_valid, bool stop_requested)
{
  if (swing == NULL || swing->state != Q1_SWING_RUNNING) return;
  if (stop_requested) {
    Q1Swing_Stop(swing, Q1_SWING_USER_STOP);
    return;
  }

  const uint32_t elapsed_ms = now_ms - swing->last_step_ms;
  if (!sample_valid || elapsed_ms > Q1_MAX_SAMPLE_GAP_MS ||
      !isfinite(roll_deg) || !isfinite(pitch_deg) ||
      !isfinite(pitch_rate_dps)) {
    Q1Swing_Stop(swing, Q1_SWING_SAMPLE_STALE);
    return;
  }
  if (fabsf(pitch_deg) > APP_Q1_STOP_PITCH_DEG ||
      fabsf(roll_deg) > APP_Q1_STOP_ROLL_DEG ||
      fabsf(pitch_rate_dps) > 150.0F) {
    Q1Swing_Stop(swing, Q1_SWING_ANGLE_LIMIT);
    return;
  }

  swing->last_step_ms = now_ms;
  swing->run_ms = now_ms - swing->started_ms;
  Q1Swing_UpdateTurnMonitor(swing, pitch_deg);

  ProblemQ1_Output reference;
  if (!ProblemQ1_Step(&swing->reference, now_ms, &reference)) {
    Q1Swing_Stop(swing, Q1_SWING_INVALID_CONFIG);
    return;
  }
  if (reference.complete) {
    Q1Swing_Stop(swing, Q1_SWING_TIME_UP);
    return;
  }

  swing->target_roll_deg = reference.target_roll_deg;
  swing->target_pitch_deg = reference.target_pitch_deg;
  swing->target_pitch_rate_dps = reference.target_pitch_rate_dps;
  swing->target_position_m = reference.target_position_m;
  swing->target_velocity_mps = reference.target_velocity_mps;
  if (elapsed_ms == 0U) return;

  const PendulumControl_Input input = {
      .target_deg = {reference.target_roll_deg, reference.target_pitch_deg},
      /* Use measured-rate damping for the Q1 comparison. */
      .target_rate_dps = {0.0F, 0.0F},
      .angle_deg = {roll_deg, pitch_deg},
      .rate_dps = {0.0F, pitch_rate_dps},
      .dt_s = (float)elapsed_ms * 0.001F,
      .integrate = {false, swing->controller.config.pitch_pid.ki != 0.0F}};
  swing->pitch_integrating = input.integrate[PENDULUM_AXIS_PITCH];
  if (!PendulumControl_StepWithRates(&swing->controller, &input,
                                      &swing->control_output) ||
      !swing->control_output.pwm_ready) {
    Q1Swing_Stop(swing, Q1_SWING_INVALID_CONFIG);
    return;
  }
  Q1Swing_UpdatePwm(swing);
}
