#include "problems/problem_q2.h"
#include "app_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define Q2_PI 3.1415926535897932F
#define Q2_DEG_PER_RAD 57.2957795130823F
#define Q2_RAD_PER_DEG 0.0174532925199433F

ProblemQ2_Config ProblemQ2_DefaultConfig(void)
{
  const PID_Config pid = {
      .kp = APP_Q2_PITCH_KP, .ki = APP_Q2_PITCH_KI, .kd = APP_Q2_PITCH_KD,
      .proportional_setpoint_weight = 1.0F,
      .derivative_filter_tau_s = APP_Q2_RATE_FILTER_TAU_S,
      .integral_min = -APP_Q2_INTEGRAL_LIMIT,
      .integral_max = APP_Q2_INTEGRAL_LIMIT,
      .output_min = -(float)APP_Q2_PWM_MAX_PERMILLE,
      .output_max = (float)APP_Q2_PWM_MAX_PERMILLE};
  const ProblemQ2_Config config = {
      .line_length_m = (float)APP_Q2_DEFAULT_LENGTH_MM * 0.001F,
      .ground_height_m = APP_PIVOT_GROUND_HEIGHT_M,
      .period_ms = APP_Q2_PERIOD_MS,
      .amplitude_ramp_ms = APP_Q2_AMPLITUDE_RAMP_MS,
      .duration_ms = APP_Q2_DURATION_MS,
      .pwm_rise_ms = APP_Q2_PWM_RISE_MS, .pwm_fall_ms = APP_Q2_PWM_FALL_MS,
      .max_pwm_permille = APP_Q2_PWM_MAX_PERMILLE,
      .roll_pid = pid, .pitch_pid = pid};
  return config;
}

bool ProblemQ2_Validate(const ProblemQ2_Config *config)
{
  PID_Controller checked;
  if (config == NULL || !isfinite(config->line_length_m) ||
      config->line_length_m < 0.30F || config->line_length_m > 0.60F ||
      !isfinite(config->ground_height_m) || config->ground_height_m <= 0.0F ||
      config->period_ms < 500U || config->period_ms > 5000U ||
      config->duration_ms == 0U || config->duration_ms > 15000U ||
      config->amplitude_ramp_ms == 0U || config->period_ms > config->duration_ms ||
      config->amplitude_ramp_ms > config->duration_ms - config->period_ms ||
      config->pwm_rise_ms == 0U || config->pwm_rise_ms > 2000U ||
      config->pwm_fall_ms == 0U || config->pwm_fall_ms > 2000U ||
      config->max_pwm_permille == 0U || config->max_pwm_permille > 1000U ||
      atan2f(0.5F * config->line_length_m, config->ground_height_m) *
          Q2_DEG_PER_RAD > 20.0F) return false;
  return PID_Init(&checked, &config->roll_pid) &&
         PID_Init(&checked, &config->pitch_pid) &&
         config->pitch_pid.output_min == -(float)config->max_pwm_permille &&
         config->pitch_pid.output_max == (float)config->max_pwm_permille;
}

void ProblemQ2_Stop(ProblemQ2 *problem, ProblemQ2_Reason reason)
{
  if (problem == NULL) return;
  problem->reason = reason;
  problem->pitch_integrating = false;
  problem->state = (reason == PROBLEM_Q2_TIME_UP) ? PROBLEM_Q2_FINISHED :
      (reason == PROBLEM_Q2_USER_STOP) ? PROBLEM_Q2_STOPPED : PROBLEM_Q2_FAULT;
  memset(problem->pwm, 0, sizeof(problem->pwm));
  memset(problem->slew_duty, 0, sizeof(problem->slew_duty));
  memset(&problem->control_output, 0, sizeof(problem->control_output));
  PendulumControl_Reset(&problem->controller, 0.0F, 0.0F);
}

bool ProblemQ2_Start(ProblemQ2 *problem, const ProblemQ2_Config *config,
                      uint32_t now_ms, float roll_deg, float pitch_deg)
{
  if (problem == NULL || problem->state == PROBLEM_Q2_RUNNING) return false;
  *problem = (ProblemQ2){0};
  if (!ProblemQ2_Validate(config)) {
    ProblemQ2_Stop(problem, PROBLEM_Q2_INVALID_CONFIG);
    return false;
  }
  if (!isfinite(roll_deg) || !isfinite(pitch_deg) ||
      fabsf(roll_deg) > APP_Q1_START_ANGLE_DEG ||
      fabsf(pitch_deg) > APP_Q1_START_ANGLE_DEG) {
    ProblemQ2_Stop(problem, PROBLEM_Q2_NOT_CENTERED);
    return false;
  }
  PendulumControl_Config control = {
      .roll_pid = config->roll_pid, .pitch_pid = config->pitch_pid,
      .fan_mix = {{0.0F, 1.0F}, {0.0F, -1.0F}, {0.0F, 0.0F}, {0.0F, 0.0F}},
      .max_duty_permille = config->max_pwm_permille,
      .mixer_calibrated = true, .disabled_axis_mask = PENDULUM_DISABLE_ROLL};
  if (!PendulumControl_Init(&problem->controller, &control)) {
    ProblemQ2_Stop(problem, PROBLEM_Q2_INVALID_CONFIG);
    return false;
  }
  PendulumControl_Reset(&problem->controller, roll_deg, pitch_deg);
  problem->config = *config;
  problem->started_ms = now_ms;
  problem->last_step_ms = now_ms;
  problem->state = PROBLEM_Q2_RUNNING;
  return true;
}

static float Approach(float current, float target, float rise, float fall)
{
  if (current < target) return fminf(current + rise, target);
  return fmaxf(current - fall, target);
}

static void ProblemQ2_Slew(ProblemQ2 *problem, uint32_t elapsed_ms)
{
  const float rise = (float)problem->config.max_pwm_permille *
      (float)elapsed_ms / (float)problem->config.pwm_rise_ms;
  const float fall = (float)problem->config.max_pwm_permille *
      (float)elapsed_ms / (float)problem->config.pwm_fall_ms;
  const uint16_t *desired = problem->control_output.pwm_permille;
  if (problem->slew_duty[0] > 0.0F && desired[1] > 0U) {
    problem->slew_duty[0] = Approach(problem->slew_duty[0], 0.0F, rise, fall);
    problem->slew_duty[1] = 0.0F;
  } else if (problem->slew_duty[1] > 0.0F && desired[0] > 0U) {
    problem->slew_duty[1] = Approach(problem->slew_duty[1], 0.0F, rise, fall);
    problem->slew_duty[0] = 0.0F;
  } else {
    for (uint32_t side = 0U; side < 2U; ++side)
      problem->slew_duty[side] = Approach(problem->slew_duty[side],
          (float)desired[side], rise, fall);
  }
  for (uint32_t side = 0U; side < 2U; ++side)
    problem->pwm[side] = (uint16_t)(problem->slew_duty[side] + 0.5F);
  problem->pwm[2] = 0U;
  problem->pwm[3] = 0U;
  const int sign = (problem->pwm[0] > 0U) ? 1 :
      (problem->pwm[1] > 0U) ? -1 : 0;
  if (sign != 0) {
    if (problem->last_pwm_sign != 0 && sign != problem->last_pwm_sign)
      problem->switches++;
    problem->last_pwm_sign = sign;
  }
}

void ProblemQ2_Step(ProblemQ2 *problem, uint32_t now_ms,
                     float roll_deg, float pitch_deg,
                     float roll_rate_dps, float pitch_rate_dps,
                     bool sample_valid, bool stop_requested)
{
  if (problem == NULL || problem->state != PROBLEM_Q2_RUNNING) return;
  if (stop_requested) {
    ProblemQ2_Stop(problem, PROBLEM_Q2_USER_STOP);
    return;
  }
  const uint32_t elapsed_ms = now_ms - problem->last_step_ms;
  if (!sample_valid || elapsed_ms > 20U || !isfinite(roll_deg) ||
      !isfinite(pitch_deg) || !isfinite(roll_rate_dps) ||
      !isfinite(pitch_rate_dps)) {
    ProblemQ2_Stop(problem, PROBLEM_Q2_SAMPLE_STALE);
    return;
  }
  if (fabsf(pitch_deg) > APP_Q1_STOP_PITCH_DEG ||
      fabsf(roll_deg) > APP_Q1_STOP_ROLL_DEG ||
      fabsf(pitch_rate_dps) > 150.0F || fabsf(roll_rate_dps) > 150.0F) {
    ProblemQ2_Stop(problem, PROBLEM_Q2_ANGLE_LIMIT);
    return;
  }
  problem->last_step_ms = now_ms;
  problem->run_ms = now_ms - problem->started_ms;
  if (problem->run_ms >= problem->config.duration_ms) {
    ProblemQ2_Stop(problem, PROBLEM_Q2_TIME_UP);
    return;
  }
  if (elapsed_ms == 0U) return;
  const float seconds = (float)problem->run_ms * 0.001F;
  const float omega = 2.0F * Q2_PI * 1000.0F / (float)problem->config.period_ms;
  const float amplitude = atan2f(0.5F * problem->config.line_length_m,
      problem->config.ground_height_m) * Q2_DEG_PER_RAD;
  float envelope = 1.0F;
  float envelope_rate = 0.0F;
  if (problem->run_ms < problem->config.amplitude_ramp_ms) {
    const float phase = Q2_PI * (float)problem->run_ms /
        (float)problem->config.amplitude_ramp_ms;
    envelope = 0.5F * (1.0F - cosf(phase));
    envelope_rate = 0.5F * Q2_PI * 1000.0F /
        (float)problem->config.amplitude_ramp_ms * sinf(phase);
  }
  problem->target_pitch_deg = amplitude * envelope * sinf(omega * seconds);
  problem->target_pitch_rate_dps = amplitude * (envelope_rate *
      sinf(omega * seconds) + envelope * omega * cosf(omega * seconds));
  const float theta = problem->target_pitch_deg * Q2_RAD_PER_DEG;
  problem->target_position_m = problem->config.ground_height_m * tanf(theta);
  problem->target_velocity_mps = problem->config.ground_height_m *
      problem->target_pitch_rate_dps * Q2_RAD_PER_DEG / (cosf(theta) * cosf(theta));
  const float applied = (float)problem->pwm[0] - (float)problem->pwm[1];
  const PendulumControl_Input input = {
      .target_deg = {0.0F, problem->target_pitch_deg},
      .target_rate_dps = {0.0F, problem->target_pitch_rate_dps},
      .angle_deg = {roll_deg, pitch_deg},
      .rate_dps = {roll_rate_dps, pitch_rate_dps},
      .dt_s = (float)elapsed_ms * 0.001F,
      .integrate = {false, fabsf(applied -
          problem->control_output.axis_output[PENDULUM_AXIS_PITCH]) <= 2.0F}};
  problem->pitch_integrating = input.integrate[PENDULUM_AXIS_PITCH];
  if (!PendulumControl_StepWithRates(&problem->controller, &input,
                                    &problem->control_output) ||
      !problem->control_output.pwm_ready) {
    ProblemQ2_Stop(problem, PROBLEM_Q2_INVALID_CONFIG);
    return;
  }
  if (-pitch_deg > problem->left_peak_deg) problem->left_peak_deg = -pitch_deg;
  if (pitch_deg > problem->right_peak_deg) problem->right_peak_deg = pitch_deg;
  ProblemQ2_Slew(problem, elapsed_ms);
}
