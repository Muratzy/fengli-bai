#include "problems/problem_q1.h"
#include "app_config.h"

#include <math.h>
#include <stddef.h>

#define PROBLEM_Q1_MIN_LENGTH_M       0.50F
#define PROBLEM_Q1_DEG_PER_RAD        57.2957795131F
#define PROBLEM_Q1_PI                 3.1415926536F
#define PROBLEM_Q1_TWO_PI             6.2831853072F

ProblemQ1_Config ProblemQ1_DefaultConfig(void)
{
  const ProblemQ1_Config config = {
      .line_length_m = APP_Q1_LINE_LENGTH_M,
      .ground_height_m = APP_PIVOT_GROUND_HEIGHT_M,
      .max_angle_deg = 20.0F,
      .direction_sign = 1.0F,
      .swing_period_ms = APP_Q1_PERIOD_MS,
      .startup_ramp_ms = 2U * APP_Q1_PERIOD_MS,
      .duration_ms = APP_Q1_DURATION_MS,
      .axis = PROBLEM_Q1_AXIS_PITCH};
  return config;
}

float ProblemQ1_TargetAmplitudeDeg(const ProblemQ1_Config *config)
{
  if (config->target_amplitude_deg > 0.0F) return config->target_amplitude_deg;
  return atan2f(0.5F * config->line_length_m, config->ground_height_m) *
      PROBLEM_Q1_DEG_PER_RAD;
}

bool ProblemQ1_Validate(const ProblemQ1_Config *config)
{


  if ((config == NULL) ||
      !isfinite(config->line_length_m) ||
      !isfinite(config->ground_height_m) ||
      !isfinite(config->max_angle_deg) ||
      !isfinite(config->direction_sign) ||
      !isfinite(config->target_amplitude_deg) ||
      (config->target_amplitude_deg < 0.0F) ||
      (config->line_length_m <= 0.0F) ||
      (config->target_amplitude_deg == 0.0F &&
       config->line_length_m < PROBLEM_Q1_MIN_LENGTH_M) ||
      (config->ground_height_m <= 0.0F) ||
      (config->max_angle_deg <= 0.0F) ||
      (config->max_angle_deg >= 89.0F) ||
      ((config->direction_sign != 1.0F) &&
       (config->direction_sign != -1.0F)) ||
      (config->duration_ms == 0U) ||
      (config->duration_ms > APP_Q1_MAX_DURATION_MS) ||
      (config->swing_period_ms == 0U) ||
      (config->swing_period_ms > config->duration_ms) ||
      (config->startup_ramp_ms == 0U) ||
      (config->startup_ramp_ms >
       config->duration_ms - config->swing_period_ms) ||
      ((config->axis != PROBLEM_Q1_AXIS_ROLL) &&
       (config->axis != PROBLEM_Q1_AXIS_PITCH))) {
    return false;
  }

  return ProblemQ1_TargetAmplitudeDeg(config) <= config->max_angle_deg;
}

ProblemQ1_Result ProblemQ1_Start(ProblemQ1 *problem,
                                 const ProblemQ1_Config *config,
                                 uint32_t now_ms, bool imu_online,
                                 bool calibration_complete)
{
  if (problem == NULL) {
    return PROBLEM_Q1_INVALID_CONFIG;
  }
  if (!ProblemQ1_Validate(config)) {
    problem->state = PROBLEM_Q1_BLOCKED;
    return PROBLEM_Q1_INVALID_CONFIG;
  }
  if (!imu_online) {
    problem->state = PROBLEM_Q1_BLOCKED;
    return PROBLEM_Q1_IMU_OFFLINE;
  }
  if (!calibration_complete) {
    problem->state = PROBLEM_Q1_BLOCKED;
    return PROBLEM_Q1_NOT_CALIBRATED;
  }

  problem->config = *config;
  problem->start_time_ms = now_ms;
  problem->state = PROBLEM_Q1_RUNNING;
  return PROBLEM_Q1_OK;
}

bool ProblemQ1_Step(ProblemQ1 *problem, uint32_t now_ms,
                    ProblemQ1_Output *output)
{
  float phase;
  float amplitude_rad;
  float envelope;
  float envelope_rate;
  float angular_rate;
  float angle_rad;
  float angle_rate;
  float target_angle_deg;
  uint32_t elapsed_ms;

  if (output == NULL) {
    return false;
  }
  *output = (ProblemQ1_Output){0};
  if ((problem == NULL) ||
      ((problem->state != PROBLEM_Q1_RUNNING) &&
       (problem->state != PROBLEM_Q1_COMPLETE))) {
    return false;
  }

  elapsed_ms = now_ms - problem->start_time_ms;
  if (elapsed_ms >= problem->config.duration_ms) {
    problem->state = PROBLEM_Q1_COMPLETE;
    output->complete = true;
    return true;
  }

  angular_rate = PROBLEM_Q1_TWO_PI * 1000.0F /
                 (float)problem->config.swing_period_ms;
  phase = angular_rate * (float)elapsed_ms * 0.001F;
  amplitude_rad = ProblemQ1_TargetAmplitudeDeg(&problem->config) /
      PROBLEM_Q1_DEG_PER_RAD;
  if (elapsed_ms < problem->config.startup_ramp_ms)
  {
    const float ramp_phase = PROBLEM_Q1_PI * (float)elapsed_ms /
                              (float)problem->config.startup_ramp_ms;
    envelope = 0.5F * (1.0F - cosf(ramp_phase));
    envelope_rate = 0.5F * PROBLEM_Q1_PI * 1000.0F /
        (float)problem->config.startup_ramp_ms * sinf(ramp_phase);
  }
  else
  {
    envelope = 1.0F;
    envelope_rate = 0.0F;
  }
  angle_rad = problem->config.direction_sign * amplitude_rad *
               envelope * sinf(phase);
  angle_rate = problem->config.direction_sign * amplitude_rad *
      (envelope_rate * sinf(phase) + envelope * angular_rate * cosf(phase));
  output->target_position_m = problem->config.ground_height_m * tanf(angle_rad);
  output->target_velocity_mps = problem->config.ground_height_m * angle_rate /
      (cosf(angle_rad) * cosf(angle_rad));
  target_angle_deg = angle_rad * PROBLEM_Q1_DEG_PER_RAD;
  if (problem->config.axis == PROBLEM_Q1_AXIS_ROLL) {
    output->target_roll_deg = target_angle_deg;
    output->target_roll_rate_dps = angle_rate * PROBLEM_Q1_DEG_PER_RAD;
  } else {
    output->target_pitch_deg = target_angle_deg;
    output->target_pitch_rate_dps = angle_rate * PROBLEM_Q1_DEG_PER_RAD;
  }
  return true;
}

void ProblemQ1_Cancel(ProblemQ1 *problem)
{
  if ((problem != NULL) && (problem->state == PROBLEM_Q1_RUNNING)) {
    problem->state = PROBLEM_Q1_ABORTED;
  }
}
