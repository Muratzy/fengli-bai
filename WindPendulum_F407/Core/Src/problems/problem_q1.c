#include "problems/problem_q1.h"

#include <math.h>
#include <stddef.h>

#define PROBLEM_Q1_MIN_LENGTH_M       0.50F
#define PROBLEM_Q1_MAX_DURATION_MS    15000U
#define PROBLEM_Q1_DEG_PER_RAD        57.2957795131F
#define PROBLEM_Q1_TWO_PI             6.2831853072F

bool ProblemQ1_Validate(const ProblemQ1_Config *config)
{
  float required_angle_deg;

  if ((config == NULL) ||
      !isfinite(config->line_length_m) ||
      !isfinite(config->effective_length_m) ||
      !isfinite(config->max_angle_deg) ||
      !isfinite(config->direction_sign) ||
      (config->line_length_m < PROBLEM_Q1_MIN_LENGTH_M) ||
      (config->effective_length_m <= 0.0F) ||
      (config->max_angle_deg <= 0.0F) ||
      (config->max_angle_deg >= 89.0F) ||
      ((config->direction_sign != 1.0F) &&
       (config->direction_sign != -1.0F)) ||
      (config->duration_ms == 0U) ||
      (config->duration_ms > PROBLEM_Q1_MAX_DURATION_MS) ||
      ((config->axis != PROBLEM_Q1_AXIS_ROLL) &&
       (config->axis != PROBLEM_Q1_AXIS_PITCH))) {
    return false;
  }

  required_angle_deg = atan2f(config->line_length_m,
                              config->effective_length_m) *
                      PROBLEM_Q1_DEG_PER_RAD;
  return required_angle_deg <= config->max_angle_deg;
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

  phase = PROBLEM_Q1_TWO_PI * (float)elapsed_ms /
          (float)problem->config.duration_ms;
  output->target_position_m =
      0.5F * problem->config.line_length_m * (1.0F - cosf(phase)) *
      problem->config.direction_sign;
  output->target_velocity_mps =
      0.5F * problem->config.line_length_m * PROBLEM_Q1_TWO_PI *
      sinf(phase) * 1000.0F / (float)problem->config.duration_ms *
      problem->config.direction_sign;
  target_angle_deg = atan2f(output->target_position_m,
                            problem->config.effective_length_m) *
                     PROBLEM_Q1_DEG_PER_RAD;
  if (problem->config.axis == PROBLEM_Q1_AXIS_ROLL) {
    output->target_roll_deg = target_angle_deg;
  } else {
    output->target_pitch_deg = target_angle_deg;
  }
  return true;
}

void ProblemQ1_Cancel(ProblemQ1 *problem)
{
  if ((problem != NULL) && (problem->state == PROBLEM_Q1_RUNNING)) {
    problem->state = PROBLEM_Q1_ABORTED;
  }
}
