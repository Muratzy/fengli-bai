#include "problems/q3_direction.h"
#include "app_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAD_PER_DEG 0.0174532925199F
#define Q3_START_RATE_DPS 5.0F

typedef struct {
  float position;
  float rate;
} Q3Projection;

static Q3Projection Q3Direction_Project(float up, float right, float y, float x,
                                       float dy, float dx)
{
  const float distance = up * y + right * x;
  return (Q3Projection){atanf(distance) / RAD_PER_DEG,
                        (up * dy + right * dx) /
                        ((1.0F + distance * distance) * RAD_PER_DEG)};
}

bool Q3Direction_StartTuned(Q3Direction *line, Q1Swing *along,
                            const ProblemParams *params,
                            const Q1Swing_Tuning *requested_tuning,
                            uint32_t now_ms, float roll_deg, float pitch_deg,
                            float roll_rate_dps, float pitch_rate_dps)
{
  if (line == NULL || along == NULL || along->state == Q1_SWING_RUNNING) return false;
  memset(line, 0, sizeof(*line));
  memset(along, 0, sizeof(*along));
  const ProblemParams defaults = ProblemApi_DefaultParams(PROBLEM_Q3);
  if (params == NULL) params = &defaults;
  if (ProblemApi_ValidateStart(PROBLEM_Q3, params) != PROBLEM_REQUEST_ACCEPTED) {
    Q1Swing_Stop(along, Q1_SWING_INVALID_CONFIG);
    return false;
  }
  if (!isfinite(roll_deg) || !isfinite(pitch_deg) ||
      !isfinite(roll_rate_dps) || !isfinite(pitch_rate_dps) ||
      hypotf(roll_deg, pitch_deg) > APP_Q1_START_ANGLE_DEG ||
      hypotf(roll_rate_dps, pitch_rate_dps) > Q3_START_RATE_DPS) {
    Q1Swing_Stop(along, Q1_SWING_NOT_CENTERED);
    return false;
  }

  const float direction = (float)params->direction_deg * RAD_PER_DEG;
  line->up = cosf(direction);
  line->right = sinf(direction);
  const float x = tanf(pitch_deg * RAD_PER_DEG) / cosf(roll_deg * RAD_PER_DEG);
  const float y = tanf(roll_deg * RAD_PER_DEG);
  const float angle = Q3Direction_Project(line->up, line->right, y, x, 0.0F, 0.0F).position;
  Q1Swing_Tuning tuning = requested_tuning != NULL ? *requested_tuning :
      Q1Swing_DefaultTuning();
  tuning.target_amplitude_deg = atanf((float)params->line_length_mm * 0.0005F /
      APP_PIVOT_GROUND_HEIGHT_M) / RAD_PER_DEG;
  tuning.duration_ms = APP_Q3_DURATION_MS;
  if (!Q1Swing_StartTuned(along, &tuning, now_ms, 0.0F, angle, 0.0F)) return false;
  if (!PID_Init(&line->cross_pid, &along->controller.config.pitch_pid)) {
    Q1Swing_Stop(along, Q1_SWING_INVALID_CONFIG);
    return false;
  }
  PID_Reset(&line->cross_pid,
      Q3Direction_Project(-line->right, line->up, y, x, 0.0F, 0.0F).position, 0.0F);
  return true;
}

bool Q3Direction_Start(Q3Direction *line, Q1Swing *along,
                       const ProblemParams *params, uint32_t now_ms,
                       float roll_deg, float pitch_deg,
                       float roll_rate_dps, float pitch_rate_dps)
{
  return Q3Direction_StartTuned(line, along, params, NULL, now_ms, roll_deg,
      pitch_deg, roll_rate_dps, pitch_rate_dps);
}

static void Q3Direction_Mix(Q3Direction *line, const PendulumControl_Output *along,
                            float cross_output)
{
  PendulumControl_Output *output = &line->output;
  const float signed_roll = line->up * along->axis_output[PENDULUM_AXIS_PITCH] -
      line->right * cross_output;
  const float signed_pitch = line->right * along->axis_output[PENDULUM_AXIS_PITCH] +
      line->up * cross_output;
  const float total = fabsf(signed_roll) + fabsf(signed_pitch);
  const float scale = total > APP_Q1_PWM_MAX_PERMILLE ?
      (float)APP_Q1_PWM_MAX_PERMILLE / total : 1.0F;
  const float roll = signed_roll * (float)APP_Q3_ROLL_SIGN;
  *output = (PendulumControl_Output){0};
  output->pwm_ready = true;
  output->axis_output[PENDULUM_AXIS_ROLL] = signed_roll * scale;
  output->axis_output[PENDULUM_AXIS_PITCH] = signed_pitch * scale;
  /* Truncate duty so rounding can never raise the total above the cap. */
  output->pwm_permille[signed_pitch >= 0.0F ? 0 : 1] =
      (uint16_t)(fabsf(signed_pitch) * scale);
  output->pwm_permille[roll >= 0.0F ? 2 : 3] = (uint16_t)(fabsf(roll) * scale);
  const float p = along->p[PENDULUM_AXIS_PITCH];
  const float i = along->i[PENDULUM_AXIS_PITCH];
  const float d = along->d[PENDULUM_AXIS_PITCH];
  output->p[PENDULUM_AXIS_ROLL] = line->up * p - line->right * line->cross_pid.p_term;
  output->p[PENDULUM_AXIS_PITCH] = line->right * p + line->up * line->cross_pid.p_term;
  output->i[PENDULUM_AXIS_ROLL] = line->up * i;
  output->i[PENDULUM_AXIS_PITCH] = line->right * i;
  output->d[PENDULUM_AXIS_ROLL] = line->up * d - line->right * line->cross_pid.d_term;
  output->d[PENDULUM_AXIS_PITCH] = line->right * d + line->up * line->cross_pid.d_term;
}

void Q3Direction_Step(Q3Direction *line, Q1Swing *along, uint32_t now_ms,
                      float roll_deg, float pitch_deg,
                      float roll_rate_dps, float pitch_rate_dps)
{
  if (line == NULL || along == NULL) return;
  if (along->state != Q1_SWING_RUNNING) {
    line->output = (PendulumControl_Output){0};
    return;
  }
  if (!isfinite(roll_deg) || !isfinite(pitch_deg) ||
      !isfinite(roll_rate_dps) || !isfinite(pitch_rate_dps)) {
    Q1Swing_Stop(along, Q1_SWING_SAMPLE_STALE);
  } else if (hypotf(roll_deg, pitch_deg) > APP_Q1_STOP_PITCH_DEG ||
      hypotf(roll_rate_dps, pitch_rate_dps) > APP_Q1_STOP_RATE_DPS) {
    Q1Swing_Stop(along, Q1_SWING_ANGLE_LIMIT);
  }
  if (along->state != Q1_SWING_RUNNING) {
    line->output = (PendulumControl_Output){0};
    return;
  }

  const uint32_t elapsed_ms = now_ms - along->last_step_ms;
  const float roll = roll_deg * RAD_PER_DEG;
  const float pitch = pitch_deg * RAD_PER_DEG;
  const float cos_roll = cosf(roll);
  const float cos_pitch = cosf(pitch);
  const float roll_rate = roll_rate_dps * RAD_PER_DEG;
  const float pitch_rate = pitch_rate_dps * RAD_PER_DEG;
  /* Ground x/y divided by pivot height, from the project's IMU convention. */
  const float x = tanf(pitch) / cos_roll;
  const float y = tanf(roll);
  const float dx = pitch_rate / (cos_roll * cos_pitch * cos_pitch) +
      x * y * roll_rate;
  const float dy = roll_rate / (cos_roll * cos_roll);
  const Q3Projection axial = Q3Direction_Project(line->up, line->right, y, x, dy, dx);
  Q1Swing_Step(along, now_ms, 0.0F, axial.position, axial.rate, true, false);
  if (along->state != Q1_SWING_RUNNING) {
    line->output = (PendulumControl_Output){0};
    return;
  }
  if (elapsed_ms == 0U) return;

  const float target = along->target_position_m / APP_PIVOT_GROUND_HEIGHT_M;
  const float velocity = along->target_velocity_mps / APP_PIVOT_GROUND_HEIGHT_M;
  const float target_y = line->up * target;
  const float target_x = line->right * target;
  const float target_cos_roll = 1.0F / sqrtf(1.0F + target_y * target_y);
  const float target_tan_pitch = target_x * target_cos_roll;
  line->target_roll_deg = atanf(target_y) / RAD_PER_DEG;
  line->target_pitch_deg = atanf(target_tan_pitch) / RAD_PER_DEG;
  line->target_pitch_rate_dps = line->right * velocity * target_cos_roll *
      target_cos_roll * target_cos_roll /
      (RAD_PER_DEG * (1.0F + target_tan_pitch * target_tan_pitch));

  const Q3Projection cross = Q3Direction_Project(-line->right, line->up, y, x, dy, dx);
  const float cross_output = PID_UpdateWithRate(&line->cross_pid, 0.0F, cross.position,
      0.0F, cross.rate, (float)elapsed_ms * 0.001F, false);
  Q3Direction_Mix(line, &along->control_output, cross_output);
}
