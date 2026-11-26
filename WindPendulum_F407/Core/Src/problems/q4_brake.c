#include "problems/q4_brake.h"
#include "app_config.h"

#include <math.h>
#include <string.h>

static bool ValidSample(const Q4Sample *sample)
{
  return sample != NULL && isfinite(sample->roll_deg) && isfinite(sample->pitch_deg) &&
      isfinite(sample->tilt_deg) && sample->tilt_deg >= 0.0F &&
      isfinite(sample->roll_rate_dps) && isfinite(sample->pitch_rate_dps);
}

void Q4Brake_Stop(Q4Brake *brake, Q1Swing_Reason reason)
{
  brake->state = reason == Q4_BRAKE_SETTLED ? PROBLEM_STATUS_FINISHED :
      reason == Q1_SWING_USER_STOP ? PROBLEM_STATUS_STOPPED : PROBLEM_STATUS_FAULT;
  brake->reason = reason;
  memset(&brake->output, 0, sizeof(brake->output));
}

bool Q4Brake_Arm(Q4Brake *brake, uint32_t now_ms, const Q4Sample *sample)
{
  if (brake->state == PROBLEM_STATUS_RUNNING || !ValidSample(sample) ||
      sample->tilt_deg > APP_Q4_RELEASE_MAX_DEG) return false;
  memset(brake, 0, sizeof(*brake));
  const PID_Config pd = {
      .kp = APP_Q4_KP, .kd = APP_Q4_KD, .proportional_setpoint_weight = 1.0F,
      .derivative_filter_tau_s = APP_Q1_RATE_FILTER_TAU_S,
      .output_min = -(float)APP_Q1_PWM_MAX_PERMILLE,
      .output_max = APP_Q1_PWM_MAX_PERMILLE};
  const float roll_sign = (float)APP_Q3_ROLL_SIGN;
  const PendulumControl_Config config = {
      .roll_pid = pd, .pitch_pid = pd,
      .fan_mix = {{0.0F, 1.0F}, {0.0F, -1.0F},
                  {roll_sign, 0.0F}, {-roll_sign, 0.0F}},
      .max_duty_permille = APP_Q1_PWM_MAX_PERMILLE,
      .mixer_calibrated = APP_Q3_ROLL_SIGN != 0};
  if (!config.mixer_calibrated || !PendulumControl_Init(&brake->controller, &config)) {
    Q4Brake_Stop(brake, Q1_SWING_INVALID_CONFIG);
    return false;
  }
  brake->armed_ms = now_ms;
  brake->last_step_ms = now_ms;
  brake->state = PROBLEM_STATUS_RUNNING;
  return true;
}

static bool DetectRelease(Q4Brake *brake, uint32_t now_ms, const Q4Sample *sample, float speed)
{
  const bool in_band = sample->tilt_deg >= APP_Q4_RELEASE_MIN_DEG &&
      sample->tilt_deg <= APP_Q4_RELEASE_MAX_DEG;
  if (speed <= APP_Q4_SETTLE_RATE_DPS && in_band) {
    if (!brake->holding) brake->hold_ms = now_ms;
    brake->holding = true;
    return false;
  }
  if (!brake->holding || now_ms - brake->hold_ms < APP_Q4_HOLD_MS ||
      sample->tilt_deg < APP_Q4_RELEASE_MIN_DEG - 2.0F ||
      sample->tilt_deg > APP_Q4_RELEASE_MAX_DEG) {
    brake->holding = false;
    return false;
  }

  /* cos(tilt) = cos(roll) cos(pitch); compare its derivative to the inward
   * speed threshold without dividing by sin(tilt). */
  const float rad_per_deg = 0.01745329252F;
  const float roll = sample->roll_deg * rad_per_deg;
  const float pitch = sample->pitch_deg * rad_per_deg;
  const float radial_numerator = sinf(roll) * cosf(pitch) * sample->roll_rate_dps +
      cosf(roll) * sinf(pitch) * sample->pitch_rate_dps;
  brake->holding = radial_numerator < 0.0F;
  return radial_numerator <= -APP_Q4_RELEASE_RATE_DPS * sinf(sample->tilt_deg * rad_per_deg);
}

void Q4Brake_Step(Q4Brake *brake, uint32_t now_ms, const Q4Sample *sample)
{
  if (brake->state != PROBLEM_STATUS_RUNNING) return;
  const uint32_t step_ms = now_ms - brake->last_step_ms;
  if (!ValidSample(sample) || step_ms > 20U) {
    Q4Brake_Stop(brake, Q1_SWING_SAMPLE_STALE);
    return;
  }
  brake->last_step_ms = now_ms;
  const float speed = hypotf(sample->roll_rate_dps, sample->pitch_rate_dps);
  if (sample->tilt_deg > APP_Q4_STOP_ANGLE_DEG || speed > APP_Q4_STOP_RATE_DPS) {
    Q4Brake_Stop(brake, Q1_SWING_ANGLE_LIMIT);
    return;
  }
  if (brake->phase == Q4_WAIT_RELEASE) {
    if (now_ms - brake->armed_ms >= APP_Q4_ARM_TIMEOUT_MS) {
      Q4Brake_Stop(brake, Q4_BRAKE_NO_RELEASE);
      return;
    }
    if (!DetectRelease(brake, now_ms, sample, speed)) return;
    brake->phase = Q4_BRAKING;
    brake->released_ms = now_ms;
  }
  brake->run_ms = now_ms - brake->released_ms;

  const bool quiet = sample->tilt_deg <= APP_Q4_SETTLE_ANGLE_DEG &&
      speed <= APP_Q4_SETTLE_RATE_DPS;
  if (quiet) {
    if (brake->phase != Q4_VERIFY_STOP) {
      brake->phase = Q4_VERIFY_STOP;
      brake->settle_ms = now_ms;
      memset(&brake->output, 0, sizeof(brake->output));
      PendulumControl_Reset(&brake->controller, sample->roll_deg, sample->pitch_deg);
    }
    if (now_ms - brake->settle_ms >= APP_Q4_SETTLE_MS &&
        brake->run_ms <= APP_Q4_DURATION_MS) {
      Q4Brake_Stop(brake, Q4_BRAKE_SETTLED);
      return;
    }
  } else {
    brake->phase = Q4_BRAKING;
  }
  if (brake->run_ms >= APP_Q4_DURATION_MS) {
    Q4Brake_Stop(brake, Q4_BRAKE_NOT_SETTLED);
    return;
  }
  if (quiet || step_ms == 0U) return;

  const PendulumControl_Input input = {
      .angle_deg = {sample->roll_deg, sample->pitch_deg},
      .rate_dps = {sample->roll_rate_dps, sample->pitch_rate_dps},
      .dt_s = (float)step_ms * 0.001F};
  if (!PendulumControl_StepWithRates(&brake->controller, &input, &brake->output)) {
    Q4Brake_Stop(brake, Q1_SWING_INVALID_CONFIG);
    return;
  }
  uint32_t total = 0U;
  for (unsigned i = 0U; i < PENDULUM_FAN_COUNT; ++i)
    total += brake->output.pwm_permille[i];
  if (total > APP_Q1_PWM_MAX_PERMILLE) {
    for (unsigned i = 0U; i < PENDULUM_FAN_COUNT; ++i)
      brake->output.pwm_permille[i] =
          brake->output.pwm_permille[i] * APP_Q1_PWM_MAX_PERMILLE / total;
  }
}
