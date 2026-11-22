#include "problems/q1_swing.h"
#include "app_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SWING_RAD_PER_DEG 0.0174532925199433F
#define SWING_DEG_PER_RAD 57.2957795130823F

static uint16_t RampDown(uint16_t duty, uint32_t elapsed_ms)
{
  const uint32_t decrement =
      (APP_Q1_PWM_MAX_PERMILLE * elapsed_ms + APP_Q1_PWM_FALL_MS - 1U) /
      APP_Q1_PWM_FALL_MS;
  return (decrement >= duty) ? 0U : (uint16_t)(duty - decrement);
}

static void UpdatePwm(Q1Swing *swing, uint32_t elapsed_ms)
{
  /* Left motor PWM1 moves the pendulum right; PWM2 moves it left.
   * Finish ramping the old side to zero before raising the new side. */
  const uint32_t channel = (swing->direction > 0) ? 0U : 1U;
  const uint32_t other = channel ^ 1U;
  if (!swing->boosting) {
    swing->pwm[0] = RampDown(swing->pwm[0], elapsed_ms);
    swing->pwm[1] = RampDown(swing->pwm[1], elapsed_ms);
    swing->rise_remainder = 0U;
  } else if (swing->pwm[other] != 0U) {
    swing->pwm[other] = RampDown(swing->pwm[other], elapsed_ms);
    swing->pwm[channel] = 0U;
    swing->rise_remainder = 0U;
  } else {
    if (APP_Q1_PWM_START_PERMILLE > 0U && swing->pwm[channel] == 0U) {
      swing->pwm[channel] = APP_Q1_PWM_START_PERMILLE;
      swing->rise_remainder = 0U;
    } else {
      const uint32_t scaled = (APP_Q1_PWM_MAX_PERMILLE -
          APP_Q1_PWM_START_PERMILLE) * elapsed_ms + swing->rise_remainder;
      const uint32_t duty = swing->pwm[channel] + scaled / APP_Q1_PWM_RISE_MS;
      swing->rise_remainder = scaled % APP_Q1_PWM_RISE_MS;
      swing->pwm[channel] = (duty >= APP_Q1_PWM_MAX_PERMILLE) ?
          APP_Q1_PWM_MAX_PERMILLE : (uint16_t)duty;
      if (duty >= APP_Q1_PWM_MAX_PERMILLE) swing->rise_remainder = 0U;
    }
  }
  swing->pwm[2] = 0U;
  swing->pwm[3] = 0U;
}

void Q1Swing_Stop(Q1Swing *swing, Q1Swing_Reason reason)
{
  if (swing == NULL) return;
  swing->reason = reason;
  swing->state = (reason == Q1_SWING_TIME_UP) ? Q1_SWING_FINISHED :
      (reason == Q1_SWING_USER_STOP) ? Q1_SWING_STOPPED : Q1_SWING_FAULT;
  swing->boosting = false;
  swing->rise_remainder = 0U;
  memset(swing->pwm, 0, sizeof(swing->pwm));
}

bool Q1Swing_Start(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                    float pitch_deg, float pitch_rate_dps)
{
  if (swing == NULL || swing->state == Q1_SWING_RUNNING) return false;
  *swing = (Q1Swing){0};
  if (!isfinite(roll_deg) || !isfinite(pitch_deg) || !isfinite(pitch_rate_dps) ||
      fabsf(roll_deg) > APP_Q1_START_ANGLE_DEG ||
      fabsf(pitch_deg) > APP_Q1_START_ANGLE_DEG) {
    Q1Swing_Stop(swing, Q1_SWING_NOT_CENTERED);
    return false;
  }
  swing->state = Q1_SWING_RUNNING;
  swing->direction = 1;
  swing->boosting = true;
  swing->started_ms = now_ms;
  swing->last_step_ms = now_ms;
  swing->last_turn_ms = now_ms;
  /* No PWM until the first validated Step. */
  return true;
}

void Q1Swing_Step(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                   float pitch_deg, float pitch_rate_dps, bool sample_valid,
                   bool stop_requested)
{
  if (swing == NULL || swing->state != Q1_SWING_RUNNING) return;
  if (stop_requested) {
    Q1Swing_Stop(swing, Q1_SWING_USER_STOP);
    return;
  }
  if (!sample_valid || !isfinite(roll_deg) || !isfinite(pitch_deg) ||
      !isfinite(pitch_rate_dps) || now_ms - swing->last_step_ms > 20U) {
    Q1Swing_Stop(swing, Q1_SWING_SAMPLE_STALE);
    return;
  }
  const uint32_t elapsed_ms = now_ms - swing->last_step_ms;
  swing->last_step_ms = now_ms;
  if (fabsf(pitch_deg) > APP_Q1_STOP_PITCH_DEG ||
      fabsf(roll_deg) > APP_Q1_STOP_ROLL_DEG || fabsf(pitch_rate_dps) > 150.0F) {
    Q1Swing_Stop(swing, Q1_SWING_ANGLE_LIMIT);
    return;
  }
  if (now_ms - swing->started_ms >= APP_Q1_DURATION_MS) {
    Q1Swing_Stop(swing, Q1_SWING_TIME_UP);
    return;
  }
  if (now_ms - swing->last_turn_ms > 2500U) {
    Q1Swing_Stop(swing, Q1_SWING_NO_TURN);
    return;
  }
  const float direction = (float)swing->direction;
  if (direction * pitch_deg > swing->peak_deg) swing->peak_deg = direction * pitch_deg;
  if (direction * pitch_rate_dps > 5.0F) swing->turn_armed = true;
  /* A minimum excursion plus outward motion prevents false turns at center.
   * The 2 dps threshold intentionally switches just before zero velocity. */
  if (swing->turn_armed && direction * pitch_deg > 2.0F &&
      direction * pitch_rate_dps <= APP_Q1_TURN_RATE_DPS &&
      now_ms - swing->last_turn_ms >= 150U) {
    if (swing->direction > 0) swing->right_peak_deg = swing->peak_deg;
    else swing->left_peak_deg = swing->peak_deg;
    swing->direction = -swing->direction;
    swing->switches++;
    swing->turn_armed = false;
    swing->peak_deg = 0.0F;
    swing->last_turn_ms = now_ms;
  }
  /* Estimate free-swing energy using the measured center distance as a
   * point-mass approximation. Once enough energy is present, coast rather
   * than keep pumping at 100%; this estimate must be checked on the bench. */
  const float theta = pitch_deg * SWING_RAD_PER_DEG;
  const float omega = pitch_rate_dps * SWING_RAD_PER_DEG;
  float peak_cos = cosf(theta) - omega * omega *
      APP_PIVOT_PENDULUM_CENTER_M / (2.0F * 9.80665F);
  if (peak_cos < -1.0F) peak_cos = -1.0F;
  if (peak_cos > 1.0F) peak_cos = 1.0F;
  swing->predicted_peak_deg = acosf(peak_cos) * SWING_DEG_PER_RAD;
  const float target_deg = atan2f(0.5F * APP_Q1_LINE_LENGTH_M,
      APP_PIVOT_GROUND_HEIGHT_M) * SWING_DEG_PER_RAD;
  if (swing->predicted_peak_deg >= target_deg) swing->boosting = false;
  else if (swing->predicted_peak_deg < target_deg - 0.5F) swing->boosting = true;
  UpdatePwm(swing, elapsed_ms);
}
