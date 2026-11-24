#include "problems/q1_swing.h"
#include "app_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void CheckOutput(const Q1Swing *swing)
{
  assert(swing->pwm[0] <= APP_Q1_PWM_MAX_PERMILLE);
  assert(swing->pwm[1] <= APP_Q1_PWM_MAX_PERMILLE);
  assert(swing->pwm[2] == 0U && swing->pwm[3] == 0U);
  assert(swing->pwm[0] == 0U || swing->pwm[1] == 0U);
}

static void CheckDirectOutput(const Q1Swing *swing)
{
  CheckOutput(swing);
  assert(swing->pwm[0] == swing->control_output.pwm_permille[0]);
  assert(swing->pwm[1] == swing->control_output.pwm_permille[1]);
}

static void TestReferenceAndPidPath(void)
{
  Q1Swing swing = {0};
  assert(Q1Swing_Start(&swing, 100U, 0.0F, 0.0F, 0.0F));
  assert(swing.state == Q1_SWING_RUNNING);
  assert(swing.reference.config.ground_height_m == APP_PIVOT_GROUND_HEIGHT_M);
  assert(swing.controller.config.disabled_axis_mask == PENDULUM_DISABLE_ROLL);

  Q1Swing_Step(&swing, 100U, 0.0F, 0.0F, 0.0F, true, false);
  assert(fabsf(swing.target_pitch_deg) < 0.001F);
  assert(swing.pwm[0] == 0U && swing.pwm[1] == 0U);

  for (uint32_t now = 102U; now <= 600U; now += 2U) {
    Q1Swing_Step(&swing, now, 0.0F, 0.0F, 0.0F, true, false);
    CheckDirectOutput(&swing);
  }
  assert(swing.target_pitch_deg > 0.0F);
  assert(swing.target_pitch_rate_dps != 0.0F);
  assert(swing.target_position_m > 0.0F);
  assert(swing.target_velocity_mps > 0.0F);
  assert(swing.control_output.axis_output[1] > 0.0F);
  assert(!swing.pitch_integrating);
  assert(swing.control_output.i[PENDULUM_AXIS_PITCH] == 0.0F);
  assert(swing.control_output.d[PENDULUM_AXIS_PITCH] == 0.0F);
  assert(swing.pwm[0] > 0U && swing.pwm[1] == 0U);

  bool saw_negative_reference = false;
  bool saw_opposite_output = false;
  for (uint32_t now = 602U; now <= 1800U; now += 2U) {
    Q1Swing_Step(&swing, now, 0.0F, 0.0F, 0.0F, true, false);
    CheckDirectOutput(&swing);
    if (swing.target_pitch_deg < 0.0F) saw_negative_reference = true;
    if (swing.pwm[1] > 0U) saw_opposite_output = true;
  }
  assert(saw_negative_reference);
  assert(saw_opposite_output);
}

static void TestDerivativeDampsMeasuredRate(void)
{
  Q1Swing swing = {0};
  const float pitch_rate_dps = 20.0F;
  const float dt_s = 0.002F;
  const float alpha = dt_s / (APP_Q1_RATE_FILTER_TAU_S + dt_s);
  assert(Q1Swing_Start(&swing, 100U, 0.0F, 0.0F, pitch_rate_dps));

  Q1Swing_Step(&swing, 102U, 0.0F, 0.0F, pitch_rate_dps, true, false);
  const float expected = -APP_Q1_PITCH_KD * alpha * pitch_rate_dps;
  assert(fabsf(swing.control_output.d[PENDULUM_AXIS_PITCH] - expected) < 0.001F);
}

static void TestSafetyAndTimeout(void)
{
  Q1Swing swing = {0};
  assert(!Q1Swing_Start(&swing, 0U, 0.0F, 7.01F, 0.0F));
  assert(swing.reason == Q1_SWING_NOT_CENTERED);

  assert(Q1Swing_Start(&swing, 0U, 0.0F, 0.0F, 0.0F));
  Q1Swing_Step(&swing, 21U, 0.0F, 0.0F, 0.0F, true, false);
  assert(swing.reason == Q1_SWING_SAMPLE_STALE && swing.pwm[0] == 0U);

  assert(Q1Swing_Start(&swing, 100U, 0.0F, 0.0F, 0.0F));
  Q1Swing_Step(&swing, 102U, 0.0F, 26.0F, 0.0F, true, false);
  assert(swing.reason == Q1_SWING_ANGLE_LIMIT && swing.pwm[0] == 0U);

  assert(Q1Swing_Start(&swing, 200U, 0.0F, 0.0F, 0.0F));
  Q1Swing_Step(&swing, 202U, 0.0F, 0.0F, 0.0F, true, true);
  assert(swing.reason == Q1_SWING_USER_STOP);

  const uint32_t started = UINT32_MAX - 500U;
  assert(Q1Swing_Start(&swing, started, 0.0F, 0.0F, 0.0F));
  assert(swing.reference.config.duration_ms == APP_Q1_DURATION_MS);
  for (uint32_t elapsed = 2U; elapsed < APP_Q1_DURATION_MS; elapsed += 2U) {
    Q1Swing_Step(&swing, started + elapsed, 0.0F, 0.0F, 0.0F, true, false);
  }
  assert(swing.state == Q1_SWING_RUNNING);
  Q1Swing_Step(&swing, started + APP_Q1_DURATION_MS,
               0.0F, 0.0F, 0.0F, true, false);
  assert(swing.state == Q1_SWING_FINISHED);
  assert(swing.reason == Q1_SWING_TIME_UP);
  CheckOutput(&swing);
  assert(swing.pwm[0] == 0U && swing.pwm[1] == 0U);
}

static void TestDutyLimitAndDirectionChange(void)
{
  Q1Swing swing = {0};
  assert(Q1Swing_Start(&swing, 0U, 0.0F, 0.0F, 0.0F));

  Q1Swing_Step(&swing, 20U, 0.0F, -24.0F, -140.0F, true, false);
  CheckDirectOutput(&swing);
  assert(swing.pwm[0] == APP_Q1_PWM_MAX_PERMILLE && swing.pwm[1] == 0U);

  Q1Swing_Step(&swing, 40U, 0.0F, 24.0F, 140.0F, true, false);
  CheckDirectOutput(&swing);
  assert(swing.pwm[0] == 0U && swing.pwm[1] > 0U);
  assert(swing.switches == 1U);

  Q1Swing_Step(&swing, 60U, 0.0F, 24.0F, 140.0F, true, false);
  CheckDirectOutput(&swing);
  assert(swing.pwm[1] == APP_Q1_PWM_MAX_PERMILLE);

  Q1Swing_Step(&swing, 62U, 0.0F, 0.0F, 0.0F, true, false);
  CheckDirectOutput(&swing);
  assert(swing.pwm[1] < APP_Q1_PWM_MAX_PERMILLE);

  Q1Swing_Step(&swing, 64U, 0.0F, 24.0F, 140.0F, true, true);
  assert(swing.reason == Q1_SWING_USER_STOP);
  assert(swing.pwm[0] == 0U && swing.pwm[1] == 0U);
}

int main(void)
{
  TestReferenceAndPidPath();
  TestDerivativeDampsMeasuredRate();
  TestSafetyAndTimeout();
  TestDutyLimitAndDirectionChange();
  puts("Q1 trajectory PID tests passed");
  return 0;
}
