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

  assert(Q1Swing_Start(&swing, UINT32_MAX - 500U, 0.0F, 0.0F, 0.0F));
  for (uint32_t elapsed = 2U; elapsed < 15000U; elapsed += 2U) {
    Q1Swing_Step(&swing, (UINT32_MAX - 500U) + elapsed,
                 0.0F, 0.0F, 0.0F, true, false);
  }
  assert(swing.state == Q1_SWING_RUNNING);
  Q1Swing_Step(&swing, (UINT32_MAX - 500U) + 15000U,
               0.0F, 0.0F, 0.0F, true, false);
  assert(swing.state == Q1_SWING_FINISHED);
  assert(swing.reason == Q1_SWING_TIME_UP);
  CheckOutput(&swing);
}

static void TestDutyLimitAndDirectionChange(void)
{
  Q1Swing swing = {0};
  assert(Q1Swing_Start(&swing, 0U, 0.0F, 0.0F, 0.0F));

  Q1Swing_Step(&swing, 2U, 0.0F, -24.0F, -140.0F, true, false);
  CheckDirectOutput(&swing);
  assert(swing.pwm[0] == APP_Q1_PWM_MAX_PERMILLE && swing.pwm[1] == 0U);

  Q1Swing_Step(&swing, 4U, 0.0F, 24.0F, 140.0F, true, false);
  CheckDirectOutput(&swing);
  assert(swing.pwm[0] == 0U && swing.pwm[1] == APP_Q1_PWM_MAX_PERMILLE);
  assert(swing.switches == 1U);

  Q1Swing_Step(&swing, 6U, 0.0F, 0.0F, 0.0F, true, false);
  CheckDirectOutput(&swing);
  assert(swing.pwm[1] < APP_Q1_PWM_MAX_PERMILLE);

  Q1Swing_Step(&swing, 8U, 0.0F, 24.0F, 140.0F, true, true);
  assert(swing.reason == Q1_SWING_USER_STOP);
  assert(swing.pwm[0] == 0U && swing.pwm[1] == 0U);
}

int main(void)
{
  TestReferenceAndPidPath();
  TestSafetyAndTimeout();
  TestDutyLimitAndDirectionChange();
  puts("Q1 trajectory PID tests passed");
  return 0;
}
