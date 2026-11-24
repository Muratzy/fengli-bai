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

  uint16_t previous = 0U;
  for (uint32_t now = 102U; now <= 600U; now += 2U) {
    Q1Swing_Step(&swing, now, 0.0F, 0.0F, 0.0F, true, false);
    CheckOutput(&swing);
    assert(swing.pwm[0] >= previous || swing.pwm[1] > 0U);
    previous = swing.pwm[0];
  }
  assert(swing.target_pitch_deg > 0.0F);
  assert(swing.target_pitch_rate_dps != 0.0F);
  assert(swing.target_position_m > 0.0F);
  assert(swing.target_velocity_mps > 0.0F);
  assert(swing.control_output.axis_output[1] > 0.0F);
  assert(swing.pwm[0] > 0U && swing.pwm[1] == 0U);

  bool saw_negative_reference = false;
  bool saw_zero_break = false;
  for (uint32_t now = 602U; now <= 1800U; now += 2U) {
    const uint16_t old_left = swing.pwm[0];
    Q1Swing_Step(&swing, now, 0.0F, 0.0F, 0.0F, true, false);
    CheckOutput(&swing);
    if (swing.target_pitch_deg < 0.0F) saw_negative_reference = true;
    if (old_left > 0U && swing.pwm[0] == 0U && swing.pwm[1] == 0U) {
      saw_zero_break = true;
    }
  }
  assert(saw_negative_reference);
  assert(saw_zero_break || swing.pwm[1] > 0U);
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

  for (uint32_t now = 10U; now <= 300U; now += 10U) {
    Q1Swing_Step(&swing, now, 0.0F, -24.0F, -140.0F, true, false);
    CheckOutput(&swing);
    assert(swing.control_output.pwm_permille[0] == 800U);
    assert(swing.pwm[0] == (uint16_t)((800U * now + 150U) / 300U));
  }

  uint32_t now = 300U;
  do {
    now += 10U;
    Q1Swing_Step(&swing, now, 0.0F, 24.0F, 140.0F, true, false);
    CheckOutput(&swing);
    assert(now <= 400U);
  } while (swing.control_output.pwm_permille[1] == 0U);
  assert(swing.pwm[0] > 0U && swing.pwm[1] == 0U);

  do {
    const uint16_t previous = swing.pwm[0];
    now += 2U;
    Q1Swing_Step(&swing, now, 0.0F, 24.0F, 140.0F, true, false);
    assert(swing.pwm[0] == (previous > 16U ? previous - 16U : 0U));
    assert(swing.pwm[1] == 0U);
  } while (swing.pwm[0] > 0U);

  const uint32_t zero_since = now;
  for (uint32_t elapsed = 2U; elapsed < 10U; elapsed += 2U) {
    Q1Swing_Step(&swing, zero_since + elapsed, 0.0F, 24.0F, 140.0F, true, false);
    assert(swing.pwm[0] == 0U && swing.pwm[1] == 0U);
    assert(!swing.pitch_integrating);
  }
  Q1Swing_Step(&swing, zero_since + 10U, 0.0F, 24.0F, 140.0F, true, false);
  assert(swing.pwm[0] == 0U && swing.pwm[1] == 5U);
  assert(swing.switches == 1U);

  Q1Swing_Step(&swing, zero_since + 12U, 0.0F, 24.0F, 140.0F, true, true);
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
