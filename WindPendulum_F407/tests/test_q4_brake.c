#include "problems/q4_brake.h"
#include "app_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const Q4Sample center = {0};
static const Q4Sample held = {.pitch_deg = 30.0F, .tilt_deg = 30.0F};
static const Q4Sample released = {.pitch_deg = 29.7F, .tilt_deg = 29.7F, .pitch_rate_dps = -12.0F};

static unsigned TotalDuty(const Q4Brake *brake)
{
  const uint16_t *pwm = brake->output.pwm_permille;
  return pwm[0] + pwm[1] + pwm[2] + pwm[3];
}

static void HoldForRelease(Q4Brake *brake, uint32_t now)
{
  assert(Q4Brake_Arm(brake, now, &center));
  for (uint32_t elapsed = 0; elapsed <= APP_Q4_HOLD_MS; elapsed += 2U) {
    Q4Brake_Step(brake, now + elapsed, &held);
    assert(brake->phase == Q4_WAIT_RELEASE && brake->run_ms == 0U && TotalDuty(brake) == 0U);
  }
}

static void TestRelease(void)
{
  Q4Brake brake = {0};
  HoldForRelease(&brake, 2000U);
  uint32_t now = 2000U + APP_Q4_HOLD_MS + 2U;
  Q4Sample outward = held;
  outward.pitch_rate_dps = 16.0F;
  Q4Brake_Step(&brake, now, &outward);
  Q4Brake_Step(&brake, now + 2U, &released);
  assert(brake.phase == Q4_WAIT_RELEASE && TotalDuty(&brake) == 0U);
  Q4Brake_Step(&brake, now + 4U, &held);
  Q4Brake_Step(&brake, now + 6U, &released);
  assert(brake.phase == Q4_WAIT_RELEASE);
  Q4Brake_Stop(&brake, Q1_SWING_USER_STOP);
  assert(brake.state == PROBLEM_STATUS_STOPPED && TotalDuty(&brake) == 0U);

  HoldForRelease(&brake, 3000U);
  now = 3000U + APP_Q4_HOLD_MS + 2U;
  assert(!Q4Brake_Arm(&brake, now, &held));
  Q4Brake_Step(&brake, now, &released);
  assert(brake.phase == Q4_BRAKING && brake.released_ms == now && brake.run_ms == 0U);
  assert(brake.output.pwm_permille[0] == 0U && brake.output.pwm_permille[1] > 0U);
  assert(brake.output.i[PENDULUM_AXIS_PITCH] == 0.0F);
  const Q4Sample diagonal = {.roll_deg = 30.0F, .pitch_deg = 30.0F, .tilt_deg = 41.41F,
      .roll_rate_dps = 150.0F, .pitch_rate_dps = 150.0F};
  Q4Brake_Step(&brake, now + 2U, &diagonal);
  const unsigned down = APP_Q3_ROLL_SIGN == -1 ? 2U : 3U;
  assert(brake.output.pwm_permille[down] > 0U && brake.output.pwm_permille[5U - down] == 0U);
  assert(brake.output.pwm_permille[1] > 0U && brake.output.pwm_permille[0] == 0U);
  assert(TotalDuty(&brake) <= APP_Q1_PWM_MAX_PERMILLE);
}

static void TestSettleAndDeadline(void)
{
  Q4Brake brake = {0};
  const uint32_t start = UINT32_MAX - 100U;
  HoldForRelease(&brake, start);
  const uint32_t release = start + APP_Q4_HOLD_MS + 2U;
  Q4Brake_Step(&brake, release, &released);
  Q4Brake_Step(&brake, release + 2U, &center);
  assert(brake.phase == Q4_VERIFY_STOP && TotalDuty(&brake) == 0U);
  Q4Brake_Step(&brake, release + 4U, &released);
  assert(brake.phase == Q4_BRAKING && TotalDuty(&brake) > 0U);
  for (uint32_t elapsed = 6U; elapsed < APP_Q4_DURATION_MS; elapsed += 2U) {
    const Q4Sample *sample = elapsed < APP_Q4_DURATION_MS - APP_Q4_SETTLE_MS ? &released : &center;
    Q4Brake_Step(&brake, release + elapsed, sample);
    assert(brake.state == PROBLEM_STATUS_RUNNING);
  }
  Q4Brake_Step(&brake, release + APP_Q4_DURATION_MS, &center);
  assert(brake.state == PROBLEM_STATUS_FINISHED && brake.reason == Q4_BRAKE_SETTLED);
  assert(brake.run_ms == APP_Q4_DURATION_MS && TotalDuty(&brake) == 0U);

  HoldForRelease(&brake, start);
  for (uint32_t elapsed = 0; elapsed <= APP_Q4_DURATION_MS; elapsed += 2U)
    Q4Brake_Step(&brake, release + elapsed, &released);
  assert(brake.state == PROBLEM_STATUS_FAULT && brake.reason == Q4_BRAKE_NOT_SETTLED);
  assert(brake.run_ms == APP_Q4_DURATION_MS && TotalDuty(&brake) == 0U);
}

static void TestProtection(void)
{
  Q4Brake brake = {0};
  assert(Q4Brake_Arm(&brake, 0U, &center));
  for (uint32_t now = 2U; now <= APP_Q4_ARM_TIMEOUT_MS; now += 2U)
    Q4Brake_Step(&brake, now, &held);
  assert(brake.reason == Q4_BRAKE_NO_RELEASE && brake.run_ms == 0U && TotalDuty(&brake) == 0U);

  Q4Sample out_of_bounds = {.tilt_deg = 51.0F, .pitch_deg = 51.0F};
  assert(!Q4Brake_Arm(&brake, 0U, &out_of_bounds));
  assert(Q4Brake_Arm(&brake, 0U, &center));
  Q4Brake_Step(&brake, 2U, &out_of_bounds);
  assert(brake.reason == Q1_SWING_ANGLE_LIMIT && TotalDuty(&brake) == 0U);

  const Q4Sample over_speed = {.roll_rate_dps = 180.0F, .pitch_rate_dps = -180.0F};
  assert(Q4Brake_Arm(&brake, 0U, &center));
  Q4Brake_Step(&brake, 2U, &over_speed);
  assert(brake.reason == Q1_SWING_ANGLE_LIMIT && TotalDuty(&brake) == 0U);

  HoldForRelease(&brake, 0U);
  Q4Brake_Step(&brake, APP_Q4_HOLD_MS + 2U, &released);
  Q4Brake_Step(&brake, APP_Q4_HOLD_MS + 24U, &released);
  assert(brake.reason == Q1_SWING_SAMPLE_STALE && TotalDuty(&brake) == 0U);
  const Q4Sample invalid = {.tilt_deg = NAN};
  assert(!Q4Brake_Arm(&brake, 0U, &invalid));
}

int main(void)
{
  if (APP_Q3_ROLL_SIGN == 0) {
    Q4Brake brake = {0};
    assert(!Q4Brake_Arm(&brake, 0U, &center));
    assert(brake.reason == Q1_SWING_INVALID_CONFIG && TotalDuty(&brake) == 0U);
  } else {
    TestRelease();
    TestSettleAndDeadline();
    TestProtection();
  }
  puts("Q4 brake tests passed");
  return 0;
}
