#include "problems/q1_swing.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void CheckOutput(const Q1Swing *s)
{
  assert(s->pwm[2] == 0U && s->pwm[3] == 0U);
  assert(s->pwm[0] <= 1000U && s->pwm[1] <= 1000U);
  assert(s->pwm[0] == 0U || s->pwm[1] == 0U);
}

static void TestTurnsAndCoast(void)
{
  Q1Swing s = {0};
  assert(Q1Swing_Start(&s, 0U, 0.0F, 0.0F, 0.0F));
  Q1Swing_Step(&s, 2U, 0.0F, 0.0F, 0.0F, true, false);
  assert(s.switches == 0U && s.pwm[0] == 4U);
  assert(s.pwm[1] == 0U); /* PWM1, the left motor, moves right. */
  for (uint32_t ms = 4U; ms < 1200U; ms += 2U)
    Q1Swing_Step(&s, ms, 0.0F, (float)ms * (8.0F / 1200.0F), 12.0F, true, false);
  Q1Swing_Step(&s, 1200U, 0.0F, 8.0F, 1.5F, true, false);
  assert(s.switches == 1U && s.direction == -1);
  assert(s.pwm[0] > 0U && s.pwm[0] < 1000U && s.pwm[1] == 0U);
  Q1Swing_Step(&s, 1202U, 0.0F, 8.0F, 0.2F, true, false);
  assert(s.switches == 1U); /* Near-zero noise cannot switch back. */
  for (uint32_t ms = 1204U; ms < 2800U; ms += 2U) {
    Q1Swing_Step(&s, ms, 0.0F, 8.0F - (float)(ms - 1200U) * 0.01F,
                 -12.0F, true, false);
    CheckOutput(&s);
    if (ms == 1398U) assert(s.pwm[0] == 0U && s.pwm[1] == 0U);
    if (ms == 1400U) assert(s.pwm[0] == 0U && s.pwm[1] == 4U);
    if (ms == 1898U) assert(s.pwm[1] == 1000U);
  }
  Q1Swing_Step(&s, 2800U, 0.0F, -8.0F, -1.5F, true, false);
  assert(s.switches == 2U && s.direction == 1);
  CheckOutput(&s);
  Q1Swing_Step(&s, 2802U, 0.0F, -14.5F, 0.0F, true, false);
  assert(!s.boosting);
  for (uint32_t ms = 2804U; ms <= 3202U; ms += 2U) {
    Q1Swing_Step(&s, ms, 0.0F, -14.5F, 0.0F, true, false);
    CheckOutput(&s);
  }
  assert(s.pwm[0] == 0U && s.pwm[1] == 0U);
}

static void TestRampAndImmediateStop(void)
{
  Q1Swing s = {0};
  assert(Q1Swing_Start(&s, 0U, 0.0F, 0.0F, 0.0F));
  Q1Swing_Step(&s, 0U, 0.0F, 0.0F, 0.0F, true, false);
  assert(s.pwm[0] == 0U);
  uint16_t previous = 0U;
  for (uint32_t ms = 1U; ms <= 500U; ++ms) {
    Q1Swing_Step(&s, ms, 0.0F, 0.0F, 0.0F, true, false);
    CheckOutput(&s);
    assert(s.pwm[0] >= previous && s.pwm[0] - previous <= 2);
    if (ms < 500U) assert(s.pwm[0] < 1000U);
    previous = s.pwm[0];
  }
  assert(s.pwm[0] == 1000U);
  Q1Swing_Step(&s, 502U, 0.0F, 0.0F, 0.0F, true, true);
  assert(s.reason == Q1_SWING_USER_STOP && s.pwm[0] == 0U);
  assert(Q1Swing_Start(&s, 2000U, 0.0F, 0.0F, 0.0F));
  Q1Swing_Step(&s, 2000U, 0.0F, 0.0F, 0.0F, true, false);
  for (uint32_t ms = 2002U; ms <= 2500U; ms += 2U)
    Q1Swing_Step(&s, ms, 0.0F, 0.0F, 0.0F, true, false);
  assert(s.pwm[0] == 1000U);
  Q1Swing_Step(&s, 2502U, 0.0F, 26.0F, 0.0F, true, false);
  assert(s.reason == Q1_SWING_ANGLE_LIMIT && s.pwm[0] == 0U);
}

static void TestLimits(void)
{
  Q1Swing s = {0};
  assert(!Q1Swing_Start(&s, 0U, 0.0F, 7.01F, 0.0F));
  assert(s.reason == Q1_SWING_NOT_CENTERED);
  assert(Q1Swing_Start(&s, 0U, 6.9F, -6.9F, 4.0F));
  Q1Swing_Stop(&s, Q1_SWING_USER_STOP);
  assert(Q1Swing_Start(&s, 0U, 0.0F, 0.0F, 0.0F));
  Q1Swing_Step(&s, 21U, 0.0F, 0.0F, 0.0F, true, false);
  assert(s.reason == Q1_SWING_SAMPLE_STALE);
  assert(Q1Swing_Start(&s, 100U, 0.0F, 0.0F, 0.0F));
  Q1Swing_Step(&s, 102U, 0.0F, 26.0F, 0.0F, true, false);
  assert(s.reason == Q1_SWING_ANGLE_LIMIT);
  assert(Q1Swing_Start(&s, 200U, 0.0F, 0.0F, 0.0F));
  Q1Swing_Step(&s, 202U, 0.0F, 0.0F, 0.0F, true, true);
  assert(s.reason == Q1_SWING_USER_STOP);
  assert(Q1Swing_Start(&s, 300U, 0.0F, 0.0F, 0.0F));
  for (uint32_t ms = 302U; ms <= 2802U; ms += 2U)
    Q1Swing_Step(&s, ms, 0.0F, 0.0F, 0.0F, true, false);
  assert(s.reason == Q1_SWING_NO_TURN);
  CheckOutput(&s);
}

static void TestFullWindow(uint32_t start)
{
  Q1Swing s = {0};
  assert(Q1Swing_Start(&s, start, 0.0F, 0.0F, 0.0F));
  /* Representative under-target oscillation over the whole window. */
  for (uint32_t t = 2U; t <= 15000U; t += 2U) {
    const float phase = (float)t * 0.001F * 6.2831853F / 1.64F;
    Q1Swing_Step(&s, start + t, 0.0F, 8.0F * sinf(phase),
                 8.0F * (6.2831853F / 1.64F) * cosf(phase), true, false);
    CheckOutput(&s);
    if (t < 15000U) assert(s.state == Q1_SWING_RUNNING);
  }
  assert(s.reason == Q1_SWING_TIME_UP && s.switches > 10U);
  CheckOutput(&s);
  Q1Swing_Step(&s, start + 15002U, 0.0F, 0.0F, 0.0F, true, false);
  assert(s.state == Q1_SWING_FINISHED); /* No automatic restart. */
}

int main(void)
{
  TestTurnsAndCoast();
  TestRampAndImmediateStop();
  TestLimits();
  TestFullWindow(0U);
  TestFullWindow(UINT32_MAX - 200U);
  puts("Q1 swing tests passed");
  return 0;
}
