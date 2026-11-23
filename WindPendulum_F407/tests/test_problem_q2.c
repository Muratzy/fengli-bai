#include "problems/problem_q2.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void CheckOutput(const ProblemQ2 *problem)
{
  assert(problem->pwm[0] <= 1000U && problem->pwm[1] <= 1000U);
  assert(problem->pwm[0] == 0U || problem->pwm[1] == 0U);
  assert(problem->pwm[2] == 0U && problem->pwm[3] == 0U);
  assert(problem->controller.axis_pid[0].integral == 0.0F);
  assert(problem->control_output.axis_output[0] == 0.0F);
}

static void TestConfigAndStart(void)
{
  ProblemQ2_Config config = ProblemQ2_DefaultConfig();
  ProblemQ2 problem = {0};
  config.line_length_m = 0.30F;
  assert(ProblemQ2_Validate(&config));
  config.line_length_m = 0.60F;
  assert(ProblemQ2_Validate(&config));
  config.line_length_m = 0.299F;
  assert(!ProblemQ2_Start(&problem, &config, 0U, 0.0F, 0.0F));
  assert(problem.reason == PROBLEM_Q2_INVALID_CONFIG);
  config.line_length_m = 0.601F;
  assert(!ProblemQ2_Validate(&config));
  config = ProblemQ2_DefaultConfig();
  config.amplitude_ramp_ms = UINT32_MAX;
  assert(!ProblemQ2_Validate(&config));
  config = ProblemQ2_DefaultConfig();
  config.pitch_pid.kp = NAN;
  assert(!ProblemQ2_Validate(&config));
  config = ProblemQ2_DefaultConfig();
  assert(!ProblemQ2_Start(&problem, &config, 0U, 7.01F, 0.0F));
  assert(ProblemQ2_Start(&problem, &config, 0U, 7.0F, -7.0F));
  assert(problem.pwm[0] == 0U && problem.pwm[1] == 0U);
  ProblemQ2_Step(&problem, 2U, 7.0F, -7.0F, 4.0F, 4.0F, true, false);
  assert(problem.state == PROBLEM_Q2_RUNNING);
  CheckOutput(&problem);
}

static void TestSlewPolarityAndStop(void)
{
  ProblemQ2_Config config = ProblemQ2_DefaultConfig();
  config.pitch_pid.kp = 100.0F;
  config.pitch_pid.ki = 0.0F;
  config.pitch_pid.kd = 0.0F;
  ProblemQ2 problem = {0};
  assert(ProblemQ2_Start(&problem, &config, 0U, 0.0F, -5.0F));
  for (uint32_t ms = 2U; ms <= 100U; ms += 2U) {
    ProblemQ2_Step(&problem, ms, 0.0F, -5.0F, 0.0F, 0.0F, true, false);
    CheckOutput(&problem);
    assert(problem.pwm[0] <= ms * 2U && problem.pwm[1] == 0U);
  }
  assert(problem.pwm[0] == 200U);
  bool saw_zero = false;
  for (uint32_t ms = 102U; ms <= 300U; ms += 2U) {
    const uint16_t old_left = problem.pwm[0];
    ProblemQ2_Step(&problem, ms, 0.0F, 5.0F, 0.0F, 0.0F, true, false);
    CheckOutput(&problem);
    if (old_left > 0U) {
      assert(problem.pwm[1] == 0U);
      assert(old_left - problem.pwm[0] <= 10);
    }
    if (problem.pwm[0] == 0U && problem.pwm[1] == 0U) saw_zero = true;
  }
  assert(saw_zero && problem.pwm[1] > 0U && problem.switches == 1U);
  ProblemQ2_Step(&problem, 302U, 0.0F, 5.0F, 0.0F, 0.0F, true, true);
  assert(problem.state == PROBLEM_Q2_STOPPED && problem.pwm[1] == 0U);
}

static void TestTrajectoryWindowAndWrap(uint32_t start, float length)
{
  ProblemQ2_Config config = ProblemQ2_DefaultConfig();
  config.line_length_m = length;
  config.period_ms = 2000U;
  config.amplitude_ramp_ms = 4000U;
  ProblemQ2 problem = {0};
  assert(ProblemQ2_Start(&problem, &config, start, 0.0F, 0.0F));
  ProblemQ2_Step(&problem, start, 0.0F, 0.0F, 0.0F, 0.0F, true, false);
  assert(problem.target_pitch_deg == 0.0F && problem.target_pitch_rate_dps == 0.0F);
  float before_position = 0.0F;
  float before_velocity = 0.0F;
  for (uint32_t ms = 2U; ms <= 15000U; ms += 2U) {
    ProblemQ2_Step(&problem, start + ms, 0.0F, 0.0F, 0.0F, 0.0F, true, false);
    CheckOutput(&problem);
    if (ms < 15000U) {
      assert(problem.state == PROBLEM_Q2_RUNNING);
      assert(fabsf(problem.target_position_m) <= length * 0.5F + 0.0001F);
      const float numerical = (problem.target_position_m - before_position) / 0.002F;
      assert(fabsf(numerical - 0.5F * (problem.target_velocity_mps + before_velocity)) < 0.001F);
    }
    if (ms == 4500U) assert(fabsf(problem.target_position_m - length * 0.5F) < 0.0001F);
    if (ms == 5500U) assert(fabsf(problem.target_position_m + length * 0.5F) < 0.0001F);
    before_position = problem.target_position_m;
    before_velocity = problem.target_velocity_mps;
  }
  assert(problem.state == PROBLEM_Q2_FINISHED && problem.reason == PROBLEM_Q2_TIME_UP);
  assert(problem.pwm[0] == 0U && problem.pwm[1] == 0U);
  ProblemQ2_Step(&problem, start + 15002U, 0.0F, 0.0F, 0.0F, 0.0F, true, false);
  assert(problem.state == PROBLEM_Q2_FINISHED);
}

static void TestFaultAndSlewIntegralFreeze(void)
{
  ProblemQ2_Config config = ProblemQ2_DefaultConfig();
  ProblemQ2 problem = {0};
  assert(ProblemQ2_Start(&problem, &config, 0U, 0.0F, -5.0F));
  for (uint32_t ms = 2U; ms <= 80U; ms += 2U)
    ProblemQ2_Step(&problem, ms, 0.0F, -5.0F, 0.0F, 0.0F, true, false);
  assert(fabsf(problem.controller.axis_pid[1].integral) < 0.1F);
  assert(problem.pwm[0] > 0U);
  ProblemQ2_Step(&problem, 101U, 0.0F, -5.0F, 0.0F, 0.0F, true, false);
  assert(problem.reason == PROBLEM_Q2_SAMPLE_STALE && problem.pwm[0] == 0U);
  assert(ProblemQ2_Start(&problem, &config, 200U, 0.0F, 0.0F));
  ProblemQ2_Step(&problem, 202U, 0.0F, 26.0F, 0.0F, 0.0F, true, false);
  assert(problem.reason == PROBLEM_Q2_ANGLE_LIMIT);
  assert(ProblemQ2_Start(&problem, &config, 300U, 0.0F, 0.0F));
  ProblemQ2_Step(&problem, 302U, 0.0F, 0.0F, NAN, 0.0F, true, false);
  assert(problem.reason == PROBLEM_Q2_SAMPLE_STALE);
}

int main(void)
{
  TestConfigAndStart();
  TestSlewPolarityAndStop();
  TestFaultAndSlewIntegralFreeze();
  TestTrajectoryWindowAndWrap(0U, 0.30F);
  TestTrajectoryWindowAndWrap(0U, 0.60F);
  TestTrajectoryWindowAndWrap(UINT32_MAX - 200U, 0.50F);
  puts("Q2 PID tests passed");
  return 0;
}
