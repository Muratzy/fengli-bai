#include "control/pendulum_control.h"
#include "control/pid.h"
#include "problems/problem_q1.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static PID_Config MakePidConfig(float kp, float ki, float kd,
                                float output_min, float output_max)
{
  PID_Config config = {
      .kp = kp,
      .ki = ki,
      .kd = kd,
      .proportional_setpoint_weight = 1.0F,
      .derivative_filter_tau_s = 0.05F,
      .integral_min = -2.0F,
      .integral_max = 2.0F,
      .output_min = output_min,
      .output_max = output_max};
  return config;
}

static void TestPidProportionalAndReset(void)
{
  PID_Controller pid;
  PID_Config config = MakePidConfig(2.0F, 0.0F, 0.0F, -20.0F, 20.0F);

  assert(PID_Init(&pid, &config));
  PID_Reset(&pid, 3.0F, 0.0F);
  assert(fabsf(PID_Update(&pid, 10.0F, 3.0F, 0.01F) - 14.0F) < 0.001F);
  assert(fabsf(pid.p_term - 14.0F) < 0.001F);
  PID_Reset(&pid, 3.0F, 0.0F);
  assert(fabsf(PID_Update(&pid, 3.0F, 3.0F, 0.01F)) < 0.001F);
}

static void TestPidFilteredDerivativeHasNoSetpointKick(void)
{
  PID_Controller pid;
  PID_Config config = MakePidConfig(0.0F, 0.0F, 1.0F, -100.0F, 100.0F);

  assert(PID_Init(&pid, &config));
  PID_Reset(&pid, 0.0F, 0.0F);
  (void)PID_Update(&pid, 0.0F, 0.0F, 0.01F);
  (void)PID_Update(&pid, 10.0F, 0.0F, 0.01F);
  assert(fabsf(pid.d_term) < 0.001F);
  (void)PID_Update(&pid, 10.0F, 1.0F, 0.01F);
  assert(pid.d_term < 0.0F);
  assert(pid.d_term > -100.0F);
}

static void TestPidAntiWindupAndLimits(void)
{
  PID_Controller pid;
  PID_Config config = MakePidConfig(0.0F, 10.0F, 0.0F, -1.0F, 1.0F);

  config.integral_min = -5.0F;
  config.integral_max = 5.0F;
  assert(PID_Init(&pid, &config));
  PID_Reset(&pid, 0.0F, 0.0F);
  for (uint32_t step = 0U; step < 100U; ++step) {
    (void)PID_Update(&pid, 10.0F, 0.0F, 0.01F);
  }
  assert(fabsf(pid.output - 1.0F) < 0.001F);
  assert(fabsf(pid.integral - 1.0F) < 0.001F);
  (void)PID_Update(&pid, -1.0F, 0.0F, 0.01F);
  (void)PID_Update(&pid, -1.0F, 0.0F, 0.01F);
  assert(pid.integral < 1.0F);
}

static ProblemQ1_Config MakeQ1Config(void)
{
  ProblemQ1_Config config = ProblemQ1_DefaultConfig();
  /* Round timing for exact phase and derivative checks. */
  config.swing_period_ms = 2000U;
  config.startup_ramp_ms = 4000U;
  return config;
}

static void TestQ1TrajectoryAndPreconditions(void)
{
  ProblemQ1 problem = {0};
  ProblemQ1_Output output;
  ProblemQ1_Config config = MakeQ1Config();

  assert(ProblemQ1_Validate(&config));
  assert(ProblemQ1_Start(&problem, &config, 100U, true, false) ==
         PROBLEM_Q1_NOT_CALIBRATED);
  assert(ProblemQ1_Start(&problem, &config, 100U, true, true) ==
         PROBLEM_Q1_OK);

  assert(ProblemQ1_Step(&problem, 100U, &output));
  assert(fabsf(output.target_position_m) < 0.001F);
  assert(fabsf(output.target_velocity_mps) < 0.001F);
  assert(ProblemQ1_Step(&problem, 4600U, &output));
  assert(fabsf(output.target_position_m - 0.25F) < 0.0001F);
  assert(fabsf(output.target_pitch_deg - 14.17234F) < 0.001F);
  assert(fabsf(output.target_roll_deg) < 0.0001F);
  assert(fabsf(output.target_velocity_mps) < 0.0001F);
  assert(ProblemQ1_Step(&problem, 5600U, &output));
  assert(fabsf(output.target_position_m + 0.25F) < 0.0001F);
  assert(fabsf(output.target_pitch_deg + 14.17234F) < 0.001F);
  assert(ProblemQ1_Step(&problem, 5100U, &output));
  assert(fabsf(output.target_position_m) < 0.0001F);
  assert(output.target_velocity_mps < 0.0F);
  assert(ProblemQ1_Step(&problem, 15100U, &output));
  assert(output.complete);
  assert(problem.state == PROBLEM_Q1_COMPLETE);
  assert(fabsf(output.target_position_m) < 0.001F);
}

static void TestQ1MeasuredGeometryAndValidation(void)
{
  ProblemQ1_Config config = ProblemQ1_DefaultConfig();
  ProblemQ1 problem = {0};
  ProblemQ1_Output output;
  assert(config.swing_period_ms == 1642U);
  assert(config.startup_ramp_ms == 3284U);
  assert(config.axis == PROBLEM_Q1_AXIS_PITCH);
  assert(ProblemQ1_Validate(&config));
  /* A 50 cm trace is +/-25 cm, not 50 cm to one side. */
  config.max_angle_deg = 14.0F;
  assert(!ProblemQ1_Validate(&config));
  config.max_angle_deg = 14.2F;
  assert(ProblemQ1_Validate(&config));
  float left = 0.0F;
  float right = 0.0F;
  assert(ProblemQ1_Start(&problem, &config, UINT32_MAX - 500U, true, true) == PROBLEM_Q1_OK);
  for (uint32_t elapsed = 0U; elapsed < config.duration_ms; elapsed += 10U)
  {
    assert(ProblemQ1_Step(&problem, (UINT32_MAX - 500U) + elapsed, &output));
    assert(isfinite(output.target_velocity_mps));
    assert(fabsf(output.target_pitch_deg) <= 14.173F);
    assert(fabsf(output.target_roll_deg) < 0.0001F);
    if (output.target_position_m < left) left = output.target_position_m;
    if (output.target_position_m > right) right = output.target_position_m;
  }
  assert(left < -0.249F && right > 0.249F);
  config.ground_height_m = 0.0F;
  assert(!ProblemQ1_Validate(&config));
  config = ProblemQ1_DefaultConfig();
  config.ground_height_m = NAN;
  assert(!ProblemQ1_Validate(&config));
  config = ProblemQ1_DefaultConfig();
  config.swing_period_ms = 0U;
  assert(!ProblemQ1_Validate(&config));
  config = ProblemQ1_DefaultConfig();
  config.startup_ramp_ms = 0U;
  assert(!ProblemQ1_Validate(&config));
  config.startup_ramp_ms = UINT32_MAX;
  assert(!ProblemQ1_Validate(&config));
}

static void TestQ1PolarityAndVelocity(void)
{
  ProblemQ1_Config config = MakeQ1Config();
  ProblemQ1 problem = {0};
  ProblemQ1_Output before, output, after;
  config.direction_sign = -1.0F;
  config.axis = PROBLEM_Q1_AXIS_ROLL;
  assert(ProblemQ1_Start(&problem, &config, 0U, true, true) == PROBLEM_Q1_OK);
  assert(ProblemQ1_Step(&problem, 4500U, &output));
  assert(fabsf(output.target_roll_deg + 14.17234F) < 0.001F);
  assert(fabsf(output.target_pitch_deg) < 0.0001F);
  /* The reported ground velocity includes both the ramp derivative and tan. */
  for (uint32_t time = 100U; time <= 5000U; time += 37U)
  {
    assert(ProblemQ1_Step(&problem, time - 1U, &before));
    assert(ProblemQ1_Step(&problem, time, &output));
    assert(ProblemQ1_Step(&problem, time + 1U, &after));
    const float numerical_velocity =
        (after.target_position_m - before.target_position_m) / 0.002F;
    assert(fabsf(numerical_velocity - output.target_velocity_mps) < 0.001F);
  }
}

static void TestControlMixerGate(void)
{
  PendulumControl control;
  PendulumControl_Output output;
  PendulumControl_Config config = {0};
  config.roll_pid = MakePidConfig(0.1F, 0.0F, 0.0F, -1.0F, 1.0F);
  config.pitch_pid = MakePidConfig(0.1F, 0.0F, 0.0F, -1.0F, 1.0F);
  config.max_duty_permille = 1000U;
  config.fan_mix[0][0] = 200.0F;
  config.fan_mix[1][1] = 200.0F;
  config.base_duty_permille[0] = 500U;
  config.base_duty_permille[1] = 500U;

  assert(PendulumControl_Init(&control, &config));
  PendulumControl_Reset(&control, 0.0F, 0.0F);
  assert(PendulumControl_Step(&control, 5.0F, 0.0F, 0.0F, 0.0F,
                              0.01F, &output));
  assert(!output.pwm_ready);
  assert(fabsf(output.axis_output[0] - 0.5F) < 0.001F);
  assert(output.pwm_permille[0] == 0U);

  config.mixer_calibrated = true;
  assert(PendulumControl_Init(&control, &config));
  PendulumControl_Reset(&control, 0.0F, 0.0F);
  assert(PendulumControl_Step(&control, 5.0F, 0.0F, 0.0F, 0.0F,
                              0.01F, &output));
  assert(output.pwm_ready);
  assert(output.pwm_permille[0] == 600U);
  assert(output.pwm_permille[1] == 500U);
}

int main(void)
{
  TestPidProportionalAndReset();
  TestPidFilteredDerivativeHasNoSetpointKick();
  TestPidAntiWindupAndLimits();
  TestQ1TrajectoryAndPreconditions();
  TestQ1MeasuredGeometryAndValidation();
  TestQ1PolarityAndVelocity();
  TestControlMixerGate();
  puts("control algorithm tests passed");
  return 0;
}
