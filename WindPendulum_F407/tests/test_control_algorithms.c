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
  ProblemQ1_Config config = {
      .line_length_m = 0.50F,
      .effective_length_m = 0.70F,
      .max_angle_deg = 45.0F,
      .direction_sign = 1.0F,
      .duration_ms = 15000U,
      .axis = PROBLEM_Q1_AXIS_ROLL};
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
  assert(ProblemQ1_Step(&problem, 3850U, &output));
  assert(fabsf(output.target_position_m - 0.25F) < 0.002F);
  assert(ProblemQ1_Step(&problem, 7600U, &output));
  assert(fabsf(output.target_position_m - 0.50F) < 0.002F);
  assert(fabsf(output.target_roll_deg - 35.54F) < 0.1F);
  assert(ProblemQ1_Step(&problem, 11350U, &output));
  assert(output.target_velocity_mps < 0.0F);
  assert(ProblemQ1_Step(&problem, 15100U, &output));
  assert(output.complete);
  assert(problem.state == PROBLEM_Q1_COMPLETE);
  assert(fabsf(output.target_position_m) < 0.001F);
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
  TestControlMixerGate();
  puts("control algorithm tests passed");
  return 0;
}
