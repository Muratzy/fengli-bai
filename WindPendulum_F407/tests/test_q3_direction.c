#include "app_config.h"
#include "problems/q3_direction.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#if APP_Q3_ROLL_SIGN != 0
static const float rad_per_deg = 0.0174532925199F;
static const unsigned up_channel = APP_Q3_ROLL_SIGN > 0 ? 2U : 3U;
static const unsigned down_channel = APP_Q3_ROLL_SIGN > 0 ? 3U : 2U;

static unsigned CheckPwm(const Q3Direction *line)
{
  const uint16_t *pwm = line->output.pwm_permille;
  assert(pwm[0] == 0U || pwm[1] == 0U);
  assert(pwm[2] == 0U || pwm[3] == 0U);
  const unsigned total = pwm[0] + pwm[1] + pwm[2] + pwm[3];
  assert(total <= APP_Q1_PWM_MAX_PERMILLE);
  return total;
}

static void TestDirections(void)
{
  for (uint32_t direction = 0; direction < 360; direction += APP_Q3_DIRECTION_STEP_DEG) {
    Q1Swing along = {0};
    Q3Direction line = {0};
    ProblemParams params = ProblemApi_DefaultParams(PROBLEM_Q3);
    params.direction_deg = direction;
    assert(Q3Direction_Start(&line, &along, &params, 0U, 0.0F, 0.0F, 0.0F, 0.0F));
    float minimum = 0.0F;
    float maximum = 0.0F;
    for (uint32_t now = 2U; now < APP_Q3_DURATION_MS; now += 2U) {
      Q3Direction_Step(&line, &along, now, 0.0F, 0.0F, 0.0F, 0.0F);
      assert(along.state == Q1_SWING_RUNNING);
      CheckPwm(&line);
      const float roll = line.target_roll_deg * rad_per_deg;
      const float pitch = line.target_pitch_deg * rad_per_deg;
      const float x = APP_PIVOT_GROUND_HEIGHT_M * tanf(pitch) / cosf(roll);
      const float y = APP_PIVOT_GROUND_HEIGHT_M * tanf(roll);
      assert(fabsf(line.up * x - line.right * y) < 0.00001F);
      const float position = line.up * y + line.right * x;
      minimum = fminf(minimum, position);
      maximum = fmaxf(maximum, position);
      if (now == 3000U) {
        const uint16_t *pwm = line.output.pwm_permille;
        /* Reference is negative at this point, so the thrust opposes the axis. */
        if (direction == 0U) assert(pwm[down_channel] > 0U && pwm[0] == 0U && pwm[1] == 0U);
        if (direction == 90U) assert(pwm[1] > 0U && pwm[2] == 0U && pwm[3] == 0U);
        if (direction == 180U) assert(pwm[up_channel] > 0U && pwm[0] == 0U && pwm[1] == 0U);
        if (direction == 270U) assert(pwm[0] > 0U && pwm[2] == 0U && pwm[3] == 0U);
        if (direction == 40U) assert(pwm[down_channel] > 0U && pwm[1] > 0U);
      }
    }
    assert(fabsf(maximum - minimum - (float)params.line_length_mm * 0.001F) < 0.0001F);
    Q3Direction_Step(&line, &along, APP_Q3_DURATION_MS, 0.0F, 0.0F, 0.0F, 0.0F);
    assert(along.state == Q1_SWING_FINISHED && along.reason == Q1_SWING_TIME_UP);
    assert(CheckPwm(&line) == 0U);
  }
}

static void TestFeedbackAndSafety(void)
{
  Q1Swing along = {0};
  Q3Direction line = {0};
  assert(!Q3Direction_Start(&line, &along, NULL, 0U, 0.0F, 0.0F, 6.0F, 0.0F));
  assert(along.reason == Q1_SWING_NOT_CENTERED);
  assert(Q3Direction_Start(&line, &along, NULL, 0U, 0.0F, 0.0F, 0.0F, 0.0F));
  Q3Direction_Step(&line, &along, 2U, 0.0F, 3.0F, 0.0F, 0.0F);
  assert(line.output.pwm_permille[1] > 0U);
  Q3Direction_Step(&line, &along, 4U, -16.0F, -16.0F, -80.0F, -80.0F);
  assert(CheckPwm(&line) >= APP_Q1_PWM_MAX_PERMILLE - 1U);
  Q3Direction_Step(&line, &along, 6U, 20.0F, 20.0F, 0.0F, 0.0F);
  assert(along.reason == Q1_SWING_ANGLE_LIMIT && CheckPwm(&line) == 0U);
  assert(Q3Direction_Start(&line, &along, NULL, 0U, 0.0F, 0.0F, 0.0F, 0.0F));
  Q3Direction_Step(&line, &along, 21U, 0.0F, 0.0F, 0.0F, 0.0F);
  assert(along.reason == Q1_SWING_SAMPLE_STALE && CheckPwm(&line) == 0U);
  assert(Q3Direction_Start(&line, &along, NULL, 0U, 0.0F, 0.0F, 0.0F, 0.0F));
  for (uint32_t now = 2U; now <= 7000U; now += 2U) {
    const float phase = (float)now / APP_Q1_PERIOD_MS * 6.2831853F;
    Q3Direction_Step(&line, &along, now, 1.0F + 7.2F * sinf(phase), 0.0F,
        7.2F * (6.2831853F * 1000.0F / APP_Q1_PERIOD_MS) * cosf(phase), 0.0F);
    assert(along.state == Q1_SWING_RUNNING);
    CheckPwm(&line);
  }
  assert(along.control_phase == Q1_SWING_CONTROL_AMPLITUDE);
  assert(along.center_output < 0.0F);
  Q1Swing_Stop(&along, Q1_SWING_USER_STOP);
  Q3Direction_Step(&line, &along, 7002U, 0.0F, 0.0F, 0.0F, 0.0F);
  assert(CheckPwm(&line) == 0U && !line.output.pwm_ready);
}

static void TestMeasuredProjections(void)
{
  for (uint32_t direction = 0U; direction < 360U; direction += APP_Q3_DIRECTION_STEP_DEG) {
    Q1Swing along = {0};
    Q3Direction line = {0};
    ProblemParams params = ProblemApi_DefaultParams(PROBLEM_Q3);
    params.direction_deg = direction;
    assert(Q3Direction_Start(&line, &along, &params, 0U, 0.0F, 0.0F, 0.0F, 0.0F));
    const float axial = tanf(4.0F * rad_per_deg);
    const float cross = tanf(2.0F * rad_per_deg);
    const float axial_rate = 12.0F * rad_per_deg * (1.0F + axial * axial);
    const float cross_rate = -7.0F * rad_per_deg * (1.0F + cross * cross);
    const float x = line.right * axial + line.up * cross;
    const float y = line.up * axial - line.right * cross;
    const float dx = line.right * axial_rate + line.up * cross_rate;
    const float dy = line.up * axial_rate - line.right * cross_rate;
    const float cos_roll = 1.0F / sqrtf(1.0F + y * y);
    const float tan_pitch = x * cos_roll;
    const float roll = atanf(y) / rad_per_deg;
    const float pitch = atanf(tan_pitch) / rad_per_deg;
    const float roll_rate = dy / ((1.0F + y * y) * rad_per_deg);
    const float pitch_rate = (dx * cos_roll - x * y * dy * cos_roll * cos_roll * cos_roll) /
        ((1.0F + tan_pitch * tan_pitch) * rad_per_deg);
    Q3Direction_Step(&line, &along, 2U, roll, pitch, roll_rate, pitch_rate);
    assert(along.state == Q1_SWING_RUNNING);
    const PID_Controller *axial_pid = &along.controller.axis_pid[PENDULUM_AXIS_PITCH];
    const float alpha = 0.002F / (APP_Q1_RATE_FILTER_TAU_S + 0.002F);
    assert(fabsf(axial_pid->previous_measurement - 4.0F) < 0.0001F);
    assert(fabsf(line.cross_pid.previous_measurement - 2.0F) < 0.0001F);
    assert(fabsf(axial_pid->derivative + 12.0F * alpha) < 0.0001F);
    assert(fabsf(line.cross_pid.derivative - 7.0F * alpha) < 0.0001F);
    CheckPwm(&line);
  }
}

static void TestTuningCanBeSelectedFromHost(void)
{
  Q1Swing along = {0};
  Q3Direction line = {0};
  ProblemParams params = ProblemApi_DefaultParams(PROBLEM_Q3);
  Q1Swing_Tuning tuning = Q1Swing_DefaultTuning();
  tuning.startup_kp = 31.0F;
  tuning.startup_kd = 8.0F;
  tuning.amplitude_kp = 3.0F;
  tuning.damping_compensation = 0.7F;
  assert(Q3Direction_StartTuned(&line, &along, &params, &tuning, 0U,
      0.0F, 0.0F, 0.0F, 0.0F));
  assert(along.controller.config.pitch_pid.kp == 31.0F);
  assert(along.controller.config.pitch_pid.kd == 8.0F);
  assert(along.amplitude_pid.config.kp == 3.0F);
  assert(along.tuning.damping_compensation == 0.7F);
  assert(along.tuning.target_amplitude_deg > 0.0F);
  assert(along.tuning.duration_ms == APP_Q3_DURATION_MS);
}
#endif

int main(void)
{
  assert(!Q3Direction_Start(NULL, NULL, NULL, 0U, 0.0F, 0.0F, 0.0F, 0.0F));
  Q3Direction_Step(NULL, NULL, 0U, 0.0F, 0.0F, 0.0F, 0.0F);
#if APP_Q3_ROLL_SIGN == 0
  Q1Swing along = {0};
  Q3Direction line = {0};
  assert(!Q3Direction_Start(&line, &along, NULL, 0U, 0.0F, 0.0F, 0.0F, 0.0F));
  assert(along.reason == Q1_SWING_INVALID_CONFIG && !line.output.pwm_ready);
#else
  TestDirections();
  TestMeasuredProjections();
  TestTuningCanBeSelectedFromHost();
  TestFeedbackAndSafety();
#endif
  puts("Q3 direction-line tests passed");
  return 0;
}
