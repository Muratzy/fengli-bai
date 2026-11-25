#include "app_config.h"
#include "problems/q1_swing.h"

#include <math.h>
#include <stdio.h>

int main(void)
{
  Q1Swing_Tuning tuning = {0};
  if (scanf("%f %f %f %f %f %f %f", &tuning.startup_kp, &tuning.startup_kd,
            &tuning.amplitude_kp, &tuning.amplitude_ki, &tuning.amplitude_kd,
            &tuning.damping_compensation, &tuning.target_amplitude_deg) != 7) {
    return 1;
  }

  Q1Swing swing = {0};
  unsigned time_ms;
  float roll, pitch, rate;
  if (scanf("%u %f %f %f", &time_ms, &roll, &pitch, &rate) != 4 ||
      time_ms != 0U || !Q1Swing_StartTuned(&swing, &tuning, 0U, roll, pitch, rate)) {
    return 2;
  }

  while (scanf("%u %f %f %f", &time_ms, &roll, &pitch, &rate) == 4) {
    Q1Swing_Step(&swing, time_ms, roll, pitch, rate, true, false);
    const unsigned right = swing.pwm[0];
    const unsigned left = swing.pwm[1];
    if (right > APP_Q1_PWM_MAX_PERMILLE || left > APP_Q1_PWM_MAX_PERMILLE ||
        (right && left) || swing.pwm[2] != 0U || swing.pwm[3] != 0U ||
        !isfinite(swing.center_estimate_deg) || !isfinite(swing.center_output) ||
        fabsf(swing.center_output) > APP_Q1_CENTER_LIMIT_PERMILLE + 0.001F) {
      return 3;
    }
    if (time_ms < APP_Q1_DURATION_MS && swing.state != Q1_SWING_RUNNING) return 4;
    if (time_ms == APP_Q1_DURATION_MS &&
        (swing.state != Q1_SWING_FINISHED || swing.reason != Q1_SWING_TIME_UP ||
         right || left || swing.center_output != 0.0F)) return 5;
    printf("%u %u %d %.5f %.5f\n", time_ms, (unsigned)swing.control_phase,
           (int)right - (int)left, (double)swing.center_estimate_deg,
           (double)swing.center_output);
  }
  return swing.reason == Q1_SWING_TIME_UP ? 0 : 6;
}
