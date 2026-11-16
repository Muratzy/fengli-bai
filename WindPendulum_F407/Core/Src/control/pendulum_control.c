#include "control/pendulum_control.h"

#include <math.h>
#include <stddef.h>

static float PendulumControl_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

bool PendulumControl_Init(PendulumControl *control,
                          const PendulumControl_Config *config)
{
  if ((control == NULL) || (config == NULL) ||
      (config->max_duty_permille > 1000U)) {
    return false;
  }

  *control = (PendulumControl){0};
  for (uint32_t fan = 0U; fan < PENDULUM_FAN_COUNT; ++fan) {
    if (config->base_duty_permille[fan] > config->max_duty_permille) {
      return false;
    }
    for (uint32_t axis = 0U; axis < 2U; ++axis) {
      if (!isfinite(config->fan_mix[fan][axis])) {
        return false;
      }
    }
  }

  control->config = *config;
  if (!PID_Init(&control->axis_pid[0], &config->roll_pid) ||
      !PID_Init(&control->axis_pid[1], &config->pitch_pid)) {
    *control = (PendulumControl){0};
    return false;
  }

  control->initialized = true;
  return true;
}

void PendulumControl_Reset(PendulumControl *control, float roll_deg,
                           float pitch_deg)
{
  if ((control == NULL) || !control->initialized) {
    return;
  }

  PID_Reset(&control->axis_pid[0], roll_deg, 0.0F);
  PID_Reset(&control->axis_pid[1], pitch_deg, 0.0F);
}

bool PendulumControl_Step(PendulumControl *control,
                          float target_roll_deg, float target_pitch_deg,
                          float roll_deg, float pitch_deg, float dt_s,
                          PendulumControl_Output *output)
{
  const float target[2] = {target_roll_deg, target_pitch_deg};
  const float measurement[2] = {roll_deg, pitch_deg};

  if (output == NULL) {
    return false;
  }
  *output = (PendulumControl_Output){0};
  if ((control == NULL) || !control->initialized ||
      !isfinite(target_roll_deg) || !isfinite(target_pitch_deg) ||
      !isfinite(roll_deg) || !isfinite(pitch_deg) ||
      !isfinite(dt_s) || (dt_s <= 0.0F)) {
    return false;
  }

  for (uint32_t axis = 0U; axis < 2U; ++axis) {
    output->axis_output[axis] = PID_Update(&control->axis_pid[axis],
                                           target[axis], measurement[axis],
                                           dt_s);
    output->p[axis] = control->axis_pid[axis].p_term;
    output->i[axis] = control->axis_pid[axis].i_term;
    output->d[axis] = control->axis_pid[axis].d_term;
  }

  output->pwm_ready = control->config.mixer_calibrated;
  if (!output->pwm_ready) {
    return true;
  }

  for (uint32_t fan = 0U; fan < PENDULUM_FAN_COUNT; ++fan) {
    float duty = (float)control->config.base_duty_permille[fan];
    duty += control->config.fan_mix[fan][0] * output->axis_output[0];
    duty += control->config.fan_mix[fan][1] * output->axis_output[1];
    if (!isfinite(duty)) {
      *output = (PendulumControl_Output){0};
      return false;
    }
    duty = PendulumControl_Clamp(duty, 0.0F,
                                 (float)control->config.max_duty_permille);
    output->pwm_permille[fan] = (uint16_t)(duty + 0.5F);
  }
  return true;
}
