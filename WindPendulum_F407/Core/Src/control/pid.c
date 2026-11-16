#include "control/pid.h"

#include <math.h>
#include <stddef.h>

static float PID_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

static bool PID_ConfigIsValid(const PID_Config *config)
{
  return (config != NULL) &&
         isfinite(config->kp) && isfinite(config->ki) &&
         isfinite(config->kd) &&
         isfinite(config->proportional_setpoint_weight) &&
         isfinite(config->derivative_filter_tau_s) &&
         isfinite(config->integral_min) && isfinite(config->integral_max) &&
         isfinite(config->output_min) && isfinite(config->output_max) &&
         (config->proportional_setpoint_weight >= 0.0F) &&
         (config->proportional_setpoint_weight <= 1.0F) &&
         (config->derivative_filter_tau_s >= 0.0F) &&
         (config->integral_min <= config->integral_max) &&
         (config->output_min < config->output_max);
}

bool PID_Init(PID_Controller *controller, const PID_Config *config)
{
  if (controller == NULL) {
    return false;
  }

  *controller = (PID_Controller){0};
  if (!PID_ConfigIsValid(config)) {
    return false;
  }

  controller->config = *config;
  controller->initialized = true;
  return true;
}

void PID_Reset(PID_Controller *controller, float measurement,
               float output_seed)
{
  if ((controller == NULL) || !PID_ConfigIsValid(&controller->config)) {
    return;
  }

  if (!isfinite(measurement)) {
    measurement = 0.0F;
  }
  if (!isfinite(output_seed)) {
    output_seed = 0.0F;
  }

  controller->integral = PID_Clamp(output_seed,
                                   controller->config.integral_min,
                                   controller->config.integral_max);
  controller->previous_error = 0.0F;
  controller->previous_measurement = measurement;
  controller->derivative = 0.0F;
  controller->p_term = 0.0F;
  controller->i_term = controller->integral;
  controller->d_term = 0.0F;
  controller->output = PID_Clamp(output_seed,
                                 controller->config.output_min,
                                 controller->config.output_max);
  controller->initialized = true;
}

float PID_Update(PID_Controller *controller, float setpoint,
                 float measurement, float dt_s)
{
  float error;
  float proportional_input;
  float derivative_input;
  float filter_alpha;
  float candidate_integral;
  float integral_delta;
  float unconstrained_output;

  if ((controller == NULL) || !controller->initialized) {
    return 0.0F;
  }

  if (!isfinite(setpoint) || !isfinite(measurement) ||
      !isfinite(dt_s) || (dt_s <= 0.0F)) {
    PID_Reset(controller, 0.0F, 0.0F);
    return 0.0F;
  }

  error = setpoint - measurement;
  proportional_input =
      (controller->config.proportional_setpoint_weight * setpoint) -
      measurement;
  controller->p_term = controller->config.kp * proportional_input;

  derivative_input = -(measurement - controller->previous_measurement) / dt_s;
  if (controller->config.derivative_filter_tau_s > 0.0F) {
    filter_alpha = dt_s / (controller->config.derivative_filter_tau_s + dt_s);
  } else {
    filter_alpha = 1.0F;
  }
  controller->derivative +=
      filter_alpha * (derivative_input - controller->derivative);
  controller->d_term = controller->config.kd * controller->derivative;

  integral_delta = 0.5F * controller->config.ki *
                   (error + controller->previous_error) * dt_s;
  candidate_integral = controller->integral + integral_delta;
  candidate_integral = PID_Clamp(candidate_integral,
                                 controller->config.integral_min,
                                 controller->config.integral_max);
  unconstrained_output = controller->p_term + candidate_integral +
                         controller->d_term;

  if ((unconstrained_output > controller->config.output_max) &&
      (integral_delta > 0.0F)) {
    candidate_integral = PID_Clamp(
        controller->config.output_max - controller->p_term -
            controller->d_term,
        controller->config.integral_min, controller->config.integral_max);
  } else if ((unconstrained_output < controller->config.output_min) &&
             (integral_delta < 0.0F)) {
    candidate_integral = PID_Clamp(
        controller->config.output_min - controller->p_term -
            controller->d_term,
        controller->config.integral_min, controller->config.integral_max);
  }

  unconstrained_output = controller->p_term + candidate_integral +
                         controller->d_term;
  if (!isfinite(unconstrained_output)) {
    PID_Reset(controller, measurement, 0.0F);
    return 0.0F;
  }

  controller->integral = candidate_integral;
  controller->i_term = candidate_integral;
  controller->output = PID_Clamp(unconstrained_output,
                                 controller->config.output_min,
                                 controller->config.output_max);
  controller->previous_error = error;
  controller->previous_measurement = measurement;
  return controller->output;
}
