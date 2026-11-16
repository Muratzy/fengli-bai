#ifndef WIND_PENDULUM_PID_H
#define WIND_PENDULUM_PID_H

#include <stdbool.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float proportional_setpoint_weight;
  float derivative_filter_tau_s;
  float integral_min;
  float integral_max;
  float output_min;
  float output_max;
} PID_Config;

typedef struct {
  PID_Config config;
  float integral;
  float previous_error;
  float previous_measurement;
  float derivative;
  float p_term;
  float i_term;
  float d_term;
  float output;
  bool initialized;
} PID_Controller;

bool PID_Init(PID_Controller *controller, const PID_Config *config);
void PID_Reset(PID_Controller *controller, float measurement,
               float output_seed);
float PID_Update(PID_Controller *controller, float setpoint,
                 float measurement, float dt_s);

#endif
