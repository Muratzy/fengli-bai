#ifndef PENDULUM_CONTROL_H
#define PENDULUM_CONTROL_H

#include "control/pid.h"

#include <stdbool.h>
#include <stdint.h>

#define PENDULUM_FAN_COUNT 4U
#define PENDULUM_AXIS_ROLL 0U
#define PENDULUM_AXIS_PITCH 1U
#define PENDULUM_DISABLE_ROLL 1U
#define PENDULUM_DISABLE_PITCH 2U

/* Output indices are motor-board PWM1..PWM4, soldered to TIM1_CH1..CH4. */

typedef struct {
  PID_Config roll_pid;
  PID_Config pitch_pid;
  float fan_mix[PENDULUM_FAN_COUNT][2];
  uint16_t base_duty_permille[PENDULUM_FAN_COUNT];
  uint16_t max_duty_permille;
  bool mixer_calibrated;
  uint8_t disabled_axis_mask; /* Zero keeps both axes enabled for legacy users. */
} PendulumControl_Config;

typedef struct {
  float target_deg[2];
  float target_rate_dps[2];
  float angle_deg[2];
  float rate_dps[2];
  float dt_s;
  bool integrate[2];
} PendulumControl_Input;

typedef struct {
  float p[2];
  float i[2];
  float d[2];
  float axis_output[2];
  uint16_t pwm_permille[PENDULUM_FAN_COUNT];
  bool pwm_ready;
} PendulumControl_Output;

typedef struct {
  PendulumControl_Config config;
  PID_Controller axis_pid[2];
  bool initialized;
} PendulumControl;

bool PendulumControl_Init(PendulumControl *control,
                          const PendulumControl_Config *config);
void PendulumControl_Reset(PendulumControl *control, float roll_deg,
                           float pitch_deg);
bool PendulumControl_Step(PendulumControl *control,
                          float target_roll_deg, float target_pitch_deg,
                          float roll_deg, float pitch_deg, float dt_s,
                          PendulumControl_Output *output);
bool PendulumControl_StepWithRates(PendulumControl *control,
                                   const PendulumControl_Input *input,
                                   PendulumControl_Output *output);

#endif
