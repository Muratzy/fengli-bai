#ifndef PENDULUM_APP_H
#define PENDULUM_APP_H

#include "control/pendulum_control.h"
#include "problems/problem_q1.h"
#include "problems/problem_api.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  PENDULUM_MODE_CALIBRATION = 0, /* Legacy mode value; no startup calibration. */
  PENDULUM_MODE_IMU_FAULT,
  PENDULUM_MODE_READY,
  PENDULUM_MODE_LINE,
  PENDULUM_MODE_CIRCLE,
  PENDULUM_MODE_BRAKE,
  PENDULUM_MODE_DISTURBANCE,
  PENDULUM_MODE_Q1_BLOCKED,
  PENDULUM_MODE_Q2_LINE,
  PENDULUM_MODE_LASER_CAL_CAPTURE
} PendulumMode;

typedef struct {
  int16_t gyro_raw[3];
  int16_t accel_raw[3];
  float gyro_dps[3];
  float accel_g[3];
  float temperature_c;
  uint32_t timestamp_ms;
} ImuSample;

typedef struct {
  uint32_t timestamp_ms;
  ImuSample imu;
  float roll_deg;
  float pitch_deg;
  /* Mechanical frame: roll toward up, pitch toward right, clockwise azimuth. */
  float direction_deg;
  float tilt_deg;
  uint8_t direction_valid;
  float target_roll_deg;
  float target_pitch_deg;
  float q1_target_position_m;
  float q1_target_velocity_mps;
  float accel_weight;
  float pid_p[2];
  float pid_i[2];
  float pid_d[2];
  float pid_output[2];
  /* Requested duty is kept separate from the duty physically applied. */
  uint16_t pwm_permille[4];
  uint16_t pwm_applied_permille[4];
  uint16_t bus_current_adc;
  float pitch_rate_dps;
  float q1_predicted_peak_deg;
  float q1_left_peak_deg;
  float q1_right_peak_deg;
  uint32_t q1_run_ms;
  uint32_t q1_switches;
  uint8_t q1_state;
  uint8_t q1_reason;
  uint8_t en_command;
  uint8_t en_pin;
  uint8_t key_bits;
  uint8_t problem_id;
  uint32_t target_length_mm;
  float roll_rate_dps;
  float target_pitch_rate_dps;
  float filtered_pitch_rate_error_dps;
  uint8_t pitch_integrating;
  float pitch_kp;
  float pitch_ki;
  float pitch_kd;
  uint32_t reference_period_ms;
  uint32_t reference_ramp_ms;
  uint8_t q1_control_phase;
  float q1_amplitude_deg;
  float q1_amplitude_target_deg;
  float q1_amplitude_error_deg;
  float q1_amplitude_output;
  uint16_t q1_amplitude_blend_permille;
  float q1_amplitude_ka;
  float q1_amplitude_kcomp;
  uint32_t q1_vcom_trial_id;
  float q1_center_deg;
  float q1_center_output;
  PendulumMode mode;
} PendulumTelemetry;

typedef struct {
  PendulumTelemetry telemetry;
  uint32_t calibration_samples; /* Legacy field, now zero with fixed offsets. */
  float gyro_bias_dps[3];
  uint8_t imu_online;
  uint8_t calibrated; /* Valid IMU data with the fixed gyro offsets applied. */
} PendulumDisplayState;

void PendulumApp_CreateTasks(void);
void PendulumApp_SetMotorsArmed(uint8_t armed);
/* Task-context API for menus/acceptance switching. ACCEPTED means queued;
 * GetProblemStatus reports actual start, completion or rejection. */
ProblemRequestResult PendulumApp_RequestProblemStart(ProblemId id,
                                                     const ProblemParams *params);
bool PendulumApp_RequestProblemStop(void);
bool PendulumApp_GetProblemStatus(ProblemStatus *status);
bool PendulumApp_RequestQ1Start(void);
bool PendulumApp_RequestQ1Cancel(void);
bool PendulumApp_RequestQ2Start(uint32_t line_length_mm);
bool PendulumApp_RequestQ2Cancel(void);

#endif
