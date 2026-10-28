#ifndef PENDULUM_APP_H
#define PENDULUM_APP_H

#include <stdint.h>

typedef enum {
  PENDULUM_MODE_CALIBRATION = 0,
  PENDULUM_MODE_IMU_FAULT,
  PENDULUM_MODE_READY,
  PENDULUM_MODE_LINE,
  PENDULUM_MODE_CIRCLE,
  PENDULUM_MODE_BRAKE,
  PENDULUM_MODE_DISTURBANCE
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
  float accel_weight;
  float pid_p[2];
  float pid_i[2];
  float pid_d[2];
  float pid_output[2];
  uint16_t pwm_permille[4];
  uint16_t bus_current_adc;
  PendulumMode mode;
} PendulumTelemetry;

typedef struct {
  PendulumTelemetry telemetry;
  uint32_t calibration_samples;
  float gyro_bias_dps[3];
  uint8_t imu_online;
  uint8_t calibrated;
} PendulumDisplayState;

void PendulumApp_CreateTasks(void);
void PendulumApp_SetMotorsArmed(uint8_t armed);

#endif
