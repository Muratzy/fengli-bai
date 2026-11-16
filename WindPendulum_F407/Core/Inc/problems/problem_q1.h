#ifndef PROBLEM_Q1_H
#define PROBLEM_Q1_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  PROBLEM_Q1_AXIS_ROLL = 0,
  PROBLEM_Q1_AXIS_PITCH = 1
} ProblemQ1_Axis;

typedef enum {
  PROBLEM_Q1_IDLE = 0,
  PROBLEM_Q1_RUNNING,
  PROBLEM_Q1_COMPLETE,
  PROBLEM_Q1_ABORTED,
  PROBLEM_Q1_BLOCKED
} ProblemQ1_State;

typedef enum {
  PROBLEM_Q1_OK = 0,
  PROBLEM_Q1_INVALID_CONFIG,
  PROBLEM_Q1_IMU_OFFLINE,
  PROBLEM_Q1_NOT_CALIBRATED
} ProblemQ1_Result;

typedef struct {
  float line_length_m;
  float effective_length_m;
  float max_angle_deg;
  float direction_sign;
  uint32_t duration_ms;
  ProblemQ1_Axis axis;
} ProblemQ1_Config;

typedef struct {
  float target_position_m;
  float target_velocity_mps;
  float target_roll_deg;
  float target_pitch_deg;
  bool complete;
} ProblemQ1_Output;

typedef struct {
  ProblemQ1_Config config;
  uint32_t start_time_ms;
  ProblemQ1_State state;
} ProblemQ1;

bool ProblemQ1_Validate(const ProblemQ1_Config *config);
ProblemQ1_Result ProblemQ1_Start(ProblemQ1 *problem,
                                 const ProblemQ1_Config *config,
                                 uint32_t now_ms, bool imu_online,
                                 bool calibration_complete);
bool ProblemQ1_Step(ProblemQ1 *problem, uint32_t now_ms,
                    ProblemQ1_Output *output);
void ProblemQ1_Cancel(ProblemQ1 *problem);

#endif
