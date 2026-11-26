#ifndef Q4_BRAKE_H
#define Q4_BRAKE_H

#include "problems/problem_api.h"
#include "problems/q1_swing.h"

typedef enum {
  Q4_WAIT_RELEASE = 0,
  Q4_BRAKING,
  Q4_VERIFY_STOP
} Q4Phase;

typedef struct {
  float roll_deg;
  float pitch_deg;
  float tilt_deg;
  float roll_rate_dps;
  float pitch_rate_dps;
} Q4Sample;

typedef struct {
  PendulumControl controller;
  PendulumControl_Output output;
  ProblemStatusCode state;
  Q1Swing_Reason reason;
  Q4Phase phase;
  uint32_t armed_ms;
  uint32_t last_step_ms;
  uint32_t released_ms;
  uint32_t run_ms;
  uint32_t hold_ms;
  uint32_t settle_ms;
  bool holding;
} Q4Brake;

bool Q4Brake_Arm(Q4Brake *brake, uint32_t now_ms, const Q4Sample *sample);
void Q4Brake_Step(Q4Brake *brake, uint32_t now_ms, const Q4Sample *sample);
void Q4Brake_Stop(Q4Brake *brake, Q1Swing_Reason reason);

#endif
