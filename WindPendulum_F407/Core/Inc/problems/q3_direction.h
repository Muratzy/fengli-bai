#ifndef Q3_DIRECTION_H
#define Q3_DIRECTION_H

#include "problems/problem_api.h"
#include "problems/q1_swing.h"

typedef struct {
  PID_Controller cross_pid;
  float up;
  float right;
  float target_roll_deg;
  float target_pitch_deg;
  float target_pitch_rate_dps;
  PendulumControl_Output output;
} Q3Direction;

bool Q3Direction_Start(Q3Direction *line, Q1Swing *along,
                       const ProblemParams *params, uint32_t now_ms,
                       float roll_deg, float pitch_deg,
                       float roll_rate_dps, float pitch_rate_dps);
bool Q3Direction_StartTuned(Q3Direction *line, Q1Swing *along,
                            const ProblemParams *params,
                            const Q1Swing_Tuning *tuning, uint32_t now_ms,
                            float roll_deg, float pitch_deg,
                            float roll_rate_dps, float pitch_rate_dps);
void Q3Direction_Step(Q3Direction *line, Q1Swing *along, uint32_t now_ms,
                      float roll_deg, float pitch_deg,
                      float roll_rate_dps, float pitch_rate_dps);

#endif
