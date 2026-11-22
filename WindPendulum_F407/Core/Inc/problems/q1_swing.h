#ifndef Q1_SWING_H
#define Q1_SWING_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  Q1_SWING_IDLE = 0,
  Q1_SWING_RUNNING,
  Q1_SWING_FINISHED,
  Q1_SWING_STOPPED,
  Q1_SWING_FAULT
} Q1Swing_State;

typedef enum {
  Q1_SWING_NONE = 0,
  Q1_SWING_TIME_UP,
  Q1_SWING_USER_STOP,
  Q1_SWING_SAMPLE_STALE,
  Q1_SWING_ANGLE_LIMIT,
  Q1_SWING_NO_TURN,
  Q1_SWING_EN_LOW,
  Q1_SWING_NOT_CENTERED
} Q1Swing_Reason;

typedef struct {
  Q1Swing_State state;
  Q1Swing_Reason reason;
  uint32_t started_ms;
  uint32_t last_step_ms;
  uint32_t last_turn_ms;
  uint32_t switches;
  int direction;
  bool turn_armed;
  bool boosting;
  float peak_deg;
  float left_peak_deg;
  float right_peak_deg;
  float predicted_peak_deg;
  uint32_t rise_remainder;
  uint16_t pwm[4];
} Q1Swing;

void Q1Swing_Stop(Q1Swing *swing, Q1Swing_Reason reason);
bool Q1Swing_Start(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                    float pitch_deg, float pitch_rate_dps);
void Q1Swing_Step(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                   float pitch_deg, float pitch_rate_dps, bool sample_valid,
                   bool stop_requested);

#endif
