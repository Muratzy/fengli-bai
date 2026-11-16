#ifndef MOTOR1_TEST_H
#define MOTOR1_TEST_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR1_TEST_RELEASE_MS       100U
#define MOTOR1_TEST_HOLD_MS         1000U
#define MOTOR1_TEST_RAMP_MS         1000U
#define MOTOR1_TEST_DURATION_MS     3000U
#define MOTOR1_TEST_MAX_DUTY         300U
#define MOTOR1_TEST_MAX_STEP_GAP_MS   50U

typedef enum {
  MOTOR1_TEST_WAIT_RELEASE = 0,
  MOTOR1_TEST_READY,
  MOTOR1_TEST_HOLD,
  MOTOR1_TEST_RUNNING
} Motor1Test_State;

typedef enum {
  MOTOR1_TEST_BOOT = 0,
  MOTOR1_TEST_KEY_RELEASED,
  MOTOR1_TEST_STOP_KEY,
  MOTOR1_TEST_TIME_LIMIT,
  MOTOR1_TEST_LOOP_DELAY,
  MOTOR1_TEST_EN_LOW
} Motor1Test_StopReason;

typedef struct {
  Motor1Test_State state;
  Motor1Test_StopReason stop_reason;
  uint32_t phase_started_ms;
  uint32_t last_step_ms;
  uint32_t run_elapsed_ms;
  uint16_t duty_permille;
} Motor1Test;

void Motor1Test_Init(Motor1Test *test, uint32_t now_ms);
void Motor1Test_Stop(Motor1Test *test, uint32_t now_ms,
                    Motor1Test_StopReason reason);
void Motor1Test_Step(Motor1Test *test, uint32_t now_ms,
                    bool run_pressed, bool stop_pressed);
void Motor1Test_CreateTasks(void);

#endif
