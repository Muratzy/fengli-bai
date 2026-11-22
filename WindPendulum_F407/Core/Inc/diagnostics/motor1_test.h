#ifndef MOTOR1_TEST_H
#define MOTOR1_TEST_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR1_TEST_RELEASE_MS       100U
#define MOTOR1_TEST_DEBOUNCE_MS       30U
#define MOTOR1_TEST_AUTO_RAMP_MS   10000U
#define MOTOR1_TEST_AUTO_HOLD_MS    5000U
#define MOTOR1_TEST_RAMP_START_DUTY  200U
#define MOTOR1_TEST_RAMP_STEP_DUTY    10U
#define MOTOR1_TEST_RAMP_STEP_MS     100U
#define MOTOR1_TEST_RAMP_MAX_DUTY    800U
#define MOTOR1_TEST_MAX_DUTY        1000U
#define MOTOR1_TEST_MAX_STEP_GAP_MS   50U

typedef enum {
  MOTOR1_TEST_WAIT_RELEASE = 0,
  MOTOR1_TEST_READY,
  MOTOR1_TEST_AUTO,
  MOTOR1_TEST_RAMP
} Motor1Test_State;

typedef enum {
  MOTOR1_TEST_BOOT = 0,
  MOTOR1_TEST_STOP_KEY,
  MOTOR1_TEST_LOOP_DELAY,
  MOTOR1_TEST_EN_LOW,
  MOTOR1_TEST_AUTO_DONE
} Motor1Test_StopReason;

typedef struct {
  Motor1Test_State state;
  Motor1Test_StopReason stop_reason;
  uint32_t phase_started_ms;
  uint32_t last_step_ms;
  uint32_t run_elapsed_ms;
  uint32_t full_duty_started_ms;
  uint32_t key_changed_ms;
  uint16_t duty_permille;
  uint8_t candidate_keys;
  uint8_t stable_keys;
  bool full_duty_reached;
} Motor1Test;

void Motor1Test_Init(Motor1Test *test, uint32_t now_ms);
void Motor1Test_Stop(Motor1Test *test, uint32_t now_ms,
                    Motor1Test_StopReason reason);
void Motor1Test_Step(Motor1Test *test, uint32_t now_ms,
                    bool auto_pressed, bool ramp_pressed, bool stop_pressed);
void Motor1Test_CreateTasks(void);

#endif
