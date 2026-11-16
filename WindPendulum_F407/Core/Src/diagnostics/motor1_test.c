#include "diagnostics/motor1_test.h"

#include <stddef.h>

void Motor1Test_Init(Motor1Test *test, uint32_t now_ms)
{
  if (test != NULL)
  {
    *test = (Motor1Test){0};
    test->phase_started_ms = now_ms;
    test->last_step_ms = now_ms;
    test->key_changed_ms = now_ms;
  }
}

void Motor1Test_Stop(Motor1Test *test, uint32_t now_ms,
                    Motor1Test_StopReason reason)
{
  if (test != NULL)
  {
    test->state = MOTOR1_TEST_WAIT_RELEASE;
    test->stop_reason = reason;
    test->phase_started_ms = now_ms;
    test->key_changed_ms = now_ms;
    test->candidate_keys = 0U;
    test->stable_keys = 0U;
    test->duty_permille = 0U;
  }
}

void Motor1Test_Step(Motor1Test *test, uint32_t now_ms,
                    bool fixed_pressed, bool ramp_pressed, bool stop_pressed)
{
  uint8_t keys;
  uint8_t pressed_edges = 0U;
  uint32_t elapsed_ms;

  if (test == NULL)
  {
    return;
  }
  if (((test->state == MOTOR1_TEST_FIXED) ||
       (test->state == MOTOR1_TEST_RAMP)) &&
      ((uint32_t)(now_ms - test->last_step_ms) > MOTOR1_TEST_MAX_STEP_GAP_MS))
  {
    Motor1Test_Stop(test, now_ms, MOTOR1_TEST_LOOP_DELAY);
  }
  test->last_step_ms = now_ms;
  /* Stop takes priority over both start keys and their debounce. */
  if (stop_pressed)
  {
    Motor1Test_Stop(test, now_ms, MOTOR1_TEST_STOP_KEY);
    return;
  }
  keys = (fixed_pressed ? 1U : 0U) | (ramp_pressed ? 2U : 0U);
  if (test->state == MOTOR1_TEST_WAIT_RELEASE)
  {
    if (keys != 0U)
    {
      test->phase_started_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - test->phase_started_ms) >=
             MOTOR1_TEST_RELEASE_MS)
    {
      test->state = MOTOR1_TEST_READY;
      test->candidate_keys = 0U;
      test->stable_keys = 0U;
      test->key_changed_ms = now_ms;
    }
    return;
  }
  if (keys != test->candidate_keys)
  {
    test->candidate_keys = keys;
    test->key_changed_ms = now_ms;
  }
  else if ((uint32_t)(now_ms - test->key_changed_ms) >=
           MOTOR1_TEST_DEBOUNCE_MS)
  {
    pressed_edges = keys & (uint8_t)~test->stable_keys;
    test->stable_keys = keys;
  }
  /* Simultaneous start keys do not select a new mode. */
  if ((keys == 1U) && ((pressed_edges & 1U) != 0U))
  {
    test->state = MOTOR1_TEST_FIXED;
    test->phase_started_ms = now_ms;
  }
  else if ((keys == 2U) && ((pressed_edges & 2U) != 0U))
  {
    test->state = MOTOR1_TEST_RAMP;
    test->phase_started_ms = now_ms;
  }

  elapsed_ms = now_ms - test->phase_started_ms;
  switch (test->state)
  {
    case MOTOR1_TEST_READY:
      test->duty_permille = 0U;
      break;

    case MOTOR1_TEST_FIXED:
      test->run_elapsed_ms = elapsed_ms;
      test->duty_permille = MOTOR1_TEST_FIXED_DUTY;
      break;

    case MOTOR1_TEST_RAMP:
      test->run_elapsed_ms = elapsed_ms;
      if (elapsed_ms >= ((MOTOR1_TEST_MAX_DUTY -
                          MOTOR1_TEST_RAMP_START_DUTY) /
                         MOTOR1_TEST_RAMP_STEP_DUTY) * MOTOR1_TEST_RAMP_STEP_MS)
      {
        test->duty_permille = MOTOR1_TEST_MAX_DUTY;
      }
      else
      {
        test->duty_permille = MOTOR1_TEST_RAMP_START_DUTY +
            (uint16_t)((elapsed_ms / MOTOR1_TEST_RAMP_STEP_MS) *
                       MOTOR1_TEST_RAMP_STEP_DUTY);
      }
      break;

    default:
      Motor1Test_Init(test, now_ms);
      break;
  }
}
