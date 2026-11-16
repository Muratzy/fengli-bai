#include "diagnostics/motor1_test.h"

#include <stddef.h>

void Motor1Test_Init(Motor1Test *test, uint32_t now_ms)
{
  if (test != NULL)
  {
    *test = (Motor1Test){0};
    test->phase_started_ms = now_ms;
    test->last_step_ms = now_ms;
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
    test->duty_permille = 0U;
  }
}

void Motor1Test_Step(Motor1Test *test, uint32_t now_ms,
                    bool run_pressed, bool stop_pressed)
{
  uint32_t elapsed_ms;

  if (test == NULL)
  {
    return;
  }

  test->duty_permille = 0U;
  if ((test->state == MOTOR1_TEST_RUNNING) &&
      ((uint32_t)(now_ms - test->last_step_ms) >
       MOTOR1_TEST_MAX_STEP_GAP_MS))
  {
    Motor1Test_Stop(test, now_ms, MOTOR1_TEST_LOOP_DELAY);
  }
  test->last_step_ms = now_ms;

  if (stop_pressed)
  {
    Motor1Test_Stop(test, now_ms, MOTOR1_TEST_STOP_KEY);
    return;
  }

  elapsed_ms = now_ms - test->phase_started_ms;
  switch (test->state)
  {
    case MOTOR1_TEST_WAIT_RELEASE:
      if (run_pressed)
      {
        test->phase_started_ms = now_ms;
      }
      else if (elapsed_ms >= MOTOR1_TEST_RELEASE_MS)
      {
        test->state = MOTOR1_TEST_READY;
      }
      break;

    case MOTOR1_TEST_READY:
      if (run_pressed)
      {
        test->phase_started_ms = now_ms;
        test->state = MOTOR1_TEST_HOLD;
        test->run_elapsed_ms = 0U;
      }
      break;

    case MOTOR1_TEST_HOLD:
      if (!run_pressed)
      {
        Motor1Test_Stop(test, now_ms, MOTOR1_TEST_KEY_RELEASED);
      }
      else if (elapsed_ms >= MOTOR1_TEST_HOLD_MS)
      {
        test->state = MOTOR1_TEST_RUNNING;
        test->phase_started_ms = now_ms;
      }
      break;

    case MOTOR1_TEST_RUNNING:
      test->run_elapsed_ms = elapsed_ms;
      if (!run_pressed)
      {
        Motor1Test_Stop(test, now_ms, MOTOR1_TEST_KEY_RELEASED);
      }
      else if (elapsed_ms >= MOTOR1_TEST_DURATION_MS)
      {
        Motor1Test_Stop(test, now_ms, MOTOR1_TEST_TIME_LIMIT);
      }
      else
      {
        test->duty_permille = (elapsed_ms >= MOTOR1_TEST_RAMP_MS) ?
            MOTOR1_TEST_MAX_DUTY :
            (uint16_t)((elapsed_ms * MOTOR1_TEST_MAX_DUTY) /
                       MOTOR1_TEST_RAMP_MS);
      }
      break;

    default:
      Motor1Test_Init(test, now_ms);
      break;
  }
}
