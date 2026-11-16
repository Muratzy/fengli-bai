#include "diagnostics/motor1_test.h"

#include <assert.h>
#include <stdio.h>

static uint32_t EnterRun(Motor1Test *test, uint32_t now_ms)
{
  Motor1Test_Init(test, now_ms);
  Motor1Test_Step(test, now_ms + 100U, false, false);
  assert(test->state == MOTOR1_TEST_READY);
  Motor1Test_Step(test, now_ms + 110U, true, false);
  assert(test->state == MOTOR1_TEST_HOLD);
  Motor1Test_Step(test, now_ms + 1109U, true, false);
  assert(test->duty_permille == 0U);
  Motor1Test_Step(test, now_ms + 1110U, true, false);
  assert(test->state == MOTOR1_TEST_RUNNING);
  assert(test->duty_permille == 0U);
  return now_ms + 1110U;
}

static void TestHeldOnBootDoesNotStart(void)
{
  Motor1Test test;
  Motor1Test_Init(&test, 0U);
  for (uint32_t time = 0U; time <= 10000U; time += 10U)
  {
    Motor1Test_Step(&test, time, true, false);
    assert(test.state == MOTOR1_TEST_WAIT_RELEASE);
    assert(test.duty_permille == 0U);
  }
  Motor1Test_Step(&test, 10099U, false, false);
  assert(test.state == MOTOR1_TEST_WAIT_RELEASE);
  Motor1Test_Step(&test, 10100U, false, false);
  assert(test.state == MOTOR1_TEST_READY);
}

static void TestRampTimeoutAndReleaseToRestart(void)
{
  Motor1Test test;
  uint32_t start = EnterRun(&test, 0U);
  uint16_t previous_duty = 0U;
  for (uint32_t elapsed = 10U; elapsed < 3000U; elapsed += 10U)
  {
    Motor1Test_Step(&test, start + elapsed, true, false);
    assert(test.state == MOTOR1_TEST_RUNNING);
    assert(test.duty_permille <= 300U);
    assert(test.duty_permille >= previous_duty);
    previous_duty = test.duty_permille;
    if (elapsed >= 1000U)
    {
      assert(test.duty_permille == 300U);
    }
  }
  Motor1Test_Step(&test, start + 3000U, true, false);
  assert(test.duty_permille == 0U);
  assert(test.stop_reason == MOTOR1_TEST_TIME_LIMIT);
  for (uint32_t elapsed = 3010U; elapsed < 8000U; elapsed += 10U)
  {
    Motor1Test_Step(&test, start + elapsed, true, false);
    assert(test.state == MOTOR1_TEST_WAIT_RELEASE);
    assert(test.duty_permille == 0U);
  }
}

static void TestStopsAndMissedDeadline(void)
{
  Motor1Test test;
  uint32_t start = EnterRun(&test, 100U);
  Motor1Test_Step(&test, start + 10U, true, false);
  assert(test.duty_permille > 0U);
  Motor1Test_Step(&test, start + 20U, false, false);
  assert(test.duty_permille == 0U);
  assert(test.stop_reason == MOTOR1_TEST_KEY_RELEASED);

  start = EnterRun(&test, 5000U);
  Motor1Test_Step(&test, start + 10U, true, false);
  Motor1Test_Step(&test, start + 20U, true, true);
  assert(test.duty_permille == 0U);
  assert(test.stop_reason == MOTOR1_TEST_STOP_KEY);

  start = EnterRun(&test, 10000U);
  Motor1Test_Step(&test, start + 10U, true, false);
  Motor1Test_Step(&test, start + 61U, true, false);
  assert(test.duty_permille == 0U);
  assert(test.stop_reason == MOTOR1_TEST_LOOP_DELAY);
}

static void TestMillisecondWraparound(void)
{
  Motor1Test test;
  uint32_t start = EnterRun(&test, UINT32_MAX - 500U);
  Motor1Test_Step(&test, start + 10U, true, false);
  assert(test.state == MOTOR1_TEST_RUNNING);
  assert(test.duty_permille == 3U);
  Motor1Test_Step(&test, start + 20U, false, false);
  assert(test.duty_permille == 0U);
}

int main(void)
{
  TestHeldOnBootDoesNotStart();
  TestRampTimeoutAndReleaseToRestart();
  TestStopsAndMissedDeadline();
  TestMillisecondWraparound();
  puts("motor1 bench state tests passed");
  return 0;
}
