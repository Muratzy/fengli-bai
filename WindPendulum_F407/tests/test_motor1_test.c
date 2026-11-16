#include "diagnostics/motor1_test.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>

static void Advance(Motor1Test *test, uint32_t *now, uint32_t duration,
                    uint8_t keys)
{
  for (uint32_t elapsed = 0U; elapsed < duration; elapsed += 10U)
  {
    *now += 10U;
    Motor1Test_Step(test, *now, (keys & 1U) != 0U,
                    (keys & 2U) != 0U, (keys & 4U) != 0U);
  }
}

static void Ready(Motor1Test *test, uint32_t *now)
{
  Motor1Test_Init(test, *now);
  Advance(test, now, 100U, 0U);
  assert(test->state == MOTOR1_TEST_READY);
}

static void TestBootAndDebounce(void)
{
  Motor1Test test;
  uint32_t now = 0U;
  Motor1Test_Init(&test, now);
  Advance(&test, &now, 500U, 3U);
  assert(test.state == MOTOR1_TEST_WAIT_RELEASE && test.duty_permille == 0U);
  Advance(&test, &now, 100U, 0U);
  assert(test.state == MOTOR1_TEST_READY);
  Advance(&test, &now, 20U, 1U);
  Advance(&test, &now, 40U, 0U);
  assert(test.duty_permille == 0U);
  Advance(&test, &now, 30U, 1U);
  assert(test.duty_permille == 0U);
  Advance(&test, &now, 10U, 1U);
  assert(test.state == MOTOR1_TEST_FIXED && test.duty_permille == 300U);
  Advance(&test, &now, 10000U, 0U);
  assert(test.state == MOTOR1_TEST_FIXED && test.duty_permille == 300U);
}

static void TestRamp(uint32_t initial_time)
{
  Motor1Test test;
  uint32_t now = initial_time;
  Ready(&test, &now);
  Advance(&test, &now, 40U, 2U);
  assert(test.state == MOTOR1_TEST_RAMP && test.duty_permille == 200U);
  for (uint32_t elapsed = 10U; elapsed <= 8000U; elapsed += 10U)
  {
    uint16_t expected = (elapsed >= 6000U) ? 800U :
                        200U + (uint16_t)((elapsed / 100U) * 10U);
    /* Keeping KEY2 pressed must not continually restart the ramp. */
    Advance(&test, &now, 10U, elapsed < 1000U ? 2U : 0U);
    assert(test.state == MOTOR1_TEST_RAMP);
    assert(test.duty_permille == expected);
  }
}

static void TestStopAndSwitch(void)
{
  Motor1Test test;
  uint32_t now = 0U;
  Ready(&test, &now);
  Advance(&test, &now, 40U, 3U);
  assert(test.duty_permille == 0U);
  Advance(&test, &now, 40U, 0U);
  Advance(&test, &now, 40U, 1U);
  Advance(&test, &now, 40U, 0U);
  Advance(&test, &now, 40U, 2U);
  assert(test.state == MOTOR1_TEST_RAMP && test.duty_permille == 200U);
  Advance(&test, &now, 40U, 0U);
  Advance(&test, &now, 40U, 1U);
  assert(test.state == MOTOR1_TEST_FIXED && test.duty_permille == 300U);
  Advance(&test, &now, 10U, 7U);
  assert(test.duty_permille == 0U && test.stop_reason == MOTOR1_TEST_STOP_KEY);
  Advance(&test, &now, 1000U, 1U);
  assert(test.state == MOTOR1_TEST_WAIT_RELEASE && test.duty_permille == 0U);
  Advance(&test, &now, 100U, 0U);
  Advance(&test, &now, 40U, 2U);
  assert(test.state == MOTOR1_TEST_RAMP);
  now += 51U;
  Motor1Test_Step(&test, now, false, true, false);
  assert(test.duty_permille == 0U && test.stop_reason == MOTOR1_TEST_LOOP_DELAY);
  Advance(&test, &now, 1000U, 2U);
  assert(test.state == MOTOR1_TEST_WAIT_RELEASE);
  Advance(&test, &now, 100U, 0U);
  Advance(&test, &now, 40U, 1U);
  Motor1Test_Stop(&test, now, MOTOR1_TEST_EN_LOW);
  assert(test.duty_permille == 0U && test.stop_reason == MOTOR1_TEST_EN_LOW);
}

int main(void)
{
  TestBootAndDebounce();
  TestRamp(0U);
  TestRamp(UINT32_MAX - 200U);
  TestStopAndSwitch();
  Motor1Test_Init(NULL, 0U);
  Motor1Test_Stop(NULL, 0U, MOTOR1_TEST_STOP_KEY);
  Motor1Test_Step(NULL, 0U, false, false, false);
  puts("motor1 button state tests passed");
  return 0;
}
