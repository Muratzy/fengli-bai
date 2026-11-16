#include "services/current_sense.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>

int main(void)
{
  CurrentSense_Config config = {50U, 5000U, 0U, 1000U, 3U};
  CurrentSense_State state = {0};
  CurrentSense_Result result;
  uint64_t previous_q8;

  /* 310 counts at 3.3 V is approximately 250 mV -> 1 A. */
  assert(CurrentSense_Update(&state, &config, 310U, 3300U, true, &result));
  assert(result.current_ma >= 998U && result.current_ma <= 1001U);
  assert(result.filtered_ma == result.current_ma);
  assert(CurrentSense_Update(&state, &config, 1241U, 3300U, true, &result));
  assert(result.current_ma >= 3999U && result.current_ma <= 4002U);
  assert(result.filtered_ma > 1000U && result.filtered_ma < 1400U);
  previous_q8 = state.filtered_q8;
  assert(!CurrentSense_Update(&state, &config, 4095U, 3300U, true, &result));
  assert(result.overrange && !result.valid && result.adc_raw == 4095U);
  assert(state.filtered_q8 == previous_q8);
  assert(!CurrentSense_Update(&state, &config, 310U, 0U, true, &result));
  assert(!CurrentSense_Update(&state, &config, 310U, 3300U, false, &result));
  assert(!CurrentSense_Update(NULL, &config, 310U, 3300U, true, &result));
  assert(!CurrentSense_Update(&state, NULL, 310U, 3300U, true, &result));
  assert(!CurrentSense_Update(&state, &config, 310U, 3300U, true, NULL));

  CurrentSense_Reset(&state);
  config.zero_uv = 250000U;
  assert(CurrentSense_Update(&state, &config, 310U, 3300U, true, &result));
  assert(result.current_ma == 0U && result.filtered_ma == 0U);
  config.zero_uv = 0U;
  config.gain_permille = 1100U;
  CurrentSense_Reset(&state);
  assert(CurrentSense_Update(&state, &config, 310U, 3300U, true, &result));
  assert(result.current_ma >= 1098U && result.current_ma <= 1101U);
  assert(CurrentSense_Update(&state, &config, 0U, 3300U, true, &result));
  assert(result.filtered_ma > 0U && result.filtered_ma < 1100U);
  config.shunt_uohm = 0U;
  assert(!CurrentSense_Update(&state, &config, 310U, 3300U, true, &result));
  puts("current sense tests passed");
  return 0;
}
