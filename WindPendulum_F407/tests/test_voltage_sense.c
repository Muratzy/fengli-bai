#include "services/voltage_sense.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
  const VoltageSense_Config direct = {1U, 1U, 1000U, 0, 3U};
  const VoltageSense_Config divider = {11U, 1U, 1000U, 0, 3U};
  VoltageSense_State state = {0};
  VoltageSense_Result result;

  assert(VoltageSense_Update(&state, &direct, 2048U, 1500U, 1500U,
                             &result));
  assert(result.valid == 1U);
  assert(result.vdda_mv == 3300U);
  assert(result.pin_mv == 1650U);
  assert(result.input_mv == 1650U);
  assert(result.filtered_mv == 1650U);

  assert(VoltageSense_Update(&state, &divider, 2048U, 1500U, 1500U,
                             &result));
  assert(result.input_mv == 18150U);
  assert(result.filtered_mv > 1650U);

  assert(!VoltageSense_Update(&state, &direct, 2048U, 0U, 1500U,
                              &result));
  assert(result.valid == 0U);
  assert(!VoltageSense_Update(&state, &direct, 4095U, 1500U, 1500U,
                              &result));
  assert(result.overrange == 1U);
  assert(result.reference_valid == 1U);
  assert(result.valid == 0U);
  assert(!VoltageSense_Update(&state, &direct, 1000U, 1500U, 1500U,
                              NULL));

  puts("voltage sense algorithm tests passed");
  return 0;
}
