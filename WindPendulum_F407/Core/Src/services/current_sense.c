#include "services/current_sense.h"

#include <stddef.h>

void CurrentSense_Reset(CurrentSense_State *state)
{
  if (state != NULL)
  {
    *state = (CurrentSense_State){0};
  }
}

bool CurrentSense_Update(CurrentSense_State *state,
                         const CurrentSense_Config *config,
                         uint16_t adc_raw, uint32_t vdda_mv,
                         bool sample_valid, CurrentSense_Result *result)
{
  uint64_t pin_uv;
  uint64_t signal_uv;
  uint64_t denominator;
  uint64_t current_ma;
  uint64_t input_q8;

  if (result == NULL)
  {
    return false;
  }
  *result = (CurrentSense_Result){0};
  result->adc_raw = adc_raw;
  result->overrange = (adc_raw >= 4090U) ? 1U : 0U;
  if ((state == NULL) || (config == NULL) || !sample_valid ||
      (result->overrange != 0U) || (vdda_mv < 1800U) || (vdda_mv > 3600U) ||
      (config->amplifier_gain == 0U) || (config->amplifier_gain > 1000U) ||
      (config->shunt_uohm == 0U) || (config->shunt_uohm > 1000000U) ||
      (config->gain_permille == 0U) || (config->gain_permille > 100000U) ||
      (config->filter_shift == 0U) || (config->filter_shift > 10U))
  {
    return false;
  }
  pin_uv = ((uint64_t)adc_raw * vdda_mv * 1000U + 2047U) / 4095U;
  result->pin_mv = (uint32_t)((pin_uv + 500U) / 1000U);
  signal_uv = (pin_uv > config->zero_uv) ? pin_uv - config->zero_uv : 0U;
  denominator = (uint64_t)config->amplifier_gain * config->shunt_uohm;
  /* uV / (V/V * uOhm) gives A; gain_permille includes the mA factor. */
  current_ma = (signal_uv * config->gain_permille + denominator / 2U) /
               denominator;
  if (current_ma > UINT32_MAX)
  {
    return false;
  }
  result->current_ma = (uint32_t)current_ma;
  input_q8 = current_ma << 8;
  if (!state->filter_initialized)
  {
    state->filtered_q8 = input_q8;
    state->filter_initialized = true;
  }
  else if (input_q8 >= state->filtered_q8)
  {
    state->filtered_q8 += (input_q8 - state->filtered_q8) >> config->filter_shift;
  }
  else
  {
    state->filtered_q8 -= (state->filtered_q8 - input_q8) >> config->filter_shift;
  }
  result->filtered_ma = (uint32_t)((state->filtered_q8 + 128U) >> 8);
  result->valid = 1U;
  return true;
}
