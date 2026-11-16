#include "services/voltage_sense.h"

#include <stddef.h>

#define ADC_FULL_SCALE             4095U
#define VREFINT_CAL_VREF_MV        3300U
#define VREFINT_CAL_RAW_MIN        500U
#define VREFINT_CAL_RAW_MAX        2500U
#define VREFINT_RAW_MIN            500U
#define VREFINT_RAW_MAX            2500U
#define VDDA_MIN_MV                1800U
#define VDDA_MAX_MV                3600U
#define ADC_OVERRANGE_RAW          4090U
#define VOLTAGE_MAX_FILTER_SHIFT   10U
#define VOLTAGE_MAX_GAIN_PERMILLE  100000U

void VoltageSense_Reset(VoltageSense_State *state)
{
  if (state != NULL)
  {
    *state = (VoltageSense_State){0};
  }
}

bool VoltageSense_Update(VoltageSense_State *state,
                         const VoltageSense_Config *config,
                         uint16_t adc_raw,
                         uint16_t vrefint_raw,
                         uint16_t vrefint_cal_raw,
                         VoltageSense_Result *result)
{
  uint64_t scaled_mv;
  uint64_t corrected_mv;
  uint64_t filtered_input_q8;
  int64_t offset_adjusted_mv;

  if (result == NULL)
  {
    return false;
  }
  *result = (VoltageSense_Result){0};
  result->adc_raw = adc_raw;
  result->vrefint_raw = vrefint_raw;

  if ((state == NULL) || (config == NULL) ||
      (config->divider_numerator == 0U) ||
      (config->divider_denominator == 0U) ||
      (config->gain_permille == 0U) ||
      (config->gain_permille > VOLTAGE_MAX_GAIN_PERMILLE) ||
      (config->filter_shift == 0U) ||
      (config->filter_shift > VOLTAGE_MAX_FILTER_SHIFT) ||
      (vrefint_raw < VREFINT_RAW_MIN) ||
      (vrefint_raw > VREFINT_RAW_MAX) ||
      (vrefint_cal_raw < VREFINT_CAL_RAW_MIN) ||
      (vrefint_cal_raw > VREFINT_CAL_RAW_MAX))
  {
    return false;
  }

  result->vdda_mv = ((uint32_t)vrefint_cal_raw * VREFINT_CAL_VREF_MV +
                     (uint32_t)(vrefint_raw / 2U)) / vrefint_raw;
  if ((result->vdda_mv < VDDA_MIN_MV) ||
      (result->vdda_mv > VDDA_MAX_MV))
  {
    return false;
  }

  result->reference_valid = 1U;
  result->pin_mv = ((uint32_t)adc_raw * result->vdda_mv +
                    (ADC_FULL_SCALE / 2U)) / ADC_FULL_SCALE;
  result->overrange = (adc_raw >= ADC_OVERRANGE_RAW) ? 1U : 0U;
  if (result->overrange != 0U)
  {
    return false;
  }

  scaled_mv = ((uint64_t)result->pin_mv * config->divider_numerator) /
              config->divider_denominator;
  if (scaled_mv > UINT32_MAX)
  {
    return false;
  }
  offset_adjusted_mv = (int64_t)scaled_mv - config->offset_mv;
  corrected_mv = (offset_adjusted_mv > 0) ? (uint64_t)offset_adjusted_mv : 0U;
  corrected_mv = (corrected_mv * config->gain_permille + 500U) / 1000U;
  if (corrected_mv > UINT32_MAX)
  {
    return false;
  }
  result->input_mv = (uint32_t)corrected_mv;

  filtered_input_q8 = corrected_mv << 8;
  if (!state->filter_initialized)
  {
    state->filtered_q8 = filtered_input_q8;
    state->filter_initialized = true;
  }
  else if (filtered_input_q8 >= state->filtered_q8)
  {
    state->filtered_q8 +=
        (filtered_input_q8 - state->filtered_q8) >> config->filter_shift;
  }
  else
  {
    state->filtered_q8 -=
        (state->filtered_q8 - filtered_input_q8) >> config->filter_shift;
  }
  result->filtered_mv = (uint32_t)((state->filtered_q8 + 128U) >> 8);
  result->valid = 1U;
  return true;
}
