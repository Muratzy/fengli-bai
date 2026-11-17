#include "services/current_sense.h"

#include <stddef.h>

#define CURRENT_ADC_FULL_SCALE    4095U
#define CURRENT_ADC_CLIP_RAW      4090U
#define CURRENT_VDDA_MIN_MV       1800U
#define CURRENT_VDDA_MAX_MV       3600U

/* INA240 transfer function, ADC averaging and zero-offset calibration:
 * reference 7Semi-INA240, revision 08e91db; see ThirdPartyNotices.md.
 * Keep the board's grounded REF and integer units instead of Arduino defaults. */
static bool CurrentSense_ConfigValid(const CurrentSense_Config *config)
{
  return (config != NULL) &&
      (config->amplifier_gain > 0U) && (config->amplifier_gain <= 1000U) &&
      (config->shunt_uohm > 0U) && (config->shunt_uohm <= 1000000U) &&
      (config->gain_permille > 0U) && (config->gain_permille <= 100000U) &&
      (config->filter_shift > 0U) && (config->filter_shift <= 10U);
}

static bool CurrentSense_ReadSamples(const uint16_t *adc_samples,
                                     uint16_t sample_count, uint32_t vdda_mv,
                                     bool samples_valid,
                                     CurrentSense_Result *result)
{
  uint64_t adc_sum = 0U;
  uint64_t denominator;

  if ((adc_samples == NULL) || (sample_count == 0U) ||
      (sample_count > CURRENT_SENSE_MAX_SAMPLES))
  {
    return false;
  }
  result->adc_raw = adc_samples[sample_count - 1U];
  result->sample_count = sample_count;
  for (uint16_t index = 0U; index < sample_count; ++index)
  {
    if (adc_samples[index] >= CURRENT_ADC_CLIP_RAW)
    {
      result->overrange = 1U;
    }
    adc_sum += adc_samples[index];
  }
  if (!samples_valid || (result->overrange != 0U) ||
      (vdda_mv < CURRENT_VDDA_MIN_MV) || (vdda_mv > CURRENT_VDDA_MAX_MV))
  {
    return false;
  }
  denominator = (uint64_t)sample_count * CURRENT_ADC_FULL_SCALE;
  /* Retain fractional ADC counts until the final uV conversion. */
  result->adc_average_q8 = (uint32_t)((adc_sum * 256U + sample_count / 2U) /
                                     sample_count);
  result->pin_uv = (uint32_t)((adc_sum * vdda_mv * 1000U + denominator / 2U) /
                             denominator);
  result->pin_mv = (result->pin_uv + 500U) / 1000U;
  return true;
}

void CurrentSense_Reset(CurrentSense_State *state)
{
  if (state != NULL)
  {
    *state = (CurrentSense_State){0};
  }
}

bool CurrentSense_UpdateSamples(CurrentSense_State *state,
                                const CurrentSense_Config *config,
                                const uint16_t *adc_samples,
                                uint16_t sample_count, uint32_t vdda_mv,
                                bool samples_valid,
                                CurrentSense_Result *result)
{
  uint64_t signal_uv;
  uint64_t denominator;
  uint64_t current_ma;
  uint64_t input_q8;

  if (result == NULL)
  {
    return false;
  }
  *result = (CurrentSense_Result){0};
  if (!CurrentSense_ReadSamples(adc_samples, sample_count, vdda_mv,
                                samples_valid, result) ||
      (state == NULL) || !CurrentSense_ConfigValid(config) ||
      (config->zero_uv > vdda_mv * 1000U))
  {
    return false;
  }
  signal_uv = (result->pin_uv > config->zero_uv) ?
      result->pin_uv - config->zero_uv : 0U;
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

bool CurrentSense_CalibrateZero(CurrentSense_State *state,
                                CurrentSense_Config *config,
                                const uint16_t *adc_samples,
                                uint16_t sample_count, uint32_t vdda_mv,
                                bool samples_valid,
                                bool zero_current_confirmed)
{
  CurrentSense_Result measurement = {0};

  if ((state == NULL) || !CurrentSense_ConfigValid(config) ||
      !zero_current_confirmed ||
      !CurrentSense_ReadSamples(adc_samples, sample_count, vdda_mv,
                                samples_valid, &measurement))
  {
    return false;
  }
  config->zero_uv = measurement.pin_uv;
  CurrentSense_Reset(state);
  return true;
}

bool CurrentSense_Update(CurrentSense_State *state,
                         const CurrentSense_Config *config,
                         uint16_t adc_raw, uint32_t vdda_mv,
                         bool sample_valid, CurrentSense_Result *result)
{
  return CurrentSense_UpdateSamples(state, config, &adc_raw, 1U, vdda_mv,
                                    sample_valid, result);
}
