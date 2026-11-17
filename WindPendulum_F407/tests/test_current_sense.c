#include "services/current_sense.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>

static void TestBatchAverage(void)
{
  CurrentSense_Config config = {50U, 5000U, 0U, 1000U, 3U};
  CurrentSense_State state = {0};
  CurrentSense_Result result;
  const uint16_t fractional[] = {310U, 311U};
  const uint16_t noisy_zero[] = {309U, 311U, 309U, 311U};
  const uint16_t one_amp[] = {341U, 342U};

  /* Retain the half count: rounding the ADC mean first changes the result. */
  assert(CurrentSense_UpdateSamples(&state, &config, fractional, 2U,
                                    3300U, true, &result));
  assert(result.adc_raw == 311U && result.sample_count == 2U);
  assert(result.adc_average_q8 == 79488U);
  assert(result.pin_uv == 250220U && result.current_ma == 1001U);

  /* Symmetric noise around a nonzero baseline must average before clamping. */
  config.zero_uv = 250000U;
  CurrentSense_Reset(&state);
  assert(CurrentSense_UpdateSamples(&state, &config, noisy_zero, 4U,
                                    3300U, true, &result));
  assert(result.current_ma == 0U && result.filtered_ma == 0U);

  /* ADC reference is the supplied VDDA estimate, not INA240's 5 V supply. */
  config.zero_uv = 0U;
  CurrentSense_Reset(&state);
  assert(CurrentSense_UpdateSamples(&state, &config, one_amp, 2U,
                                    3000U, true, &result));
  assert(result.pin_uv == 250183U && result.current_ma == 1001U);
}

static void TestBatchRejection(void)
{
  CurrentSense_Config config = {50U, 5000U, 0U, 1000U, 3U};
  CurrentSense_State state = {0};
  CurrentSense_Result result;
  uint16_t samples[CURRENT_SENSE_MAX_SAMPLES] = {0};
  uint64_t previous_q8;

  for (uint16_t index = 0U; index < CURRENT_SENSE_MAX_SAMPLES; ++index)
  {
    samples[index] = 310U;
  }
  assert(CurrentSense_UpdateSamples(&state, &config, samples,
      CURRENT_SENSE_MAX_SAMPLES, 3300U, true, &result));
  assert(result.current_ma == 999U);
  previous_q8 = state.filtered_q8;
  /* Every clipped position is detected even when the mean would be in range. */
  for (uint16_t index = 0U; index < CURRENT_SENSE_MAX_SAMPLES; ++index)
  {
    samples[index] = 4090U;
    assert(!CurrentSense_UpdateSamples(&state, &config, samples,
        CURRENT_SENSE_MAX_SAMPLES, 3300U, true, &result));
    assert(result.overrange && !result.valid);
    assert(state.filtered_q8 == previous_q8);
    samples[index] = 310U;
  }
  assert(!CurrentSense_UpdateSamples(&state, &config, NULL, 1U,
                                     3300U, true, &result));
  assert(!CurrentSense_UpdateSamples(&state, &config, samples, 0U,
                                     3300U, true, &result));
  assert(!CurrentSense_UpdateSamples(&state, &config, samples,
      CURRENT_SENSE_MAX_SAMPLES + 1U, 3300U, true, &result));
  assert(!CurrentSense_UpdateSamples(&state, &config, samples, 8U,
                                     3300U, false, &result));
  assert(!CurrentSense_UpdateSamples(&state, &config, samples, 8U,
                                     1799U, true, &result));
  assert(!CurrentSense_UpdateSamples(&state, &config, samples, 8U,
                                     3601U, true, &result));
  config.zero_uv = 3300001U;
  assert(!CurrentSense_UpdateSamples(&state, &config, samples, 8U,
                                     3300U, true, &result));
  assert(state.filtered_q8 == previous_q8);
  /* A valid sample range can still overflow mA with an invalid scale choice. */
  config = (CurrentSense_Config){1U, 1U, 0U, 100000U, 3U};
  assert(!CurrentSense_Update(&state, &config, 4000U, 3300U, true, &result));
  assert(!result.valid && state.filtered_q8 == previous_q8);
}

static void TestZeroCalibration(void)
{
  CurrentSense_Config config = {50U, 5000U, 0U, 1000U, 3U};
  CurrentSense_State state = {0};
  CurrentSense_Result result;
  const uint16_t zero_samples[] = {12U, 13U, 12U, 13U};
  const uint16_t invalid_samples[] = {12U, 4095U, 12U, 13U};
  const uint16_t loaded_samples[] = {322U, 323U, 322U, 323U};
  uint64_t previous_q8;

  assert(CurrentSense_Update(&state, &config, 1241U, 3300U, true, &result));
  previous_q8 = state.filtered_q8;
  assert(!CurrentSense_CalibrateZero(&state, &config, zero_samples, 4U,
                                    3300U, true, false));
  assert(!CurrentSense_CalibrateZero(&state, &config, zero_samples, 4U,
                                    3300U, false, true));
  assert(!CurrentSense_CalibrateZero(&state, &config, invalid_samples, 4U,
                                    3300U, true, true));
  assert(!CurrentSense_CalibrateZero(&state, &config, zero_samples, 4U,
                                    0U, true, true));
  assert(!CurrentSense_CalibrateZero(&state, &config, NULL, 4U,
                                    3300U, true, true));
  assert(!CurrentSense_CalibrateZero(NULL, &config, zero_samples, 4U,
                                    3300U, true, true));
  assert(!CurrentSense_CalibrateZero(&state, NULL, zero_samples, 4U,
                                    3300U, true, true));
  assert(config.zero_uv == 0U && state.filtered_q8 == previous_q8);
  assert(CurrentSense_CalibrateZero(&state, &config, zero_samples, 4U,
                                   3300U, true, true));
  assert(config.zero_uv == 10073U);
  assert(!state.filter_initialized && state.filtered_q8 == 0U);
  assert(CurrentSense_UpdateSamples(&state, &config, zero_samples, 4U,
                                    3300U, true, &result));
  assert(result.current_ma == 0U && result.filtered_ma == 0U);
  CurrentSense_Reset(&state);
  assert(CurrentSense_UpdateSamples(&state, &config, loaded_samples, 4U,
                                    3300U, true, &result));
  assert(result.current_ma == 999U && result.filtered_ma == 999U);
  /* Recalibration resets a nonzero filter as well as replacing the baseline. */
  assert(CurrentSense_CalibrateZero(&state, &config, zero_samples, 4U,
                                   3300U, true, true));
  assert(!state.filter_initialized && state.filtered_q8 == 0U);
}

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
  TestBatchAverage();
  TestBatchRejection();
  TestZeroCalibration();
  puts("current sense tests passed");
  return 0;
}
