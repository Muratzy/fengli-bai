#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>

#define CURRENT_SENSE_MAX_SAMPLES 250U

typedef struct {
  uint32_t amplifier_gain;
  uint32_t shunt_uohm;
  uint32_t zero_uv;
  uint32_t gain_permille;
  uint8_t filter_shift;
} CurrentSense_Config;

typedef struct {
  uint64_t filtered_q8;
  bool filter_initialized;
} CurrentSense_State;

typedef struct {
  /* Latest raw code is retained separately from the batch mean. */
  uint16_t adc_raw;
  uint16_t sample_count;
  uint32_t adc_average_q8;
  uint32_t pin_uv;
  uint32_t pin_mv;
  uint32_t current_ma;
  uint32_t filtered_ma;
  uint8_t overrange;
  uint8_t valid;
} CurrentSense_Result;

void CurrentSense_Reset(CurrentSense_State *state);
/* A single VDDA estimate must apply to all samples in this short batch.
 * Reject the entire batch if any code clips; never average a clipped code away.
 * Average before zero subtraction and the unidirectional zero clamp. */
bool CurrentSense_UpdateSamples(CurrentSense_State *state,
                                const CurrentSense_Config *config,
                                const uint16_t *adc_samples,
                                uint16_t sample_count, uint32_t vdda_mv,
                                bool samples_valid,
                                CurrentSense_Result *result);
/* Caller must provide acquisition/reference validity and confirm zero current
 * through the shared shunt, not just PWM=0.
 * On success, update the caller's config and discard the old current filter.
 * This does not persist calibration or start a hardware sampling loop. */
bool CurrentSense_CalibrateZero(CurrentSense_State *state,
                                CurrentSense_Config *config,
                                const uint16_t *adc_samples,
                                uint16_t sample_count, uint32_t vdda_mv,
                                bool samples_valid,
                                bool zero_current_confirmed);
/* Compatibility entry point for an already acquired single ADC sample. */
bool CurrentSense_Update(CurrentSense_State *state,
                         const CurrentSense_Config *config,
                         uint16_t adc_raw, uint32_t vdda_mv,
                         bool sample_valid, CurrentSense_Result *result);

#endif
