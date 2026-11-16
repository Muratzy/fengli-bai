#ifndef VOLTAGE_SENSE_H
#define VOLTAGE_SENSE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t divider_numerator;
  uint32_t divider_denominator;
  uint32_t gain_permille;
  int32_t offset_mv;
  uint8_t filter_shift;
} VoltageSense_Config;

typedef struct {
  uint64_t filtered_q8;
  bool filter_initialized;
} VoltageSense_State;

typedef struct {
  uint16_t adc_raw;
  uint16_t vrefint_raw;
  uint32_t vdda_mv;
  uint32_t pin_mv;
  uint32_t input_mv;
  uint32_t filtered_mv;
  uint8_t reference_valid;
  uint8_t overrange;
  uint8_t valid;
} VoltageSense_Result;

void VoltageSense_Reset(VoltageSense_State *state);
/* vrefint_cal_raw is an explicitly configured code at VDDA=3300 mV.
 * It may be nominal; sample validity does not imply calibrated accuracy. */
bool VoltageSense_Update(VoltageSense_State *state,
                         const VoltageSense_Config *config,
                         uint16_t adc_raw,
                         uint16_t vrefint_raw,
                         uint16_t vrefint_cal_raw,
                         VoltageSense_Result *result);

#endif
