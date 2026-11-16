#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>

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
  uint16_t adc_raw;
  uint32_t pin_mv;
  uint32_t current_ma;
  uint32_t filtered_ma;
  uint8_t overrange;
  uint8_t valid;
} CurrentSense_Result;

void CurrentSense_Reset(CurrentSense_State *state);
bool CurrentSense_Update(CurrentSense_State *state,
                         const CurrentSense_Config *config,
                         uint16_t adc_raw, uint32_t vdda_mv,
                         bool sample_valid, CurrentSense_Result *result);

#endif
