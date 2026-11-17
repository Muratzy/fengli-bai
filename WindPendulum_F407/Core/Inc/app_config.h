#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Select one output owner. The bench image never starts a competition task. */
#ifndef APP_MOTOR1_TEST_ENABLED
#define APP_MOTOR1_TEST_ENABLED 1
#endif

#ifndef APP_OLED_ENABLED
#define APP_OLED_ENABLED 0
#endif

/* Read-only bench telemetry, independent from the full button/menu UI. */
#ifndef APP_MOTOR1_STATUS_OLED_ENABLED
#define APP_MOTOR1_STATUS_OLED_ENABLED 1
#endif

#define APP_MOTOR1_OLED_PERIOD_MS 100U

/* User calibration: VREFINT raw code normalized to VDDA=3300 mV.
 * Zero selects a nominal 1.210 V estimate, not a factory calibration claim. */
#define APP_ADC_VREFINT_CAL_RAW 0U
#define APP_ADC_VREFINT_NOMINAL_MV 1210U

/* INA240A2 / 5 mOhm / grounded REF. Zero is an output offset in uV,
 * not the Arduino library's ADC-reference/2 default. */
#define APP_CURRENT_SAMPLE_COUNT 8U
#define APP_CURRENT_ZERO_UV 0U
#define APP_CURRENT_GAIN_PERMILLE 1000U

#if (APP_CURRENT_SAMPLE_COUNT < 1U) || (APP_CURRENT_SAMPLE_COUNT > 250U)
#error "Current ADC averaging requires 1..250 samples."
#endif

#if APP_MOTOR1_TEST_ENABLED && APP_OLED_ENABLED
#error "Motor 1 bench testing owns the buttons; disable the OLED UI."
#endif

#endif
