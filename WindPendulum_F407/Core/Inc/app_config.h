#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Select one output owner. The bench image never starts a competition task. */
#ifndef APP_MOTOR1_TEST_ENABLED
#define APP_MOTOR1_TEST_ENABLED 0
#endif

#ifndef APP_LASER_CAL_CAPTURE_ENABLED
#define APP_LASER_CAL_CAPTURE_ENABLED 0
#endif

#ifndef APP_Q2_UI_ENABLED
#define APP_Q2_UI_ENABLED 1
#endif

#ifndef APP_OLED_ENABLED
#if APP_LASER_CAL_CAPTURE_ENABLED
#define APP_OLED_ENABLED 1
#elif APP_Q2_UI_ENABLED
#define APP_OLED_ENABLED 1
#else
#define APP_OLED_ENABLED 0
#endif
#endif

/* Read-only bench telemetry, independent from the full button/menu UI. */
#ifndef APP_MOTOR1_STATUS_OLED_ENABLED
#define APP_MOTOR1_STATUS_OLED_ENABLED 1
#endif

#define APP_MOTOR1_OLED_PERIOD_MS 100U

/* Fixed gyro offsets in sensor axes, deg/s. Measured while stationary on
 * 2026-11-23; startup no longer collects a new calibration batch. */
#define APP_IMU_GYRO_BIAS_X_DPS  0.31284F
#define APP_IMU_GYRO_BIAS_Y_DPS -0.08947F
#define APP_IMU_GYRO_BIAS_Z_DPS  0.12095F

/* Sensor-to-pendulum rotation fitted from center/up/right/down/left poses.
 * Rows are right, up and the center gravity direction; right x up = normal. */
#define APP_IMU_FRAME_RIGHT  {0.64941527F, -0.75831454F, 0.05673503F}
#define APP_IMU_FRAME_UP     {-0.76034698F, -0.64865960F, 0.03336445F}
#define APP_IMU_FRAME_NORMAL {0.01150097F, -0.06480569F, -0.99783162F}

/* Centered laser zero offsets from the 2026-11-23 ground-mark check. */
#define APP_IMU_ROLL_ZERO_DEG -0.97740F
#define APP_IMU_PITCH_ZERO_DEG 0.19396F

/* Measured geometry, meters. The laser is coaxial with the pendulum rod.
 * Ground projection uses pivot height, not pivot-to-pendulum-center length. */
#define APP_PIVOT_GROUND_HEIGHT_M     0.99F
#define APP_Q1_LINE_LENGTH_M          0.60F
/* Q1 baseline period from the last full-duration trial. */
#define APP_Q1_PERIOD_MS              1642U
#define APP_Q1_DURATION_MS           25000U
#define APP_Q1_TURN_RATE_DPS             2.0F
#define APP_Q1_STOP_PITCH_DEG           25.0F
#define APP_Q1_STOP_ROLL_DEG             8.0F
#define APP_Q1_TELEMETRY_PERIOD_MS      20U
#define APP_Q1_START_ANGLE_DEG           7.0F
#define APP_Q1_PWM_MAX_PERMILLE        850U
#define APP_Q1_CONTROL_MODE_PID           0U
#define APP_Q1_CONTROL_MODE_TWO_PHASE_AMPLITUDE 1U
#define APP_Q1_CONTROL_MODE APP_Q1_CONTROL_MODE_TWO_PHASE_AMPLITUDE
#define APP_Q1_PITCH_KP               28.0F
#define APP_Q1_PITCH_KI                 0.0F
/* Last tested fixed-reference baseline used Kd=7. */
#define APP_Q1_PITCH_KD                 7.0F
#define APP_Q1_RATE_FILTER_TAU_S        0.05F
#define APP_Q1_INTEGRAL_LIMIT          150.0F
#define APP_Q1_AMPLITUDE_SWITCH_RATIO    0.90F
#define APP_Q1_AMPLITUDE_RELEASE_RATIO   0.85F
#define APP_Q1_AMPLITUDE_HOLD_MS          250U
#define APP_Q1_AMPLITUDE_BLEND_MS APP_Q1_PERIOD_MS
/* Conservative first-trial gains, in PWM permille and degree-based units. */
#define APP_Q1_AMPLITUDE_KA               2.0F
#define APP_Q1_AMPLITUDE_KCOMP            0.5F
#define APP_Q1_AMPLITUDE_D_FILTER_S       0.10F
#define APP_Q1_AMPLITUDE_I_LIMIT         10.0F
#define APP_Q1_AMPLITUDE_GAIN_LIMIT     100.0F
#define APP_Q1_CENTER_KP                 10.0F
#define APP_Q1_CENTER_LIMIT_PERMILLE     80U
#define APP_Q1_CENTER_FILTER_S            0.50F
#define APP_Q1_CENTER_TURN_RATE_DPS        5.0F
#define APP_Q1_STOP_RATE_DPS           150.0F

/* Q2 trajectory PID, current bench candidate in PWM permille / degrees.
 * Qualification is pending; Roll remains disabled. */
#define APP_Q2_DEFAULT_LENGTH_MM       500U
#define APP_Q2_PERIOD_MS              1610U
#define APP_Q2_AMPLITUDE_RAMP_MS      (2U * APP_Q2_PERIOD_MS)
#define APP_Q2_DURATION_MS           15000U
#define APP_Q2_PWM_MAX_PERMILLE        850U
#define APP_Q2_PITCH_KP                 32.75F
#define APP_Q2_PITCH_KI                  0.0F
#define APP_Q2_PITCH_KD                  8.0F
#define APP_Q2_RATE_FILTER_TAU_S         0.05F
#define APP_Q2_INTEGRAL_LIMIT          150.0F

#if (APP_Q1_PWM_MAX_PERMILLE == 0U) || (APP_Q1_PWM_MAX_PERMILLE > 1000U)
#error "Q1 PWM maximum must be within 1..1000 permille."
#endif

/* User calibration: VREFINT raw code normalized to VDDA=3300 mV.
 * Zero selects a nominal 1.210 V estimate, not a factory calibration claim. */
#define APP_ADC_VREFINT_CAL_RAW 0U
#define APP_ADC_VREFINT_NOMINAL_MV 1210U

/* INA240A2 / 5 mOhm / grounded REF. Zero is an output offset in uV,
 * not the Arduino library's ADC-reference/2 default. */
#define APP_CURRENT_SAMPLE_COUNT 8U
#define APP_CURRENT_ZERO_UV 0U
#define APP_CURRENT_GAIN_PERMILLE 100U

#if (APP_CURRENT_SAMPLE_COUNT < 1U) || (APP_CURRENT_SAMPLE_COUNT > 250U)
#error "Current ADC averaging requires 1..250 samples."
#endif

#if APP_MOTOR1_TEST_ENABLED && APP_OLED_ENABLED
#error "Motor 1 bench testing owns the buttons; disable the OLED UI."
#endif

#endif
