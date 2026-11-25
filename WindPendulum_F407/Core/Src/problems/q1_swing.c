#include "problems/q1_swing.h"
#include "app_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define Q1_MAX_SAMPLE_GAP_MS 20U
#define Q1_DEG_PER_RAD 57.2957795131F
#define Q1_RAD_PER_DEG 0.0174532925199F
#define Q1_TWO_PI 6.2831853072F

static float Q1Swing_Clamp(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

Q1Swing_Tuning Q1Swing_DefaultTuning(void)
{
  return (Q1Swing_Tuning){
      .startup_kp = APP_Q1_PITCH_KP,
      .startup_kd = APP_Q1_PITCH_KD,
      .amplitude_kp = APP_Q1_AMPLITUDE_KA,
      .damping_compensation = APP_Q1_AMPLITUDE_KCOMP};
}

static bool Q1Swing_TuningIsValid(const Q1Swing_Tuning *tuning)
{
  return tuning->startup_kp >= 0.0F && tuning->startup_kp <= 100.0F &&
      tuning->startup_kd >= 0.0F && tuning->startup_kd <= 50.0F &&
      tuning->amplitude_kp >= 0.0F && tuning->amplitude_kp <= 10.0F &&
      tuning->amplitude_ki >= 0.0F && tuning->amplitude_ki <= 5.0F &&
      tuning->amplitude_kd >= 0.0F && tuning->amplitude_kd <= 2.0F &&
      tuning->damping_compensation >= 0.0F &&
      tuning->damping_compensation <= 5.0F &&
      (tuning->target_amplitude_deg == 0.0F ||
       (tuning->target_amplitude_deg >= 5.0F &&
        tuning->target_amplitude_deg <= 20.0F));
}

static PID_Config Q1Swing_PidConfig(const Q1Swing_Tuning *tuning)
{
  const PID_Config config = {
      .kp = tuning->startup_kp,
      .ki = APP_Q1_PITCH_KI,
      .kd = tuning->startup_kd,
      .proportional_setpoint_weight = 1.0F,
      .derivative_filter_tau_s = APP_Q1_RATE_FILTER_TAU_S,
      .integral_min = -APP_Q1_INTEGRAL_LIMIT,
      .integral_max = APP_Q1_INTEGRAL_LIMIT,
      .output_min = -(float)APP_Q1_PWM_MAX_PERMILLE,
      .output_max = (float)APP_Q1_PWM_MAX_PERMILLE};
  return config;
}

void Q1Swing_Stop(Q1Swing *swing, Q1Swing_Reason reason)
{
  if (swing == NULL) return;
  if (reason != Q1_SWING_TIME_UP) ProblemQ1_Cancel(&swing->reference);
  swing->reason = reason;
  swing->state = (reason == Q1_SWING_TIME_UP) ? Q1_SWING_FINISHED :
      (reason == Q1_SWING_USER_STOP) ? Q1_SWING_STOPPED : Q1_SWING_FAULT;
  swing->pitch_integrating = false;
  memset(swing->pwm, 0, sizeof(swing->pwm));
  memset(&swing->control_output, 0, sizeof(swing->control_output));
  PendulumControl_Reset(&swing->controller, 0.0F, 0.0F);
}

bool Q1Swing_Start(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                    float pitch_deg, float pitch_rate_dps)
{
  const Q1Swing_Tuning tuning = Q1Swing_DefaultTuning();
  return Q1Swing_StartTuned(swing, &tuning, now_ms, roll_deg, pitch_deg,
                            pitch_rate_dps);
}

bool Q1Swing_StartTuned(Q1Swing *swing, const Q1Swing_Tuning *tuning,
                        uint32_t now_ms, float roll_deg, float pitch_deg,
                        float pitch_rate_dps)
{
  if (swing == NULL || swing->state == Q1_SWING_RUNNING) return false;
  *swing = (Q1Swing){0};
  if (tuning == NULL || !Q1Swing_TuningIsValid(tuning)) {
    Q1Swing_Stop(swing, Q1_SWING_INVALID_CONFIG);
    return false;
  }
  swing->tuning = *tuning;
  if (!isfinite(roll_deg) || !isfinite(pitch_deg) ||
      !isfinite(pitch_rate_dps) ||
      fabsf(roll_deg) > APP_Q1_START_ANGLE_DEG ||
      fabsf(pitch_deg) > APP_Q1_START_ANGLE_DEG) {
    Q1Swing_Stop(swing, Q1_SWING_NOT_CENTERED);
    return false;
  }

  ProblemQ1_Config reference_config = ProblemQ1_DefaultConfig();
  reference_config.target_amplitude_deg = tuning->target_amplitude_deg;
  if (tuning->target_amplitude_deg > 0.0F) {
    reference_config.line_length_m = 2.0F * reference_config.ground_height_m *
        tanf(tuning->target_amplitude_deg * Q1_RAD_PER_DEG);
  }
  swing->target_amplitude_deg = ProblemQ1_TargetAmplitudeDeg(&reference_config);
  if (ProblemQ1_Start(&swing->reference, &reference_config, now_ms,
                      true, true) != PROBLEM_Q1_OK) {
    Q1Swing_Stop(swing, Q1_SWING_INVALID_CONFIG);
    return false;
  }

  const PID_Config pid = Q1Swing_PidConfig(tuning);
  const PID_Config amplitude_pid = {
      .kp = tuning->amplitude_kp,
      .ki = tuning->amplitude_ki,
      .kd = tuning->amplitude_kd,
      .proportional_setpoint_weight = 1.0F,
      .derivative_filter_tau_s = APP_Q1_AMPLITUDE_D_FILTER_S,
      .integral_min = -APP_Q1_AMPLITUDE_I_LIMIT,
      .integral_max = APP_Q1_AMPLITUDE_I_LIMIT,
      .output_min = -APP_Q1_AMPLITUDE_GAIN_LIMIT,
      .output_max = APP_Q1_AMPLITUDE_GAIN_LIMIT};
  const PendulumControl_Config control_config = {
      .roll_pid = pid,
      .pitch_pid = pid,
      /* Positive Pitch is right; PWM1 moves right and PWM2 moves left. */
      .fan_mix = {{0.0F, 1.0F}, {0.0F, -1.0F},
                  {0.0F, 0.0F}, {0.0F, 0.0F}},
      .max_duty_permille = APP_Q1_PWM_MAX_PERMILLE,
      .mixer_calibrated = true,
      .disabled_axis_mask = PENDULUM_DISABLE_ROLL};
  if (!PendulumControl_Init(&swing->controller, &control_config) ||
      !PID_Init(&swing->amplitude_pid, &amplitude_pid)) {
    Q1Swing_Stop(swing, Q1_SWING_INVALID_CONFIG);
    return false;
  }

  PendulumControl_Reset(&swing->controller, roll_deg, pitch_deg);
  swing->amplitude_mode_enabled =
      APP_Q1_CONTROL_MODE == APP_Q1_CONTROL_MODE_TWO_PHASE_AMPLITUDE;
  swing->state = Q1_SWING_RUNNING;
  swing->started_ms = now_ms;
  swing->last_step_ms = now_ms;
  return true;
}

static void Q1Swing_UpdatePwm(Q1Swing *swing)
{
  memcpy(swing->pwm, swing->control_output.pwm_permille, sizeof(swing->pwm));
  const int sign = (swing->pwm[0] > 0U) ? 1 :
      (swing->pwm[1] > 0U) ? -1 : 0;
  if (sign != 0) {
    if (swing->last_pwm_sign != 0 && sign != swing->last_pwm_sign) {
      swing->switches++;
    }
    swing->last_pwm_sign = sign;
  }
}

static void Q1Swing_UpdateTurnMonitor(Q1Swing *swing, float pitch_deg)
{
  if (pitch_deg > swing->right_peak_deg) swing->right_peak_deg = pitch_deg;
  if (-pitch_deg > swing->left_peak_deg) swing->left_peak_deg = -pitch_deg;
  swing->peak_deg = fmaxf(swing->right_peak_deg, swing->left_peak_deg);
}

static void Q1Swing_UpdateAmplitudeEstimate(Q1Swing *swing, float pitch_deg)
{
  const float filtered_rate_dps =
      -swing->controller.axis_pid[PENDULUM_AXIS_PITCH].derivative;
  const float omega0_rad_s = Q1_TWO_PI * 1000.0F /
      (float)swing->reference.config.swing_period_ms;
  const float theta_rad = pitch_deg * Q1_RAD_PER_DEG;
  const float rate_rad_s = filtered_rate_dps * Q1_RAD_PER_DEG;
  const float amplitude_rad = sqrtf(theta_rad * theta_rad +
      (rate_rad_s / omega0_rad_s) * (rate_rad_s / omega0_rad_s));

  swing->amplitude_estimate_deg = amplitude_rad * Q1_DEG_PER_RAD;
  swing->amplitude_error_deg = swing->target_amplitude_deg -
      swing->amplitude_estimate_deg;
}

static void Q1Swing_UpdateAmplitudePhase(Q1Swing *swing, uint32_t now_ms)
{
  if (!swing->amplitude_mode_enabled) return;

  if (swing->control_phase == Q1_SWING_CONTROL_PID) {
    const float switch_threshold = swing->target_amplitude_deg *
        APP_Q1_AMPLITUDE_SWITCH_RATIO;
    const float release_threshold = swing->target_amplitude_deg *
        APP_Q1_AMPLITUDE_RELEASE_RATIO;
    if (swing->run_ms < swing->reference.config.startup_ramp_ms ||
        swing->amplitude_estimate_deg < release_threshold) {
      swing->amplitude_threshold_tracking = false;
      return;
    }
    if (swing->amplitude_estimate_deg < switch_threshold) {
      swing->amplitude_threshold_tracking = false;
      return;
    }
    if (!swing->amplitude_threshold_tracking) {
      swing->amplitude_threshold_tracking = true;
      swing->amplitude_threshold_since_ms = now_ms;
      return;
    }
    if (now_ms - swing->amplitude_threshold_since_ms >=
        APP_Q1_AMPLITUDE_HOLD_MS) {
      swing->control_phase = Q1_SWING_CONTROL_BLEND;
      swing->amplitude_blend_started_ms = now_ms;
    }
  }

  if (swing->control_phase == Q1_SWING_CONTROL_BLEND) {
    const uint32_t blend_elapsed_ms = now_ms - swing->amplitude_blend_started_ms;
    if (blend_elapsed_ms >= APP_Q1_AMPLITUDE_BLEND_MS) {
      swing->control_phase = Q1_SWING_CONTROL_AMPLITUDE;
      swing->amplitude_blend_permille = 1000U;
    } else {
      swing->amplitude_blend_permille = (uint16_t)(
          (blend_elapsed_ms * 1000U) / APP_Q1_AMPLITUDE_BLEND_MS);
    }
  } else if (swing->control_phase == Q1_SWING_CONTROL_AMPLITUDE) {
    swing->amplitude_blend_permille = 1000U;
  }
}

static void Q1Swing_SetSignedPitchOutput(Q1Swing *swing, float output)
{
  const float limit = (float)APP_Q1_PWM_MAX_PERMILLE;
  output = Q1Swing_Clamp(output, -limit, limit);
  swing->control_output.axis_output[PENDULUM_AXIS_PITCH] = output;
  swing->control_output.pwm_permille[0] = output > 0.0F ?
      (uint16_t)(output + 0.5F) : 0U;
  swing->control_output.pwm_permille[1] = output < 0.0F ?
      (uint16_t)(-output + 0.5F) : 0U;
  swing->control_output.pwm_permille[2] = 0U;
  swing->control_output.pwm_permille[3] = 0U;
  swing->control_output.pwm_ready = true;
}

void Q1Swing_Step(Q1Swing *swing, uint32_t now_ms, float roll_deg,
                   float pitch_deg, float pitch_rate_dps,
                   bool sample_valid, bool stop_requested)
{
  if (swing == NULL || swing->state != Q1_SWING_RUNNING) return;
  if (stop_requested) {
    Q1Swing_Stop(swing, Q1_SWING_USER_STOP);
    return;
  }

  const uint32_t elapsed_ms = now_ms - swing->last_step_ms;
  if (!sample_valid || elapsed_ms > Q1_MAX_SAMPLE_GAP_MS ||
      !isfinite(roll_deg) || !isfinite(pitch_deg) ||
      !isfinite(pitch_rate_dps)) {
    Q1Swing_Stop(swing, Q1_SWING_SAMPLE_STALE);
    return;
  }
  if (fabsf(pitch_deg) > APP_Q1_STOP_PITCH_DEG ||
      fabsf(roll_deg) > APP_Q1_STOP_ROLL_DEG ||
      fabsf(pitch_rate_dps) > APP_Q1_STOP_RATE_DPS) {
    Q1Swing_Stop(swing, Q1_SWING_ANGLE_LIMIT);
    return;
  }

  swing->last_step_ms = now_ms;
  swing->run_ms = now_ms - swing->started_ms;
  Q1Swing_UpdateTurnMonitor(swing, pitch_deg);

  ProblemQ1_Output reference;
  if (!ProblemQ1_Step(&swing->reference, now_ms, &reference)) {
    Q1Swing_Stop(swing, Q1_SWING_INVALID_CONFIG);
    return;
  }
  if (reference.complete) {
    Q1Swing_Stop(swing, Q1_SWING_TIME_UP);
    return;
  }

  swing->target_roll_deg = reference.target_roll_deg;
  swing->target_pitch_deg = reference.target_pitch_deg;
  swing->target_pitch_rate_dps = reference.target_pitch_rate_dps;
  swing->target_position_m = reference.target_position_m;
  swing->target_velocity_mps = reference.target_velocity_mps;
  if (elapsed_ms == 0U) return;

  const PendulumControl_Input input = {
      .target_deg = {reference.target_roll_deg, reference.target_pitch_deg},
      /* Use measured-rate damping for the Q1 comparison. */
      .target_rate_dps = {0.0F, 0.0F},
      .angle_deg = {roll_deg, pitch_deg},
      .rate_dps = {0.0F, pitch_rate_dps},
      .dt_s = (float)elapsed_ms * 0.001F,
      .integrate = {false, swing->controller.config.pitch_pid.ki != 0.0F}};
  swing->pitch_integrating = input.integrate[PENDULUM_AXIS_PITCH];
  if (!PendulumControl_StepWithRates(&swing->controller, &input,
                                      &swing->control_output) ||
      !swing->control_output.pwm_ready) {
    Q1Swing_Stop(swing, Q1_SWING_INVALID_CONFIG);
    return;
  }

  Q1Swing_UpdateAmplitudeEstimate(swing, pitch_deg);
  Q1Swing_UpdateAmplitudePhase(swing, now_ms);
  if (swing->amplitude_mode_enabled) {
    PID_Controller *amplitude = &swing->amplitude_pid;
    const float filtered_rate = -swing->controller.axis_pid[PENDULUM_AXIS_PITCH].derivative;
    const float amp_rate = (swing->amplitude_estimate_deg -
        amplitude->previous_measurement) / input.dt_s;
    const float previous_gain = amplitude->output + swing->tuning.damping_compensation;
    const bool integrate = swing->amplitude_blend_permille > 0U &&
        swing->tuning.amplitude_ki > 0.0F &&
        (fabsf(previous_gain * filtered_rate) < (float)APP_Q1_PWM_MAX_PERMILLE ||
         previous_gain * swing->amplitude_error_deg <= 0.0F);
    const float gain = PID_UpdateWithRate(amplitude, swing->target_amplitude_deg,
        swing->amplitude_estimate_deg, 0.0F, amp_rate, input.dt_s, integrate);
    swing->amplitude_feedback_output = gain * filtered_rate;
    swing->amplitude_compensation_output = swing->tuning.damping_compensation * filtered_rate;
    const float blend = (float)swing->amplitude_blend_permille / 1000.0F;
    const float pid_output = swing->control_output.axis_output[PENDULUM_AXIS_PITCH];
    const float pid_p = swing->control_output.p[PENDULUM_AXIS_PITCH];
    const float pid_d = swing->control_output.d[PENDULUM_AXIS_PITCH];
    const float energy_output = Q1Swing_Clamp(
        swing->amplitude_feedback_output + swing->amplitude_compensation_output,
        -(float)APP_Q1_PWM_MAX_PERMILLE, (float)APP_Q1_PWM_MAX_PERMILLE);
    const float blended_output = pid_output + blend * (energy_output - pid_output);
    swing->control_output.p[PENDULUM_AXIS_PITCH] = pid_p * (1.0F - blend) +
        amplitude->p_term * filtered_rate * blend;
    swing->control_output.i[PENDULUM_AXIS_PITCH] =
        amplitude->i_term * filtered_rate * blend;
    swing->control_output.d[PENDULUM_AXIS_PITCH] = pid_d * (1.0F - blend) +
        (amplitude->d_term * filtered_rate + swing->amplitude_compensation_output) * blend;
    Q1Swing_SetSignedPitchOutput(swing, blended_output);
  }
  Q1Swing_UpdatePwm(swing);
}
