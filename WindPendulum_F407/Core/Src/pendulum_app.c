#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "pendulum_app.h"
#include "icm45686_stm32.h"
#include "pendulum_ui.h"
#include "app_config.h"
#include "services/imu_frame.h"
#include "problems/q1_swing.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define IMU_SAMPLE_PERIOD_MS 2U
#define LOG_PERIOD_MS 100U
#define Q1_MAX_SAMPLE_GAP_MS 10U

typedef struct {
  ImuSample sample;
  float roll_deg;
  float pitch_deg;
  float accel_weight;
  float gyro_bias_dps[3];
  float direction_deg;
  float tilt_deg;
  uint8_t direction_valid;
  float pitch_rate_dps;
} FusionFrame;

static QueueHandle_t g_imu_queue;
static QueueHandle_t g_fusion_queue;
static QueueHandle_t g_control_queue;
static QueueHandle_t g_telemetry_queue;
static QueueHandle_t g_display_queue;
/* Non-halting SWD mailbox. Zero on reset. Consumed once; no automatic start. */
volatile uint32_t g_q1_remote_command;
static volatile uint32_t g_motor_fault_reason;
static volatile bool g_q1_run_enabled;
static volatile BaseType_t g_imu_ready;
static const float g_gyro_bias_dps[3] = {
    APP_IMU_GYRO_BIAS_X_DPS, APP_IMU_GYRO_BIAS_Y_DPS,
    APP_IMU_GYRO_BIAS_Z_DPS};
static const ImuFrame_Config g_imu_frame = {
    {APP_IMU_FRAME_RIGHT, APP_IMU_FRAME_UP, APP_IMU_FRAME_NORMAL}};

static uint16_t Board_ReadBusCurrentAdc(void)
{
  ADC_ChannelConfTypeDef channel = {0};
  channel.Channel = ADC_CHANNEL_8;
  channel.Rank = 1;
  channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK ||
      HAL_ADC_Start(&hadc1) != HAL_OK) {
    return 0U;
  }

  uint16_t value = 0U;
  if (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK) {
    value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  }
  (void)HAL_ADC_Stop(&hadc1);
  return value;
}

static void ImuTask(void *argument)
{
  (void)argument;
  TickType_t wake_time = xTaskGetTickCount();
  TickType_t log_time = wake_time;
  uint32_t retry_time = 0U;
  BaseType_t sample_ok;
  BaseType_t imu_ready = pdFALSE;
  ImuSample sample;

  for (;;) {
    if ((imu_ready == pdFALSE) &&
        ((int32_t)(HAL_GetTick() - retry_time) >= 0)) {
      if (ICM45686_STM32_Init() == 0) {
        imu_ready = pdTRUE;
        vTaskDelay(pdMS_TO_TICKS(100U));
      } else {
        retry_time = HAL_GetTick() + 1000U;
      }
    }

    sample_ok = ((imu_ready != pdFALSE) &&
                 (ICM45686_STM32_Read(&sample) == 0)) ? pdTRUE : pdFALSE;
    if (sample_ok != pdFALSE) {
      g_imu_ready = pdTRUE;
      sample.timestamp_ms = HAL_GetTick();
      (void)xQueueOverwrite(g_imu_queue, &sample);
    } else {
      g_imu_ready = pdFALSE;
      if (imu_ready != pdFALSE) {
        imu_ready = pdFALSE;
        retry_time = HAL_GetTick() + 1000U;
      }
    }

    if ((sample_ok == pdFALSE) &&
        ((xTaskGetTickCount() - log_time) >= pdMS_TO_TICKS(LOG_PERIOD_MS))) {
      PendulumTelemetry telemetry = {0};
      PendulumDisplayState display = {0};
      telemetry.timestamp_ms = HAL_GetTick();
      telemetry.mode = PENDULUM_MODE_IMU_FAULT;
      telemetry.bus_current_adc = Board_ReadBusCurrentAdc();
      display.telemetry = telemetry;
      memcpy(display.gyro_bias_dps, g_gyro_bias_dps, sizeof(g_gyro_bias_dps));
      (void)xQueueOverwrite(g_display_queue, &display);
      (void)xQueueOverwrite(g_telemetry_queue, &telemetry);
      log_time = xTaskGetTickCount();
    }
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
  }
}

static void FusionTask(void *argument)
{
  (void)argument;
  ImuSample sample;
  FusionFrame frame = {0};
  float previous_gyro[3] = {0.0F, 0.0F, 0.0F};
  uint32_t previous_ms = 0U;

  configASSERT(ImuFrame_ConfigValid(&g_imu_frame));

  for (;;) {
    if (xQueueReceive(g_imu_queue, &sample, portMAX_DELAY) != pdPASS) {
      continue;
    }

    float accel[3];
    float gyro[3];
    float corrected_gyro[3];
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
      corrected_gyro[axis] = sample.gyro_dps[axis] - g_gyro_bias_dps[axis];
    }
    if (!ImuFrame_Rotate(&g_imu_frame, sample.accel_g, accel) ||
        !ImuFrame_Rotate(&g_imu_frame, corrected_gyro, gyro)) {
      continue;
    }

    const float dt = (previous_ms == 0U) ? 0.002F :
                     (float)(sample.timestamp_ms - previous_ms) * 0.001F;
    previous_ms = sample.timestamp_ms;
    const float safe_dt = (dt > 0.0001F) ? dt : 0.002F;

    const float accel_roll = atan2f(accel[1], accel[2]) * 57.2957795F;
    const float accel_pitch = atan2f(accel[0],
                                     sqrtf(accel[1] * accel[1] +
                                           accel[2] * accel[2])) * 57.2957795F;
    const float accel_norm = sqrtf(sample.accel_g[0] * sample.accel_g[0] +
                                   sample.accel_g[1] * sample.accel_g[1] +
                                   sample.accel_g[2] * sample.accel_g[2]);
    const float norm_error = fabsf(accel_norm - 1.0F);
    const float angular_acceleration =
        sqrtf((gyro[0] - previous_gyro[0]) * (gyro[0] - previous_gyro[0]) +
              (gyro[1] - previous_gyro[1]) * (gyro[1] - previous_gyro[1]) +
              (gyro[2] - previous_gyro[2]) * (gyro[2] - previous_gyro[2])) / safe_dt;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
      previous_gyro[axis] = gyro[axis];
    }
    frame.accel_weight = (norm_error < 0.15F) ? 0.02F :
                         (norm_error < 0.50F) ? 0.005F : 0.0F;
    if (angular_acceleration > 250.0F) {
      frame.accel_weight = 0.0F;
    }
    frame.roll_deg = (1.0F - frame.accel_weight) *
                     (frame.roll_deg + gyro[0] * safe_dt) +
                     frame.accel_weight * accel_roll;
    frame.pitch_deg = (1.0F - frame.accel_weight) *
                      (frame.pitch_deg - gyro[1] * safe_dt) +
                      frame.accel_weight * accel_pitch;
    frame.direction_valid = ImuFrame_Polar(frame.roll_deg, frame.pitch_deg,
        &frame.direction_deg, &frame.tilt_deg) ? 1U : 0U;
    frame.pitch_rate_dps = -gyro[1];
    frame.sample = sample;
    memcpy(frame.gyro_bias_dps, g_gyro_bias_dps, sizeof(g_gyro_bias_dps));
    (void)xQueueOverwrite(g_fusion_queue, &frame);
  }
}

static void TrajectoryTask(void *argument)
{
  (void)argument;
  FusionFrame frame = {0};
  Q1Swing swing = {0};
  uint32_t last_sample_ms = 0U;
  uint32_t last_remote = 0U;

  for (;;) {
    const uint32_t remote = g_q1_remote_command;
    if (remote != 0U) {
      g_q1_remote_command = 0U;
      last_remote = remote;
    }
    const bool received = xQueueReceive(g_fusion_queue, &frame,
        pdMS_TO_TICKS(5U)) == pdPASS;
    const uint32_t now = HAL_GetTick();
    const bool stop_key = HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET;
    const bool sample_fresh = received && g_imu_ready != pdFALSE &&
        now - frame.sample.timestamp_ms <= 20U;
    if (last_remote == 0x51315354U) {
      Q1Swing_Stop(&swing, Q1_SWING_USER_STOP);
      g_q1_run_enabled = false;
      last_remote = 0U;
    }
    if (last_remote == 0x5131474FU) {
      if (swing.state != Q1_SWING_RUNNING) {
        if (sample_fresh && !stop_key && now >= 2000U &&
            Q1Swing_Start(&swing, now, frame.roll_deg, frame.pitch_deg,
                          frame.pitch_rate_dps)) {
          g_motor_fault_reason = 0U;
          g_q1_run_enabled = true;
        } else {
          Q1Swing_Stop(&swing, Q1_SWING_NOT_CENTERED);
          g_q1_run_enabled = false;
        }
      }
      last_remote = 0U;
    }
    if (swing.state == Q1_SWING_RUNNING) {
      if (g_motor_fault_reason != 0U) {
        Q1Swing_Stop(&swing, (Q1Swing_Reason)g_motor_fault_reason);
      } else if (!sample_fresh && now - last_sample_ms > 20U) {
        Q1Swing_Stop(&swing, Q1_SWING_SAMPLE_STALE);
      } else if (sample_fresh) {
        Q1Swing_Step(&swing, now, frame.roll_deg, frame.pitch_deg,
                      frame.pitch_rate_dps, true, stop_key);
      } else if (stop_key) {
        Q1Swing_Stop(&swing, Q1_SWING_USER_STOP);
      }
    }
    if (received) last_sample_ms = frame.sample.timestamp_ms;
    if (swing.state != Q1_SWING_RUNNING) g_q1_run_enabled = false;

    PendulumTelemetry command = {0};
    command.timestamp_ms = received ? frame.sample.timestamp_ms : now;
    command.imu = frame.sample;
    command.roll_deg = frame.roll_deg;
    command.pitch_deg = frame.pitch_deg;
    command.direction_deg = frame.direction_deg;
    command.tilt_deg = frame.tilt_deg;
    command.direction_valid = frame.direction_valid;
    command.accel_weight = frame.accel_weight;
    command.pitch_rate_dps = frame.pitch_rate_dps;
    command.q1_state = (uint8_t)swing.state;
    command.q1_reason = (uint8_t)swing.reason;
    command.q1_switches = swing.switches;
    command.q1_predicted_peak_deg = swing.predicted_peak_deg;
    command.q1_left_peak_deg = swing.left_peak_deg;
    command.q1_right_peak_deg = swing.right_peak_deg;
    command.q1_run_ms = (swing.state == Q1_SWING_IDLE) ? 0U : now - swing.started_ms;
    command.target_pitch_deg = (float)swing.direction *
        atan2f(0.5F * APP_Q1_LINE_LENGTH_M, APP_PIVOT_GROUND_HEIGHT_M) * 57.2957795F;
    command.mode = (swing.state == Q1_SWING_RUNNING) ? PENDULUM_MODE_LINE :
        (swing.state == Q1_SWING_FAULT) ? PENDULUM_MODE_Q1_BLOCKED : PENDULUM_MODE_READY;
    if (g_imu_ready == pdFALSE) command.mode = PENDULUM_MODE_IMU_FAULT;
    if (sample_fresh && g_q1_run_enabled) {
      memcpy(command.pwm_permille, swing.pwm, sizeof(swing.pwm));
    }
    if (received) command.bus_current_adc = Board_ReadBusCurrentAdc();
    (void)xQueueOverwrite(g_control_queue, &command);
    if (received) {
      PendulumDisplayState display = {0};
      display.telemetry = command;
      display.imu_online = 1U;
      display.calibrated = 1U;
      memcpy(display.gyro_bias_dps, frame.gyro_bias_dps, sizeof(frame.gyro_bias_dps));
      (void)xQueueOverwrite(g_display_queue, &display);
    }
  }
}

static void MotorTask(void *argument)
{
  (void)argument;
  static const uint32_t channels[4] = {
      TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
  PendulumTelemetry command = {0};
  IWDG_HandleTypeDef watchdog = {0};
  uint32_t last_log = 0U;
  uint32_t beep_until = 0U;
  uint8_t previous_state = Q1_SWING_IDLE;
  bool previous_output = false;
  uint16_t previous_left = 0U;
  uint16_t previous_right = 0U;
  uint32_t en_released_ms = 0U;
  watchdog.Instance = IWDG;
  watchdog.Init.Prescaler = IWDG_PRESCALER_32;
  watchdog.Init.Reload = 499U;
  __HAL_DBGMCU_UNFREEZE_IWDG();
  if (HAL_IWDG_Init(&watchdog) != HAL_OK) Error_Handler();

  for (;;) {
    const bool received = xQueueReceive(g_control_queue, &command,
        pdMS_TO_TICKS(5U)) == pdPASS;
    const uint32_t now = HAL_GetTick();
    const bool stop = HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET;
    bool active = g_q1_run_enabled && g_motor_fault_reason == 0U &&
        command.q1_state == Q1_SWING_RUNNING && command.mode == PENDULUM_MODE_LINE;
    if (active && (stop || g_imu_ready == pdFALSE ||
        now - command.timestamp_ms > 20U || !isfinite(command.pitch_deg) ||
        !isfinite(command.roll_deg))) {
      g_motor_fault_reason = stop ? Q1_SWING_USER_STOP : Q1_SWING_SAMPLE_STALE;
      active = false;
    }
    if (!received && g_q1_run_enabled && now - command.timestamp_ms > 20U) {
      g_motor_fault_reason = Q1_SWING_SAMPLE_STALE;
      active = false;
    }
    const bool valid_output = command.pwm_permille[2] == 0U &&
        command.pwm_permille[3] == 0U &&
        command.pwm_permille[0] <= APP_Q1_PWM_MAX_PERMILLE &&
        command.pwm_permille[1] <= APP_Q1_PWM_MAX_PERMILLE &&
        (command.pwm_permille[0] == 0U || command.pwm_permille[1] == 0U);
    if (active && (!valid_output || command.q1_run_ms >= APP_Q1_DURATION_MS ||
        fabsf(command.pitch_deg) > APP_Q1_STOP_PITCH_DEG ||
        fabsf(command.roll_deg) > APP_Q1_STOP_ROLL_DEG)) {
      g_motor_fault_reason = Q1_SWING_ANGLE_LIMIT;
      active = false;
    }
    /* Keep EN stable during a ramp on the same side. Gate channel changes
     * and latch all preload registers before re-enabling the bridge. */
    const bool channel_changed =
        ((command.pwm_permille[0] > 0U) != (previous_left > 0U)) ||
        ((command.pwm_permille[1] > 0U) != (previous_right > 0U));
    if (!active || channel_changed) {
      HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
    }
    for (uint32_t ch = 0U; ch < 4U; ++ch) {
      const uint16_t duty = active ? command.pwm_permille[ch] : 0U;
      __HAL_TIM_SET_COMPARE(&htim1, channels[ch],
          ((htim1.Init.Period + 1U) * duty) / 1000U);
      command.pwm_applied_permille[ch] = duty;
    }
    if (active) {
      if (!previous_output || channel_changed) {
        (void)HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
        en_released_ms = now;
      }
      HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_SET);
      if (now - en_released_ms >= 50U &&
          HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin) == GPIO_PIN_RESET) {
        g_motor_fault_reason = Q1_SWING_EN_LOW;
        HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
        for (uint32_t ch = 0U; ch < 4U; ++ch) {
          __HAL_TIM_SET_COMPARE(&htim1, channels[ch], 0U);
          command.pwm_applied_permille[ch] = 0U;
        }
        active = false;
      }
    }
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, active ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (command.q1_state == Q1_SWING_FINISHED && previous_state != Q1_SWING_FINISHED)
      beep_until = now + 200U;
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
        (int32_t)(beep_until - now) > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    command.en_command = active ? 1U : 0U;
    command.en_pin = HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin) == GPIO_PIN_SET;
    command.key_bits = (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET ? 1U : 0U) |
        (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET ? 2U : 0U) |
        (stop ? 4U : 0U);
    if (received && (now - last_log >= APP_Q1_TELEMETRY_PERIOD_MS ||
        command.q1_state != previous_state || active != previous_output)) {
      (void)xQueueOverwrite(g_telemetry_queue, &command);
      last_log = now;
    }
    previous_state = command.q1_state;
    previous_output = active;
    previous_left = active ? command.pwm_applied_permille[0] : 0U;
    previous_right = active ? command.pwm_applied_permille[1] : 0U;
    (void)HAL_IWDG_Refresh(&watchdog);
  }
}

static void TelemetryTask(void *argument)
{
  (void)argument;
  static const char header[] =
      "time_ms,gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw,roll_cdeg,pitch_cdeg,accel_weight_permille,"
      "target_roll_cdeg,target_pitch_cdeg,temp_cdeg,"
      "p_roll_milli,i_roll_milli,d_roll_milli,p_pitch_milli,i_pitch_milli,d_pitch_milli,"
      "pid_roll_milli,pid_pitch_milli,target_position_mm,target_velocity_mm_s,"
      "pwm1_requested_permille,pwm2_requested_permille,pwm3_requested_permille,pwm4_requested_permille,"
      "pwm1_applied_permille,pwm2_applied_permille,pwm3_applied_permille,pwm4_applied_permille,"
      "bus_current_adc,mode,direction_cdeg,tilt_cdeg,direction_valid,"
      "pitch_rate_cdeg_s,q1_state,q1_reason,q1_run_ms,q1_switches,"
      "predicted_peak_cdeg,left_peak_cdeg,right_peak_cdeg,en_command,en_pin,keys\r\n";
  PendulumTelemetry data;
  char line[512];

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)header, sizeof(header) - 1U, 100U);
  for (;;) {
    if (xQueueReceive(g_telemetry_queue, &data, portMAX_DELAY) != pdPASS) {
      continue;
    }
    const int length = snprintf(line, sizeof(line),
        "%lu,%d,%d,%d,%d,%d,%d,%ld,%ld,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%ld,%ld,%u,"
        "%ld,%u,%u,%lu,%lu,%ld,%ld,%ld,%u,%u,%u\r\n",
        (unsigned long)data.timestamp_ms,
        data.imu.gyro_raw[0], data.imu.gyro_raw[1], data.imu.gyro_raw[2],
        data.imu.accel_raw[0], data.imu.accel_raw[1], data.imu.accel_raw[2],
        (long)(data.roll_deg * 100.0F), (long)(data.pitch_deg * 100.0F),
        (unsigned)(data.accel_weight * 1000.0F),
        (long)(data.target_roll_deg * 100.0F),
        (long)(data.target_pitch_deg * 100.0F),
        (long)(data.imu.temperature_c * 100.0F),
        (long)(data.pid_p[0] * 1000.0F), (long)(data.pid_i[0] * 1000.0F),
        (long)(data.pid_d[0] * 1000.0F), (long)(data.pid_p[1] * 1000.0F),
        (long)(data.pid_i[1] * 1000.0F), (long)(data.pid_d[1] * 1000.0F),
        (long)(data.pid_output[0] * 1000.0F),
        (long)(data.pid_output[1] * 1000.0F),
        (long)(data.q1_target_position_m * 1000.0F),
        (long)(data.q1_target_velocity_mps * 1000.0F),
        (unsigned)data.pwm_permille[0], (unsigned)data.pwm_permille[1],
        (unsigned)data.pwm_permille[2], (unsigned)data.pwm_permille[3],
        (unsigned)data.pwm_applied_permille[0],
        (unsigned)data.pwm_applied_permille[1],
        (unsigned)data.pwm_applied_permille[2],
        (unsigned)data.pwm_applied_permille[3],
        (unsigned)data.bus_current_adc,
        (unsigned)data.mode,
        (long)(data.direction_deg * 100.0F),
        (long)(data.tilt_deg * 100.0F),
        (unsigned)data.direction_valid,
        (long)(data.pitch_rate_dps * 100.0F),
        (unsigned)data.q1_state, (unsigned)data.q1_reason,
        (unsigned long)data.q1_run_ms, (unsigned long)data.q1_switches,
        (long)(data.q1_predicted_peak_deg * 100.0F),
        (long)(data.q1_left_peak_deg * 100.0F),
        (long)(data.q1_right_peak_deg * 100.0F),
        (unsigned)data.en_command, (unsigned)data.en_pin, (unsigned)data.key_bits);
    if (length > 0 && (size_t)length < sizeof(line)) {
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)length, 50U);
    }
  }
}

void PendulumApp_CreateTasks(void)
{
  g_imu_queue = xQueueCreate(1U, sizeof(ImuSample));
  g_fusion_queue = xQueueCreate(1U, sizeof(FusionFrame));
  g_control_queue = xQueueCreate(1U, sizeof(PendulumTelemetry));
  g_telemetry_queue = xQueueCreate(1U, sizeof(PendulumTelemetry));
  g_display_queue = xQueueCreate(1U, sizeof(PendulumDisplayState));
  configASSERT(g_imu_queue != NULL);
  configASSERT(g_fusion_queue != NULL);
  configASSERT(g_control_queue != NULL);
  configASSERT(g_telemetry_queue != NULL);
  configASSERT(g_display_queue != NULL);

  configASSERT(xTaskCreate(ImuTask, "imu", 512U, NULL, 5U, NULL) == pdPASS);
  configASSERT(xTaskCreate(FusionTask, "fusion", 768U, NULL, 4U, NULL) == pdPASS);
  configASSERT(xTaskCreate(TrajectoryTask, "trajectory", 512U, NULL, 3U, NULL) == pdPASS);
  configASSERT(xTaskCreate(MotorTask, "motor", 512U, NULL, 5U, NULL) == pdPASS);
  configASSERT(xTaskCreate(TelemetryTask, "telemetry", 768U, NULL, 1U, NULL) == pdPASS);
#if APP_OLED_ENABLED
  configASSERT(xTaskCreate(PendulumUI_Task, "ui", 768U,
                           (void *)g_display_queue, 2U, NULL) == pdPASS);
#endif
}

void PendulumApp_SetMotorsArmed(uint8_t armed)
{
  /* All actuator writes belong to MotorTask. This legacy API only cancels. */
  if (armed == 0U) (void)PendulumApp_RequestQ1Cancel();
}

bool PendulumApp_RequestQ1Start(void)
{
  if (g_q1_remote_command != 0U || g_q1_run_enabled) return false;
  g_q1_remote_command = 0x5131474FU;
  return true;
}

bool PendulumApp_RequestQ1Cancel(void)
{
  g_q1_remote_command = 0x51315354U;
  return true;
}
