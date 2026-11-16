#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "pendulum_app.h"
#include "icm45686_stm32.h"
#include "pendulum_ui.h"
#include "app_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define IMU_SAMPLE_PERIOD_MS 2U
#define CALIBRATION_SAMPLE_COUNT 5000U
#define LOG_PERIOD_MS 100U
#define CALIBRATION_GYRO_LIMIT_DPS 8.0F
#define CALIBRATION_ACCEL_NORM_ERROR 0.15F
#define Q1_MAX_SAMPLE_GAP_MS 10U

typedef struct {
  ImuSample sample;
  float roll_deg;
  float pitch_deg;
  float accel_weight;
  float gyro_bias_dps[3];
} FusionFrame;

typedef enum {
  Q1_REQUEST_START = 0,
  Q1_REQUEST_CANCEL
} Q1_RequestType;

typedef struct {
  Q1_RequestType type;
  ProblemQ1_Config problem_config;
  PendulumControl_Config control_config;
} Q1_Request;

static QueueHandle_t g_imu_queue;
static QueueHandle_t g_fusion_queue;
static QueueHandle_t g_control_queue;
static QueueHandle_t g_telemetry_queue;
static QueueHandle_t g_display_queue;
static QueueHandle_t g_q1_request_queue;
static volatile BaseType_t g_calibrated;
static volatile uint32_t g_calibration_samples;

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
      sample.timestamp_ms = HAL_GetTick();
      (void)xQueueOverwrite(g_imu_queue, &sample);
    } else if (imu_ready != pdFALSE) {
      imu_ready = pdFALSE;
      retry_time = HAL_GetTick() + 1000U;
    }

    if ((sample_ok == pdFALSE) &&
        ((xTaskGetTickCount() - log_time) >= pdMS_TO_TICKS(LOG_PERIOD_MS))) {
      PendulumTelemetry telemetry = {0};
      PendulumDisplayState display = {0};
      telemetry.timestamp_ms = HAL_GetTick();
      telemetry.mode = PENDULUM_MODE_IMU_FAULT;
      telemetry.bus_current_adc = Board_ReadBusCurrentAdc();
      display.telemetry = telemetry;
      display.calibration_samples = g_calibration_samples;
      display.calibrated = (g_calibrated != pdFALSE) ? 1U : 0U;
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
  float gyro_bias[3] = {0.0F, 0.0F, 0.0F};
  float gyro_sum[3] = {0.0F, 0.0F, 0.0F};
  float previous_gyro[3] = {0.0F, 0.0F, 0.0F};
  uint32_t calibration_samples = 0U;
  uint32_t calibration_report_ticks = 0U;
  uint32_t previous_ms = 0U;

  for (;;) {
    if (xQueueReceive(g_imu_queue, &sample, portMAX_DELAY) != pdPASS) {
      continue;
    }

    if (g_calibrated == pdFALSE) {
      const float accel_norm = sqrtf(sample.accel_g[0] * sample.accel_g[0] +
                                     sample.accel_g[1] * sample.accel_g[1] +
                                     sample.accel_g[2] * sample.accel_g[2]);
      const float gyro_norm = sqrtf(sample.gyro_dps[0] * sample.gyro_dps[0] +
                                    sample.gyro_dps[1] * sample.gyro_dps[1] +
                                    sample.gyro_dps[2] * sample.gyro_dps[2]);
      PendulumDisplayState display = {0};

      if ((fabsf(accel_norm - 1.0F) <= CALIBRATION_ACCEL_NORM_ERROR) &&
          (gyro_norm <= CALIBRATION_GYRO_LIMIT_DPS)) {
        for (uint32_t axis = 0U; axis < 3U; ++axis) {
          gyro_sum[axis] += sample.gyro_dps[axis];
        }
        calibration_samples++;
      } else {
        memset(gyro_sum, 0, sizeof(gyro_sum));
        calibration_samples = 0U;
      }
      g_calibration_samples = calibration_samples;
      calibration_report_ticks++;

      if (calibration_report_ticks >= 50U ||
          calibration_samples >= CALIBRATION_SAMPLE_COUNT) {
        calibration_report_ticks = 0U;
        display.telemetry.imu = sample;
        display.telemetry.timestamp_ms = sample.timestamp_ms;
        display.telemetry.mode = PENDULUM_MODE_CALIBRATION;
        display.calibration_samples = calibration_samples;
        display.imu_online = 1U;
        (void)xQueueOverwrite(g_display_queue, &display);
        display.telemetry.bus_current_adc = Board_ReadBusCurrentAdc();
        (void)xQueueOverwrite(g_telemetry_queue, &display.telemetry);
      }

      if (calibration_samples >= CALIBRATION_SAMPLE_COUNT) {
        for (uint32_t axis = 0U; axis < 3U; ++axis) {
          gyro_bias[axis] = gyro_sum[axis] / (float)calibration_samples;
        }
        g_calibrated = pdTRUE;
      }
      continue;
    }

    const float dt = (previous_ms == 0U) ? 0.002F :
                     (float)(sample.timestamp_ms - previous_ms) * 0.001F;
    previous_ms = sample.timestamp_ms;
    const float safe_dt = (dt > 0.0001F) ? dt : 0.002F;

    const float accel_roll = atan2f(sample.accel_g[1], sample.accel_g[2]) * 57.2957795F;
    const float accel_pitch = atan2f(-sample.accel_g[0],
                                     sqrtf(sample.accel_g[1] * sample.accel_g[1] +
                                           sample.accel_g[2] * sample.accel_g[2])) * 57.2957795F;
    const float accel_norm = sqrtf(sample.accel_g[0] * sample.accel_g[0] +
                                   sample.accel_g[1] * sample.accel_g[1] +
                                   sample.accel_g[2] * sample.accel_g[2]);
    const float norm_error = fabsf(accel_norm - 1.0F);
    const float angular_acceleration =
        sqrtf((sample.gyro_dps[0] - previous_gyro[0]) *
                  (sample.gyro_dps[0] - previous_gyro[0]) +
              (sample.gyro_dps[1] - previous_gyro[1]) *
                  (sample.gyro_dps[1] - previous_gyro[1]) +
              (sample.gyro_dps[2] - previous_gyro[2]) *
                  (sample.gyro_dps[2] - previous_gyro[2])) / safe_dt;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
      previous_gyro[axis] = sample.gyro_dps[axis];
    }
    frame.accel_weight = (norm_error < 0.15F) ? 0.02F :
                         (norm_error < 0.50F) ? 0.005F : 0.0F;
    if (angular_acceleration > 250.0F) {
      frame.accel_weight = 0.0F;
    }
    frame.roll_deg = (1.0F - frame.accel_weight) *
                     (frame.roll_deg + (sample.gyro_dps[0] - gyro_bias[0]) * safe_dt) +
                     frame.accel_weight * accel_roll;
    frame.pitch_deg = (1.0F - frame.accel_weight) *
                      (frame.pitch_deg + (sample.gyro_dps[1] - gyro_bias[1]) * safe_dt) +
                      frame.accel_weight * accel_pitch;
    frame.sample = sample;
    memcpy(frame.gyro_bias_dps, gyro_bias, sizeof(gyro_bias));
    (void)xQueueOverwrite(g_fusion_queue, &frame);
  }
}

static void TrajectoryTask(void *argument)
{
  (void)argument;
  FusionFrame frame;
  PendulumTelemetry command;
  ProblemQ1 problem = {0};
  PendulumControl control = {0};
  ProblemQ1_Config pending_problem_config = {0};
  bool q1_active = false;
  bool q1_start_pending = false;
  bool q1_blocked = false;
  uint32_t q1_request_time_ms = 0U;
  uint32_t previous_sample_ms = 0U;
  Q1_Request request;

  for (;;) {
    while (xQueueReceive(g_q1_request_queue, &request, 0U) == pdPASS) {
      if (request.type == Q1_REQUEST_CANCEL) {
        ProblemQ1_Cancel(&problem);
        q1_active = false;
        q1_start_pending = false;
        q1_blocked = false;
        previous_sample_ms = 0U;
      } else if ((request.type == Q1_REQUEST_START) &&
                 !q1_active && !q1_start_pending) {
        if (!ProblemQ1_Validate(&request.problem_config) ||
            (g_calibrated == pdFALSE) ||
            !PendulumControl_Init(&control, &request.control_config)) {
          q1_blocked = true;
        } else {
          pending_problem_config = request.problem_config;
          q1_start_pending = true;
          q1_request_time_ms = HAL_GetTick();
          q1_blocked = false;
        }
      }
    }

    if (xQueueReceive(g_fusion_queue, &frame,
                      pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS)) != pdPASS) {
      if (q1_active &&
          ((uint32_t)(HAL_GetTick() - previous_sample_ms) >
           Q1_MAX_SAMPLE_GAP_MS)) {
        ProblemQ1_Cancel(&problem);
        q1_active = false;
        q1_blocked = true;
      }
      if (q1_start_pending &&
          ((uint32_t)(HAL_GetTick() - q1_request_time_ms) > 100U)) {
        q1_start_pending = false;
        q1_blocked = true;
      }
      continue;
    }

    command = (PendulumTelemetry){0};
    command.imu = frame.sample;
    command.timestamp_ms = frame.sample.timestamp_ms;
    command.roll_deg = frame.roll_deg;
    command.pitch_deg = frame.pitch_deg;
    command.accel_weight = frame.accel_weight;
    command.mode = q1_blocked ? PENDULUM_MODE_Q1_BLOCKED : PENDULUM_MODE_READY;
    command.bus_current_adc = Board_ReadBusCurrentAdc();

    if (q1_start_pending) {
      const ProblemQ1_Result start_result = ProblemQ1_Start(
          &problem, &pending_problem_config, frame.sample.timestamp_ms,
          true, (g_calibrated != pdFALSE));
      q1_start_pending = false;
      q1_active = (start_result == PROBLEM_Q1_OK);
      q1_blocked = !q1_active;
      if (q1_active) {
        PendulumControl_Reset(&control, frame.roll_deg, frame.pitch_deg);
        previous_sample_ms = frame.sample.timestamp_ms;
      }
      command.mode = q1_blocked ? PENDULUM_MODE_Q1_BLOCKED :
                                  PENDULUM_MODE_READY;
    }

    if (q1_active) {
      const uint32_t sample_gap_ms = frame.sample.timestamp_ms -
                                     previous_sample_ms;
      const float dt_s = (sample_gap_ms == 0U) ?
                         ((float)IMU_SAMPLE_PERIOD_MS * 0.001F) :
                         ((float)sample_gap_ms * 0.001F);
      ProblemQ1_Output target;
      PendulumControl_Output control_output;

      if ((sample_gap_ms > Q1_MAX_SAMPLE_GAP_MS) ||
          !ProblemQ1_Step(&problem, frame.sample.timestamp_ms, &target)) {
        ProblemQ1_Cancel(&problem);
        q1_active = false;
        q1_blocked = true;
        command.mode = PENDULUM_MODE_Q1_BLOCKED;
      } else if (target.complete) {
        q1_active = false;
        q1_blocked = false;
        PendulumControl_Reset(&control, frame.roll_deg, frame.pitch_deg);
        command.mode = PENDULUM_MODE_READY;
      } else if (!PendulumControl_Step(
                     &control, target.target_roll_deg,
                     target.target_pitch_deg, frame.roll_deg,
                     frame.pitch_deg, dt_s, &control_output)) {
        ProblemQ1_Cancel(&problem);
        q1_active = false;
        q1_blocked = true;
        command.mode = PENDULUM_MODE_Q1_BLOCKED;
      } else {
        command.target_roll_deg = target.target_roll_deg;
        command.target_pitch_deg = target.target_pitch_deg;
        command.q1_target_position_m = target.target_position_m;
        command.q1_target_velocity_mps = target.target_velocity_mps;
        memcpy(command.pid_p, control_output.p, sizeof(command.pid_p));
        memcpy(command.pid_i, control_output.i, sizeof(command.pid_i));
        memcpy(command.pid_d, control_output.d, sizeof(command.pid_d));
        memcpy(command.pid_output, control_output.axis_output,
               sizeof(command.pid_output));
        memcpy(command.pwm_permille, control_output.pwm_permille,
               sizeof(command.pwm_permille));
        command.mode = PENDULUM_MODE_LINE;
      }
      previous_sample_ms = frame.sample.timestamp_ms;
    } else {
      previous_sample_ms = frame.sample.timestamp_ms;
    }

    (void)xQueueOverwrite(g_control_queue, &command);

    PendulumDisplayState display = {0};
    display.telemetry = command;
    display.calibration_samples = g_calibration_samples;
    display.imu_online = 1U;
    display.calibrated = (g_calibrated != pdFALSE) ? 1U : 0U;
    memcpy(display.gyro_bias_dps, frame.gyro_bias_dps,
           sizeof(display.gyro_bias_dps));
    (void)xQueueOverwrite(g_display_queue, &display);
  }
}

static void MotorTask(void *argument)
{
  (void)argument;
  PendulumTelemetry command;
  static const uint32_t pwm_channels[4] = {
      TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

  for (;;) {
    if (xQueueReceive(g_control_queue, &command, portMAX_DELAY) != pdPASS) {
      continue;
    }

    HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
    for (uint32_t output = 0U; output < 4U; ++output) {
      __HAL_TIM_SET_COMPARE(&htim1, pwm_channels[output], 0U);
    }
    memset(command.pwm_applied_permille, 0,
           sizeof(command.pwm_applied_permille));
    (void)xQueueOverwrite(g_telemetry_queue, &command);
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
      "bus_current_adc,mode\r\n";
  PendulumTelemetry data;
  char line[384];

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)header, sizeof(header) - 1U, 100U);
  for (;;) {
    if (xQueueReceive(g_telemetry_queue, &data, portMAX_DELAY) != pdPASS) {
      continue;
    }
    const int length = snprintf(line, sizeof(line),
        "%lu,%d,%d,%d,%d,%d,%d,%ld,%ld,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
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
        (unsigned)data.mode);
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
  g_q1_request_queue = xQueueCreate(4U, sizeof(Q1_Request));
  configASSERT(g_imu_queue != NULL);
  configASSERT(g_fusion_queue != NULL);
  configASSERT(g_control_queue != NULL);
  configASSERT(g_telemetry_queue != NULL);
  configASSERT(g_display_queue != NULL);
  configASSERT(g_q1_request_queue != NULL);

  configASSERT(xTaskCreate(ImuTask, "imu", 512U, NULL, 5U, NULL) == pdPASS);
  configASSERT(xTaskCreate(FusionTask, "fusion", 768U, NULL, 4U, NULL) == pdPASS);
  configASSERT(xTaskCreate(TrajectoryTask, "trajectory", 512U, NULL, 3U, NULL) == pdPASS);
  configASSERT(xTaskCreate(MotorTask, "motor", 384U, NULL, 5U, NULL) == pdPASS);
  configASSERT(xTaskCreate(TelemetryTask, "telemetry", 768U, NULL, 1U, NULL) == pdPASS);
#if APP_OLED_ENABLED
  configASSERT(xTaskCreate(PendulumUI_Task, "ui", 768U,
                           (void *)g_display_queue, 2U, NULL) == pdPASS);
#endif
}

void PendulumApp_SetMotorsArmed(uint8_t armed)
{
  (void)armed;
  HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
}

bool PendulumApp_RequestQ1Start(const ProblemQ1_Config *problem_config,
                                const PendulumControl_Config *control_config)
{
  Q1_Request request = {0};

  if ((problem_config == NULL) || (control_config == NULL) ||
      (g_q1_request_queue == NULL)) {
    return false;
  }

  request.type = Q1_REQUEST_START;
  request.problem_config = *problem_config;
  request.control_config = *control_config;
  return xQueueSend(g_q1_request_queue, &request, 0U) == pdPASS;
}

bool PendulumApp_RequestQ1Cancel(void)
{
  Q1_Request request = {0};

  if (g_q1_request_queue == NULL) {
    return false;
  }

  request.type = Q1_REQUEST_CANCEL;
  return xQueueSendToFront(g_q1_request_queue, &request, 0U) == pdPASS;
}
