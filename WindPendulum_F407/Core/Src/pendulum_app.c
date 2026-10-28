#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "pendulum_app.h"
#include "icm45686_stm32.h"
#include "pendulum_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define IMU_SAMPLE_PERIOD_MS 2U
#define CALIBRATION_SAMPLE_COUNT 5000U
#define LOG_PERIOD_MS 100U
#define CALIBRATION_GYRO_LIMIT_DPS 8.0F
#define CALIBRATION_ACCEL_NORM_ERROR 0.15F

typedef struct {
  ImuSample sample;
  float roll_deg;
  float pitch_deg;
  float accel_weight;
  float gyro_bias_dps[3];
} FusionFrame;

static QueueHandle_t g_imu_queue;
static QueueHandle_t g_fusion_queue;
static QueueHandle_t g_control_queue;
static QueueHandle_t g_telemetry_queue;
static QueueHandle_t g_display_queue;
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
  PendulumTelemetry command = {0};

  for (;;) {
    if (xQueueReceive(g_fusion_queue, &frame, portMAX_DELAY) != pdPASS) {
      continue;
    }

    command.imu = frame.sample;
    command.timestamp_ms = frame.sample.timestamp_ms;
    command.roll_deg = frame.roll_deg;
    command.pitch_deg = frame.pitch_deg;
    command.accel_weight = frame.accel_weight;
    command.mode = PENDULUM_MODE_READY;
    command.bus_current_adc = Board_ReadBusCurrentAdc();

    /* Setpoints, PID gains, and the four-fan mixer require bench calibration. */
    memset(command.pid_p, 0, sizeof(command.pid_p));
    memset(command.pid_i, 0, sizeof(command.pid_i));
    memset(command.pid_d, 0, sizeof(command.pid_d));
    memset(command.pid_output, 0, sizeof(command.pid_output));
    memset(command.pwm_permille, 0, sizeof(command.pwm_permille));
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
  (void)htim1.Init.Period;

  for (;;) {
    if (xQueueReceive(g_control_queue, &command, portMAX_DELAY) != pdPASS) {
      continue;
    }

    HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
    memset(command.pwm_permille, 0, sizeof(command.pwm_permille));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
    (void)xQueueOverwrite(g_telemetry_queue, &command);
  }
}

static void TelemetryTask(void *argument)
{
  (void)argument;
  static const char header[] =
      "time_ms,gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw,roll_cdeg,pitch_cdeg,accel_weight_permille,"
      "temp_cdeg,"
      "p_roll_milli,i_roll_milli,d_roll_milli,p_pitch_milli,i_pitch_milli,d_pitch_milli,"
      "pid_roll_milli,pid_pitch_milli,pwm1_permille,pwm2_permille,pwm3_permille,pwm4_permille,"
      "bus_current_adc,mode\r\n";
  PendulumTelemetry data;
  char line[256];

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)header, sizeof(header) - 1U, 100U);
  for (;;) {
    if (xQueueReceive(g_telemetry_queue, &data, portMAX_DELAY) != pdPASS) {
      continue;
    }
    const int length = snprintf(line, sizeof(line),
        "%lu,%d,%d,%d,%d,%d,%d,%ld,%ld,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
        "%ld,%u,%u,%u,%u,%u,%u\r\n",
        (unsigned long)data.timestamp_ms,
        data.imu.gyro_raw[0], data.imu.gyro_raw[1], data.imu.gyro_raw[2],
        data.imu.accel_raw[0], data.imu.accel_raw[1], data.imu.accel_raw[2],
        (long)(data.roll_deg * 100.0F), (long)(data.pitch_deg * 100.0F),
        (unsigned)(data.accel_weight * 1000.0F),
        (long)(data.imu.temperature_c * 100.0F),
        (long)(data.pid_p[0] * 1000.0F), (long)(data.pid_i[0] * 1000.0F),
        (long)(data.pid_d[0] * 1000.0F), (long)(data.pid_p[1] * 1000.0F),
        (long)(data.pid_i[1] * 1000.0F), (long)(data.pid_d[1] * 1000.0F),
        (long)(data.pid_output[0] * 1000.0F),
        (long)(data.pid_output[1] * 1000.0F),
        data.pwm_permille[0], data.pwm_permille[1], data.pwm_permille[2],
        data.pwm_permille[3], data.bus_current_adc, (unsigned)data.mode);
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
  configASSERT(xTaskCreate(MotorTask, "motor", 384U, NULL, 5U, NULL) == pdPASS);
  configASSERT(xTaskCreate(TelemetryTask, "telemetry", 768U, NULL, 1U, NULL) == pdPASS);
  configASSERT(xTaskCreate(PendulumUI_Task, "ui", 768U,
                           (void *)g_display_queue, 2U, NULL) == pdPASS);
}

void PendulumApp_SetMotorsArmed(uint8_t armed)
{
  (void)armed;
  HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_RESET);
}
