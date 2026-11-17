#include "diagnostics/motor1_test.h"

#include "FreeRTOS.h"
#include "app_config.h"
#include "main.h"
#include "queue.h"
#include "services/current_sense.h"
#include "services/voltage_sense.h"
#include "task.h"
#if APP_MOTOR1_STATUS_OLED_ENABLED
#include "kk_oled.h"
#endif

#include <stdio.h>

#define MOTOR1_TEST_PERIOD_MS 10U
#define MOTOR1_TEST_LOG_MS   100U

/* U1 is currently exposed as the ADC pin voltage; update the divider after
 * the external sensing network is confirmed and calibrated. */
static const VoltageSense_Config g_voltage_config = {
    1U, 1U, 1000U, 0, 3U};
static VoltageSense_State g_voltage_state;
/* INA240A2: 50 V/V, two 10 mOhm shunts in parallel, grounded REF. */
static const CurrentSense_Config g_current_config = {
    .amplifier_gain = 50U,
    .shunt_uohm = 5000U,
    .zero_uv = APP_CURRENT_ZERO_UV,
    .gain_permille = APP_CURRENT_GAIN_PERMILLE,
    .filter_shift = 3U};
static CurrentSense_State g_current_state;

typedef struct {
  uint32_t timestamp_ms;
  Motor1Test test;
  uint16_t current_adc;
  CurrentSense_Result current;
  VoltageSense_Result voltage;
  uint16_t ccr[4];
  uint8_t key_bits;
  uint8_t en_command;
  uint8_t en_pin;
  uint8_t adc_valid;
  uint8_t voltage_valid;
  uint32_t step_gap_ms;
  uint32_t max_step_gap_ms;
} Motor1Test_Snapshot;

static QueueHandle_t g_motor1_log_queue;
#if APP_MOTOR1_STATUS_OLED_ENABLED
static QueueHandle_t g_motor1_oled_queue;
#endif
static IWDG_HandleTypeDef g_motor1_watchdog;
static const char *const g_motor1_state_names[] = {"RELEASE", "READY", "FIXED", "RAMP"};
static const char *const g_motor1_reason_names[] = {
    "BOOT", "STOP_KEY", "LOOP_DELAY", "EN_LOW"};

/* Visible in the debugger. Values report MCU commands, not measured speed. */
volatile Motor1Test_Snapshot g_motor1_test_status;

static void Motor1Test_Apply(uint16_t duty_permille)
{
  uint32_t compare;

  if (duty_permille > MOTOR1_TEST_MAX_DUTY)
  {
    duty_permille = 0U;
  }
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);
  compare = ((htim1.Init.Period + 1U) * duty_permille) / 1000U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
  /* Normal stops use zero PWM; board protection can still pull EN low. */
  HAL_GPIO_WritePin(MOTOR_ENABLE_GPIO_Port, MOTOR_ENABLE_Pin, GPIO_PIN_SET);
}

static bool Motor1Test_ReadAdcSamples(uint32_t adc_channel, uint16_t *values,
                                     uint16_t sample_count)
{
  ADC_ChannelConfTypeDef channel = {0};
  bool valid = true;

  if ((values == NULL) || (sample_count == 0U) ||
      (sample_count > CURRENT_SENSE_MAX_SAMPLES))
  {
    return false;
  }
  channel.Channel = adc_channel;
  channel.Rank = 1U;
  channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK)
  {
    valid = false;
  }
  else
  {
    /* Discard the first conversion after switching ADC channels. */
    for (uint32_t conversion = 0U; conversion <= sample_count; ++conversion)
    {
      if (HAL_ADC_Start(&hadc1) != HAL_OK)
      {
        valid = false;
        break;
      }
      if (HAL_ADC_PollForConversion(&hadc1, 2U) != HAL_OK)
      {
        valid = false;
        (void)HAL_ADC_Stop(&hadc1);
        break;
      }
      const uint16_t sample = (uint16_t)HAL_ADC_GetValue(&hadc1);
      if (conversion != 0U)
      {
        values[conversion - 1U] = sample;
      }
      if (HAL_ADC_Stop(&hadc1) != HAL_OK)
      {
        valid = false;
        break;
      }
    }
  }
  return valid;
}

static bool Motor1Test_ReadAdcChannel(uint32_t adc_channel, uint16_t *value)
{
  if (value == NULL)
  {
    return false;
  }
  *value = 0U;
  return Motor1Test_ReadAdcSamples(adc_channel, value, 1U);
}

static void Motor1Test_ReadAnalog(Motor1Test_Snapshot *snapshot)
{
  VoltageSense_Result result = {0};
  uint16_t vrefint_adc = 0U;
  uint16_t voltage_adc = 0U;
  uint16_t current_samples[APP_CURRENT_SAMPLE_COUNT] = {0};
  /* Use an explicit board calibration or a clearly nominal reference. */
  const uint16_t vrefint_cal_adc = (APP_ADC_VREFINT_CAL_RAW != 0U) ?
      APP_ADC_VREFINT_CAL_RAW :
      (uint16_t)((APP_ADC_VREFINT_NOMINAL_MV * 4095U + 1650U) / 3300U);
  const bool vrefint_ok = Motor1Test_ReadAdcChannel(ADC_CHANNEL_VREFINT,
                                                     &vrefint_adc);
  const bool voltage_ok = Motor1Test_ReadAdcChannel(ADC_CHANNEL_9,
                                                     &voltage_adc);
  const bool current_ok = Motor1Test_ReadAdcSamples(ADC_CHANNEL_8,
      current_samples, APP_CURRENT_SAMPLE_COUNT);

  /* Keep the 28-column protocol's current_adc as the latest raw code. */
  snapshot->current_adc = current_ok ?
      current_samples[APP_CURRENT_SAMPLE_COUNT - 1U] : 0U;

  snapshot->voltage_valid = 0U;
  if (vrefint_ok)
  {
    (void)VoltageSense_Update(&g_voltage_state, &g_voltage_config,
                              voltage_ok ? voltage_adc : 4095U,
                              vrefint_adc, vrefint_cal_adc,
                              &result);
  }
  else
  {
    result.adc_raw = voltage_adc;
    result.vrefint_raw = vrefint_adc;
  }
  result.adc_raw = voltage_adc;
  snapshot->voltage = result;
  snapshot->voltage_valid = voltage_ok ? result.valid : 0U;
  snapshot->adc_valid = current_ok ? 1U : 0U;
  (void)CurrentSense_UpdateSamples(&g_current_state, &g_current_config,
      current_samples, APP_CURRENT_SAMPLE_COUNT, result.vdda_mv,
      current_ok && (result.reference_valid != 0U), &snapshot->current);
}

static void Motor1Test_Task(void *argument)
{
  (void)argument;
  Motor1Test_Snapshot snapshot = {0};
  Motor1Test_State previous_state = MOTOR1_TEST_WAIT_RELEASE;
  TickType_t wake_time = xTaskGetTickCount();
  uint32_t log_time_ms = HAL_GetTick();

  VoltageSense_Reset(&g_voltage_state);
  CurrentSense_Reset(&g_current_state);
  Motor1Test_Apply(0U);
  Motor1Test_Init(&snapshot.test, log_time_ms);
  /* LSI tolerance applies: approximately 0.5 s at 32 kHz. */
  g_motor1_watchdog.Instance = IWDG;
  g_motor1_watchdog.Init.Prescaler = IWDG_PRESCALER_32;
  g_motor1_watchdog.Init.Reload = 499U;
  __HAL_DBGMCU_UNFREEZE_IWDG();
  if (HAL_IWDG_Init(&g_motor1_watchdog) != HAL_OK)
  {
    Error_Handler();
  }

  for (;;)
  {
    uint32_t now_ms = HAL_GetTick();
    uint32_t keys = KEY1_GPIO_Port->IDR;
    bool fixed_pressed = (keys & KEY1_Pin) == 0U;
    bool ramp_pressed = (keys & KEY2_Pin) == 0U;
    bool stop_pressed = (keys & KEY3_Pin) == 0U;
    snapshot.key_bits = (fixed_pressed ? 1U : 0U) |
                        (ramp_pressed ? 2U : 0U) |
                        (stop_pressed ? 4U : 0U);
    snapshot.step_gap_ms = now_ms - snapshot.test.last_step_ms;
    if (snapshot.step_gap_ms > snapshot.max_step_gap_ms)
    {
      snapshot.max_step_gap_ms = snapshot.step_gap_ms;
    }
    Motor1Test_Step(&snapshot.test, now_ms, fixed_pressed, ramp_pressed,
                    stop_pressed);
    Motor1Test_Apply(snapshot.test.duty_permille);
    snapshot.en_command = 1U;
    snapshot.en_pin = (HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,
                       MOTOR_ENABLE_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    if (((snapshot.test.state == MOTOR1_TEST_FIXED) ||
         (snapshot.test.state == MOTOR1_TEST_RAMP)) &&
        (snapshot.en_pin == 0U) &&
        (snapshot.test.run_elapsed_ms >= 50U))
    {
      Motor1Test_Stop(&snapshot.test, now_ms, MOTOR1_TEST_EN_LOW);
      Motor1Test_Apply(0U);
    }

    snapshot.timestamp_ms = now_ms;
    Motor1Test_ReadAnalog(&snapshot);
    snapshot.ccr[0] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    snapshot.ccr[1] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    snapshot.ccr[2] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    snapshot.ccr[3] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    if ((snapshot.test.state != previous_state) ||
        ((uint32_t)(now_ms - log_time_ms) >= MOTOR1_TEST_LOG_MS))
    {
      (void)xQueueOverwrite(g_motor1_log_queue, &snapshot);
      log_time_ms = now_ms;
    }
    previous_state = snapshot.test.state;
    g_motor1_test_status = snapshot;
#if APP_MOTOR1_STATUS_OLED_ENABLED
    /* A private one-slot queue keeps OLED reads independent of UART logging. */
    (void)xQueueOverwrite(g_motor1_oled_queue, &snapshot);
#endif
    (void)HAL_IWDG_Refresh(&g_motor1_watchdog);
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(MOTOR1_TEST_PERIOD_MS));
  }
}

static void Motor1Test_LogTask(void *argument)
{
  (void)argument;
  static const char banner[] =
      "MOTOR1_BENCH,EN=HIGH,PE9=TIM1_CH1,20kHz,max=800permille\r\n"
      "KEY1=PC13:30%; KEY2=PC0:20-80%,1%/100ms; KEY3=PC1:STOP. Latched.\r\n"
      "time_ms,state,reason,duty_permille,en_command,en_pin,ccr1,ccr2,ccr3,ccr4,"
      "current_adc,current_valid,voltage_adc,vrefint_adc,vdda_mv,u1_pin_mv,"
      "u1_input_mv,u1_filtered_mv,voltage_valid,keys,run_ms,step_gap_ms,"
      "max_step_gap_ms,current_ma,current_filtered_ma,current_calc_valid,"
      "current_overrange,reference_calibrated\r\n";
  Motor1Test_Snapshot snapshot;
  char line[320];

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)banner, sizeof(banner) - 1U, 100U);
  for (;;)
  {
    if (xQueueReceive(g_motor1_log_queue, &snapshot, portMAX_DELAY) == pdPASS)
    {
      int length = snprintf(line, sizeof(line),
          "%lu,%s,%s,"
          "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
          "%lu,%lu,%lu,%lu,%u,%u,%lu,%lu,%lu,"
          "%lu,%lu,%u,%u,%u\r\n",
          (unsigned long)snapshot.timestamp_ms,
          g_motor1_state_names[snapshot.test.state],
          g_motor1_reason_names[snapshot.test.stop_reason],
          (unsigned)snapshot.test.duty_permille, (unsigned)snapshot.en_command,
          (unsigned)snapshot.en_pin, (unsigned)snapshot.ccr[0],
          (unsigned)snapshot.ccr[1], (unsigned)snapshot.ccr[2],
          (unsigned)snapshot.ccr[3], (unsigned)snapshot.current_adc,
          (unsigned)snapshot.adc_valid,
          (unsigned)snapshot.voltage.adc_raw,
          (unsigned)snapshot.voltage.vrefint_raw,
          (unsigned long)snapshot.voltage.vdda_mv,
          (unsigned long)snapshot.voltage.pin_mv,
          (unsigned long)snapshot.voltage.input_mv,
          (unsigned long)snapshot.voltage.filtered_mv,
          (unsigned)snapshot.voltage_valid, (unsigned)snapshot.key_bits,
          (unsigned long)snapshot.test.run_elapsed_ms,
          (unsigned long)snapshot.step_gap_ms,
          (unsigned long)snapshot.max_step_gap_ms,
          (unsigned long)snapshot.current.current_ma,
          (unsigned long)snapshot.current.filtered_ma,
          (unsigned)snapshot.current.valid,
          (unsigned)snapshot.current.overrange,
          (unsigned)(APP_ADC_VREFINT_CAL_RAW != 0U));
      if ((length > 0) && ((size_t)length < sizeof(line)))
      {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)length, 25U);
      }
    }
  }
}

#if APP_MOTOR1_STATUS_OLED_ENABLED
static void Motor1Test_OledTask(void *argument)
{
  (void)argument;
  extern const uint8_t u8g2_font_5x7_tf[];
  Motor1Test_Snapshot snapshot;
  bool online = false;
  uint32_t last_attempt_ms = HAL_GetTick() - 1000U;
  TickType_t wake_time = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(APP_MOTOR1_OLED_PERIOD_MS);
  char line[32];

  for (;;)
  {
    /* Never scan keys or write actuators here; only the test task owns them. */
    if (xQueuePeek(g_motor1_oled_queue, &snapshot, 0U) == pdPASS)
    {
      uint32_t now_ms = HAL_GetTick();
      if (!online && ((uint32_t)(now_ms - last_attempt_ms) >= 1000U))
      {
        last_attempt_ms = now_ms;
        online = OLED_Init() == OLED_OK;
        if (online)
        {
          OLED_SetFont(u8g2_font_5x7_tf);
          OLED_SetFontPosition(OLED_FONT_POS_TOP);
          /* Initialization can take a frame time; obtain fresh telemetry. */
          (void)xQueuePeek(g_motor1_oled_queue, &snapshot, 0U);
        }
      }
      if (online)
      {
        OLED_Clear();
        (void)snprintf(line, sizeof(line), "MOTOR1 %s",
                       g_motor1_state_names[snapshot.test.state]);
        OLED_DrawUTF8(0, 0, line);
        (void)snprintf(line, sizeof(line), "K1:%u K2:%u K3:%u",
                       (unsigned)(snapshot.key_bits & 1U),
                       (unsigned)((snapshot.key_bits >> 1U) & 1U),
                       (unsigned)((snapshot.key_bits >> 2U) & 1U));
        OLED_DrawUTF8(0, 9, line);
        (void)snprintf(line, sizeof(line), "PWM1:%u.%u%% 20kHz",
                       (unsigned)(snapshot.test.duty_permille / 10U),
                       (unsigned)(snapshot.test.duty_permille % 10U));
        OLED_DrawUTF8(0, 18, line);
        if (snapshot.voltage_valid != 0U)
        {
          (void)snprintf(line, sizeof(line), "C1:%u U1:%lu.%02luV",
              (unsigned)snapshot.ccr[0],
              (unsigned long)(snapshot.voltage.filtered_mv / 1000U),
              (unsigned long)((snapshot.voltage.filtered_mv % 1000U) / 10U));
        }
        else
        {
          (void)snprintf(line, sizeof(line), "C1:%u U1:INVALID",
                         (unsigned)snapshot.ccr[0]);
        }
        OLED_DrawUTF8(0, 27, line);
        if (snapshot.adc_valid == 0U)
        {
          (void)snprintf(line, sizeof(line), "ADC:ERR I:--");
        }
        else if (snapshot.current.valid != 0U)
        {
          (void)snprintf(line, sizeof(line), "ADC:%u I:%lu.%03luA",
              (unsigned)snapshot.current_adc,
              (unsigned long)(snapshot.current.filtered_ma / 1000U),
              (unsigned long)(snapshot.current.filtered_ma % 1000U));
        }
        else
        {
          (void)snprintf(line, sizeof(line), "ADC:%u I:%s",
              (unsigned)snapshot.current_adc,
              snapshot.current.overrange != 0U ? "CLIP" : "--");
        }
        OLED_DrawUTF8(0, 36, line);
        (void)snprintf(line, sizeof(line), "EN SET:%u READ:%u",
                       (unsigned)snapshot.en_command, (unsigned)snapshot.en_pin);
        OLED_DrawUTF8(0, 45, line);
        (void)snprintf(line, sizeof(line), "STOP:%s",
                       g_motor1_reason_names[snapshot.test.stop_reason]);
        OLED_DrawUTF8(0, 54, line);
        if (OLED_Update() != OLED_OK)
        {
          online = false;
          last_attempt_ms = HAL_GetTick();
        }
      }
    }
    /* Skip missed display frames instead of trying to catch up in a burst. */
    TickType_t now_ticks = xTaskGetTickCount();
    if ((TickType_t)(now_ticks - wake_time) >= period)
    {
      wake_time = now_ticks;
    }
    vTaskDelayUntil(&wake_time, period);
  }
}
#endif

void Motor1Test_CreateTasks(void)
{
  g_motor1_log_queue = xQueueCreate(1U, sizeof(Motor1Test_Snapshot));
  configASSERT(g_motor1_log_queue != NULL);
#if APP_MOTOR1_STATUS_OLED_ENABLED
  g_motor1_oled_queue = xQueueCreate(1U, sizeof(Motor1Test_Snapshot));
  configASSERT(g_motor1_oled_queue != NULL);
#endif
  configASSERT(xTaskCreate(Motor1Test_Task, "motor1_test", 512U, NULL,
                          5U, NULL) == pdPASS);
  configASSERT(xTaskCreate(Motor1Test_LogTask, "motor1_log", 512U, NULL,
                          1U, NULL) == pdPASS);
#if APP_MOTOR1_STATUS_OLED_ENABLED
  configASSERT(xTaskCreate(Motor1Test_OledTask, "motor1_oled", 512U, NULL,
                          1U, NULL) == pdPASS);
#endif
}
