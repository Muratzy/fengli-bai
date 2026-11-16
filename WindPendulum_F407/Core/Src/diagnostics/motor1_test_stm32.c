#include "diagnostics/motor1_test.h"

#include "FreeRTOS.h"
#include "app_config.h"
#include "main.h"
#include "queue.h"
#include "task.h"
#if APP_MOTOR1_STATUS_OLED_ENABLED
#include "kk_oled.h"
#endif

#include <stdio.h>

#define MOTOR1_TEST_PERIOD_MS 10U
#define MOTOR1_TEST_LOG_MS   100U

typedef struct {
  uint32_t timestamp_ms;
  Motor1Test test;
  uint16_t current_adc;
  uint16_t ccr[4];
  uint8_t key_bits;
  uint8_t en_command;
  uint8_t en_pin;
  uint8_t adc_valid;
  uint32_t step_gap_ms;
  uint32_t max_step_gap_ms;
} Motor1Test_Snapshot;

static QueueHandle_t g_motor1_log_queue;
#if APP_MOTOR1_STATUS_OLED_ENABLED
static QueueHandle_t g_motor1_oled_queue;
#endif
static IWDG_HandleTypeDef g_motor1_watchdog;
static const char *const g_motor1_state_names[] = {"RELEASE", "READY", "HOLD", "RUN"};
static const char *const g_motor1_reason_names[] = {
    "BOOT", "KEY_RELEASED", "STOP_KEY", "TIME_LIMIT", "LOOP_DELAY", "EN_LOW"};

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

static uint16_t Motor1Test_ReadCurrent(uint8_t *valid)
{
  ADC_ChannelConfTypeDef channel = {0};
  uint16_t adc = 0U;
  channel.Channel = ADC_CHANNEL_8;
  channel.Rank = 1;
  channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  *valid = 0U;
  if ((HAL_ADC_ConfigChannel(&hadc1, &channel) == HAL_OK) &&
      (HAL_ADC_Start(&hadc1) == HAL_OK))
  {
    if (HAL_ADC_PollForConversion(&hadc1, 1U) == HAL_OK)
    {
      adc = (uint16_t)HAL_ADC_GetValue(&hadc1);
      *valid = 1U;
    }
    (void)HAL_ADC_Stop(&hadc1);
  }
  return adc;
}

static void Motor1Test_Task(void *argument)
{
  (void)argument;
  Motor1Test_Snapshot snapshot = {0};
  Motor1Test_State previous_state = MOTOR1_TEST_WAIT_RELEASE;
  TickType_t wake_time = xTaskGetTickCount();
  uint32_t log_time_ms = HAL_GetTick();

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
    bool run_pressed = (keys & KEY1_Pin) == 0U;
    bool stop_pressed = (keys & (KEY2_Pin | KEY3_Pin)) !=
                        (KEY2_Pin | KEY3_Pin);
    snapshot.key_bits = (run_pressed ? 1U : 0U) |
                        ((keys & KEY2_Pin) == 0U ? 2U : 0U) |
                        ((keys & KEY3_Pin) == 0U ? 4U : 0U);
    snapshot.step_gap_ms = now_ms - snapshot.test.last_step_ms;
    if (snapshot.step_gap_ms > snapshot.max_step_gap_ms)
    {
      snapshot.max_step_gap_ms = snapshot.step_gap_ms;
    }
    Motor1Test_Step(&snapshot.test, now_ms, run_pressed, stop_pressed);
    Motor1Test_Apply(snapshot.test.duty_permille);
    snapshot.en_command = 1U;
    snapshot.en_pin = (HAL_GPIO_ReadPin(MOTOR_ENABLE_GPIO_Port,
                       MOTOR_ENABLE_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    if ((snapshot.test.state == MOTOR1_TEST_RUNNING) &&
        (snapshot.en_pin == 0U) &&
        (snapshot.test.run_elapsed_ms >= 50U))
    {
      Motor1Test_Stop(&snapshot.test, now_ms, MOTOR1_TEST_EN_LOW);
      Motor1Test_Apply(0U);
    }

    snapshot.timestamp_ms = now_ms;
    snapshot.ccr[0] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    snapshot.ccr[1] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    snapshot.ccr[2] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    snapshot.ccr[3] = (uint16_t)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    if ((snapshot.test.state != previous_state) ||
        ((uint32_t)(now_ms - log_time_ms) >= MOTOR1_TEST_LOG_MS))
    {
      snapshot.current_adc = Motor1Test_ReadCurrent(&snapshot.adc_valid);
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
      "MOTOR1_BENCH,EN=HIGH,PE9=TIM1_CH1,20kHz,max=300permille,run=3000ms\r\n"
      "KEY1=PC13,KEY2=PC0,KEY3=PC1. Hold KEY1 for 1s; release or KEY2/3 stops.\r\n"
      "time_ms,state,reason,duty_permille,en_command,en_pin,ccr1,ccr2,ccr3,ccr4,"
      "current_adc,adc_valid,keys,run_ms,step_gap_ms,max_step_gap_ms\r\n";
  Motor1Test_Snapshot snapshot;
  char line[192];

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)banner, sizeof(banner) - 1U, 50U);
  for (;;)
  {
    if (xQueueReceive(g_motor1_log_queue, &snapshot, portMAX_DELAY) == pdPASS)
    {
      int length = snprintf(line, sizeof(line),
          "%lu,%s,%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu\r\n",
          (unsigned long)snapshot.timestamp_ms,
          g_motor1_state_names[snapshot.test.state],
          g_motor1_reason_names[snapshot.test.stop_reason],
          (unsigned)snapshot.test.duty_permille, (unsigned)snapshot.en_command,
          (unsigned)snapshot.en_pin, (unsigned)snapshot.ccr[0],
          (unsigned)snapshot.ccr[1], (unsigned)snapshot.ccr[2],
          (unsigned)snapshot.ccr[3], (unsigned)snapshot.current_adc,
          (unsigned)snapshot.adc_valid, (unsigned)snapshot.key_bits,
          (unsigned long)snapshot.test.run_elapsed_ms,
          (unsigned long)snapshot.step_gap_ms,
          (unsigned long)snapshot.max_step_gap_ms);
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
        (void)snprintf(line, sizeof(line), "KEY1 PC13: %s",
                       (snapshot.key_bits & 1U) != 0U ? "DOWN" : "UP");
        OLED_DrawUTF8(0, 10, line);
        (void)snprintf(line, sizeof(line), "PWM1:%u.%u%% 20kHz",
                       (unsigned)(snapshot.test.duty_permille / 10U),
                       (unsigned)(snapshot.test.duty_permille % 10U));
        OLED_DrawUTF8(0, 20, line);
        (void)snprintf(line, sizeof(line), "CCR1:%u/%lu",
                       (unsigned)snapshot.ccr[0], (unsigned long)htim1.Init.Period);
        OLED_DrawUTF8(0, 30, line);
        (void)snprintf(line, sizeof(line), "EN SET:%u READ:%u",
                       (unsigned)snapshot.en_command, (unsigned)snapshot.en_pin);
        OLED_DrawUTF8(0, 40, line);
        (void)snprintf(line, sizeof(line), "STOP:%s",
                       g_motor1_reason_names[snapshot.test.stop_reason]);
        OLED_DrawUTF8(0, 50, line);
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
