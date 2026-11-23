#include "pendulum_ui.h"

#include "FreeRTOS.h"
#include "main.h"
#include "pendulum_app.h"
#include "problems/problem_q2.h"
#include "app_config.h"
#include "queue.h"
#include "task.h"

#include "kk_oled.h"
#include "kk_ui.h"
#include "kk_ui_draw.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#define UI_CONTRAST_CHANGED_EVENT 1U
#define UI_PAGE_ROOT              1U
#define UI_PAGE_STATUS            2U
#define UI_PAGE_SENSORS           3U
#define UI_PAGE_ATTITUDE          4U
#define UI_PAGE_ACCEL             5U
#define UI_PAGE_GYRO              6U
#define UI_PAGE_CALIBRATION       7U
#define UI_PAGE_DISPLAY           8U
#define UI_PAGE_ABOUT             9U

extern const uint8_t u8g2_font_5x7_tf[];

static PendulumDisplayState g_display_state;
#if !APP_LASER_CAL_CAPTURE_ENABLED && !APP_Q2_UI_ENABLED
static int32_t g_contrast = 128;

static const KK_UI_MenuItem root_items[] = {
    {"Live Status", KK_UI_MENU_PAGE, UI_PAGE_STATUS},
    {"IMU Data", KK_UI_MENU_PAGE, UI_PAGE_SENSORS},
    {"Gyro Offsets", KK_UI_MENU_PAGE, UI_PAGE_CALIBRATION},
    {"Display", KK_UI_MENU_PAGE, UI_PAGE_DISPLAY},
    {"About", KK_UI_MENU_PAGE, UI_PAGE_ABOUT}
};

static const KK_UI_MenuItem sensor_items[] = {
    {"Attitude", KK_UI_MENU_PAGE, UI_PAGE_ATTITUDE},
    {"Accelerometer", KK_UI_MENU_PAGE, UI_PAGE_ACCEL},
    {"Gyroscope", KK_UI_MENU_PAGE, UI_PAGE_GYRO}
};

static const KK_UI_MenuItem display_items[] = {
    {"Contrast", KK_UI_MENU_INT, 0U}
};

static const KK_UI_PageRoute routes[] = {
    {KK_UI_PAGE_MENU, 0U},
    {KK_UI_PAGE_CUSTOM, 0U},
    {KK_UI_PAGE_MENU, 1U},
    {KK_UI_PAGE_CUSTOM, 1U},
    {KK_UI_PAGE_CUSTOM, 2U},
    {KK_UI_PAGE_CUSTOM, 3U},
    {KK_UI_PAGE_CUSTOM, 4U},
    {KK_UI_PAGE_MENU, 2U},
    {KK_UI_PAGE_INFO, 0U}
};

static const KK_UI_MenuPage menu_pages[] = {
    {"Pendulum", root_items, NULL, sizeof(root_items) / sizeof(root_items[0])},
    {"IMU Data", sensor_items, NULL,
     sizeof(sensor_items) / sizeof(sensor_items[0])},
    {"Display", display_items, NULL,
     sizeof(display_items) / sizeof(display_items[0])}
};

static const KK_UI_InfoRow about_rows[] = {
    {"Target", "STM32F407VET6"},
    {"Board", "TKX Youth"},
    {"IMU", "ICM-45686"},
    {"OLED", "SSD1306 128x64"},
    {"RTOS", "FreeRTOS LTS"}
};

static const KK_UI_InfoPage info_pages[] = {
    {"About", about_rows, sizeof(about_rows) / sizeof(about_rows[0])}
};

static const KK_UI_IntBinding int_bindings[] = {
    {"OLED Contrast", &g_contrast, 8, 255, 16U, "", UI_CONTRAST_CHANGED_EVENT}
};

static const KK_UI_App app = {
    .root_page = UI_PAGE_ROOT,
    .routes = routes,
    .route_count = sizeof(routes) / sizeof(routes[0]),
    .home_pages = NULL,
    .home_page_count = 0U,
    .menu_pages = menu_pages,
    .menu_page_count = sizeof(menu_pages) / sizeof(menu_pages[0]),
    .info_pages = info_pages,
    .info_page_count = sizeof(info_pages) / sizeof(info_pages[0]),
    .custom_page_count = 5U,
    .int_bindings = int_bindings,
    .int_binding_count = sizeof(int_bindings) / sizeof(int_bindings[0]),
    .bool_bindings = NULL,
    .bool_binding_count = 0U,
    .confirm_descs = NULL,
    .confirm_desc_count = 0U,
    .fonts = {u8g2_font_5x7_tf, u8g2_font_5x7_tf, u8g2_font_5x7_tf},
    .texts = {"Back", "Cancel", "Set", "ON", "OFF", "Info"}
};
#endif

#if !APP_LASER_CAL_CAPTURE_ENABLED && !APP_Q2_UI_ENABLED
static void UI_DrawHeader(const char *title)
{
  OLED_SetFont(u8g2_font_5x7_tf);
  OLED_DrawUTF8(2, 1, title);
  OLED_DrawHLine(1, 10, 126U);
}

static void UI_FormatTenths(char *buffer, size_t length, const char *name,
                            float value)
{
  const long tenths = (long)(value * 10.0F);
  long fraction = tenths % 10L;
  if (fraction < 0L) {
    fraction = -fraction;
  }
  (void)snprintf(buffer, length, "%s%+ld.%01ld", name, tenths / 10L,
                 fraction);
}

static void UI_FormatAxis(char *buffer, size_t length, const char *axis,
                          int16_t raw, float value, const char *unit)
{
  const long hundredths = (long)(value * 100.0F);
  long fraction = hundredths % 100L;
  if (fraction < 0L) {
    fraction = -fraction;
  }
  (void)snprintf(buffer, length, "%s %d %+ld.%02ld%s", axis, raw,
                 hundredths / 100L, fraction, unit);
}

static const char *UI_ModeText(void)
{
  if (g_display_state.imu_online == 0U) {
    return "IMU FAULT";
  }
  return (g_display_state.calibrated != 0U) ? "READY" : "INITIALIZING";
}

void KK_UI_CustomOnEnter(KK_UI_PageId page)
{
  (void)page;
}

void KK_UI_CustomOnLeave(KK_UI_PageId page)
{
  (void)page;
}

void KK_UI_CustomOnInput(KK_UI_PageId page, KK_UI_InputEvent event)
{
  (void)page;
  if (event.action == KK_UI_INPUT_OK) {
    (void)KK_UI_CustomRequestClose();
  }
}

bool KK_UI_CustomOnTick(KK_UI_PageId page, uint32_t now_ms)
{
  (void)page;
  (void)now_ms;
  return false;
}

void KK_UI_CustomOnDraw(KK_UI_PageId page, int16_t x_offset,
                        int16_t clip_x, uint16_t clip_width)
{
  const PendulumTelemetry *data = &g_display_state.telemetry;
  char line[32];
  const int16_t x = (int16_t)(3 + x_offset);

  OLED_SetClipWindow(clip_x, 0, clip_width, 64U);
  if (page == 0U) {
    UI_DrawHeader("LIVE STATUS");
    (void)snprintf(line, sizeof(line), "IMU: %s",
                   g_display_state.imu_online ? "ICM45686 OK" : "OFFLINE");
    OLED_DrawUTF8(x, 13, line);
    (void)snprintf(line, sizeof(line), "MODE: %s", UI_ModeText());
    OLED_DrawUTF8(x, 23, line);
    UI_FormatTenths(line, sizeof(line), "ROLL: ", data->roll_deg);
    OLED_DrawUTF8(x, 33, line);
    UI_FormatTenths(line, sizeof(line), "PITCH: ", data->pitch_deg);
    OLED_DrawUTF8(x, 43, line);
    (void)snprintf(line, sizeof(line), "GYRO: FIXED OFFSETS");
    OLED_DrawUTF8(x, 53, line);
  } else if (page == 1U) {
    UI_DrawHeader("ATTITUDE");
    UI_FormatTenths(line, sizeof(line), "ROLL  ", data->roll_deg);
    OLED_DrawUTF8(x, 18, line);
    UI_FormatTenths(line, sizeof(line), "PITCH ", data->pitch_deg);
    OLED_DrawUTF8(x, 30, line);
    (void)snprintf(line, sizeof(line), "ACCEL WEIGHT %u/1000",
                   (unsigned)(data->accel_weight * 1000.0F));
    OLED_DrawUTF8(x, 44, line);
  } else if (page == 2U) {
    UI_DrawHeader("ACCELEROMETER");
    UI_FormatAxis(line, sizeof(line), "AX", data->imu.accel_raw[0],
                  data->imu.accel_g[0], "g");
    OLED_DrawUTF8(x, 17, line);
    UI_FormatAxis(line, sizeof(line), "AY", data->imu.accel_raw[1],
                  data->imu.accel_g[1], "g");
    OLED_DrawUTF8(x, 29, line);
    UI_FormatAxis(line, sizeof(line), "AZ", data->imu.accel_raw[2],
                  data->imu.accel_g[2], "g");
    OLED_DrawUTF8(x, 41, line);
    UI_FormatTenths(line, sizeof(line), "TEMP ", data->imu.temperature_c);
    if (strlen(line) + 2U < sizeof(line)) {
      (void)strcat(line, " C");
    }
    OLED_DrawUTF8(x, 53, line);
  } else if (page == 3U) {
    UI_DrawHeader("GYROSCOPE");
    UI_FormatAxis(line, sizeof(line), "GX", data->imu.gyro_raw[0],
                  data->imu.gyro_dps[0], "dps");
    OLED_DrawUTF8(x, 17, line);
    UI_FormatAxis(line, sizeof(line), "GY", data->imu.gyro_raw[1],
                  data->imu.gyro_dps[1], "dps");
    OLED_DrawUTF8(x, 29, line);
    UI_FormatAxis(line, sizeof(line), "GZ", data->imu.gyro_raw[2],
                  data->imu.gyro_dps[2], "dps");
    OLED_DrawUTF8(x, 41, line);
    (void)snprintf(line, sizeof(line), "TEMP %+ld C",
                   (long)data->imu.temperature_c);
    OLED_DrawUTF8(x, 53, line);
  } else {
    UI_DrawHeader("FIXED GYRO OFFSETS");
    (void)snprintf(line, sizeof(line), "MODE: %s", UI_ModeText());
    OLED_DrawUTF8(x, 15, line);
    (void)snprintf(line, sizeof(line), "NO STARTUP CALIBRATION");
    OLED_DrawUTF8(x, 27, line);
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
      const char *name = (axis == 0U) ? "X" : (axis == 1U) ? "Y" : "Z";
      UI_FormatTenths(line, sizeof(line), name,
                      g_display_state.gyro_bias_dps[axis]);
      OLED_DrawUTF8((int16_t)(x + axis * 39U), 39, line);
    }
    OLED_DrawUTF8(x, 53, "OFFSETS IN DEG/S");
  }
  OLED_ResetClipWindow();
}
#endif

#if APP_LASER_CAL_CAPTURE_ENABLED || APP_Q2_UI_ENABLED
void KK_UI_CustomOnEnter(KK_UI_PageId page)
{
  (void)page;
}

void KK_UI_CustomOnLeave(KK_UI_PageId page)
{
  (void)page;
}

void KK_UI_CustomOnInput(KK_UI_PageId page, KK_UI_InputEvent event)
{
  (void)page;
  (void)event;
}

bool KK_UI_CustomOnTick(KK_UI_PageId page, uint32_t now_ms)
{
  (void)page;
  (void)now_ms;
  return false;
}

void KK_UI_CustomOnDraw(KK_UI_PageId page, int16_t x_offset,
                        int16_t clip_x, uint16_t clip_width)
{
  (void)page;
  (void)x_offset;
  (void)clip_x;
  (void)clip_width;
}
#endif

#if !APP_LASER_CAL_CAPTURE_ENABLED && !APP_Q2_UI_ENABLED
static KK_UI_Input UI_ReadKeys(void)
{
  KK_UI_Input input = {0};
  if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
    input.keys |= KK_UI_KEY_UP;
  }
  if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET) {
    input.keys |= KK_UI_KEY_DOWN;
  }
  if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
    input.keys |= KK_UI_KEY_OK;
  }
  return input;
}
#endif

void PendulumUI_Task(void *argument)
{
#if APP_LASER_CAL_CAPTURE_ENABLED
  QueueHandle_t display_queue = (QueueHandle_t)argument;
  PendulumDisplayState display = {0};
  PendulumDisplayState incoming;
  TickType_t wake_time = xTaskGetTickCount();

  if (OLED_Init() != OLED_OK) {
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000U));
  }
  OLED_SetFont(u8g2_font_5x7_tf);
  OLED_SetFontPosition(OLED_FONT_POS_TOP);

  for (;;) {
    if (xQueueReceive(display_queue, &incoming, 0U) == pdPASS) {
      display = incoming;
    }
    OLED_Clear();
    OLED_DrawUTF8(2, 1, "LASER CALIBRATION");
    OLED_DrawHLine(1, 11, 126U);
    if (display.imu_online == 0U) {
      OLED_DrawUTF8(2, 20, "IMU WAIT");
      OLED_DrawUTF8(2, 34, "CHECK SENSOR");
    } else if (display.telemetry.q1_state == 1U) {
      char line[24];
      (void)snprintf(line, sizeof(line), "POINT %u/3", (unsigned)(display.telemetry.q1_switches + 1U));
      OLED_DrawUTF8(2, 18, line);
      (void)snprintf(line, sizeof(line), "VALID %lu s",
          (unsigned long)(display.telemetry.q1_run_ms / 1000U));
      OLED_DrawUTF8(2, 31, line);
      OLED_DrawUTF8(2, 44, "KEY2 TO FINISH");
    } else if (display.telemetry.q1_switches >= 3U) {
      OLED_DrawUTF8(2, 22, "3 POINTS COMPLETE");
      OLED_DrawUTF8(2, 38, "DATA FLAG CLOSED");
    } else {
      char line[24];
      (void)snprintf(line, sizeof(line), "POINT %u/3 READY",
          (unsigned)(display.telemetry.q1_switches + 1U));
      OLED_DrawUTF8(2, 20, line);
      OLED_DrawUTF8(2, 34, "KEY1 TO START");
    }
    OLED_DrawUTF8(2, 55, "MOTOR OUTPUT OFF");
    (void)OLED_Update();
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(100U));
  }
#elif APP_Q2_UI_ENABLED
  QueueHandle_t display_queue = (QueueHandle_t)argument;
  PendulumDisplayState incoming;
  TickType_t wake_time = xTaskGetTickCount();
  uint32_t length_cm = APP_Q2_DEFAULT_LENGTH_MM / 10U;
  uint8_t previous_keys = 0U;
  uint8_t screen = 0U; /* 0=setup, 1=running, 2=result */

  if (OLED_Init() != OLED_OK) {
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000U));
  }
  OLED_SetFont(u8g2_font_5x7_tf);
  OLED_SetFontPosition(OLED_FONT_POS_TOP);

  for (;;) {
    if (xQueueReceive(display_queue, &incoming, 0U) == pdPASS) {
      g_display_state = incoming;
      if (incoming.telemetry.problem_id == PROBLEM_Q2 &&
          incoming.telemetry.target_length_mm >= 300U &&
          incoming.telemetry.target_length_mm <= 600U) {
        length_cm = incoming.telemetry.target_length_mm / 10U;
      }
      if (screen == 1U && incoming.telemetry.problem_id == PROBLEM_Q2 &&
          incoming.telemetry.q1_state != PROBLEM_Q2_RUNNING) {
        if (incoming.telemetry.q1_state >= PROBLEM_Q2_FINISHED) screen = 2U;
      }
    }

    const uint8_t keys =
        (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET ? 1U : 0U) |
        (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET ? 2U : 0U) |
        (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET ? 4U : 0U);
    const uint8_t pressed = keys & (uint8_t)~previous_keys;
    previous_keys = keys;

    if (screen != 1U) {
      if ((pressed & 2U) != 0U && length_cm < 60U) length_cm += 10U;
      if ((pressed & 4U) != 0U && length_cm > 30U) length_cm -= 10U;
      if ((pressed & 1U) != 0U &&
          PendulumApp_RequestQ2Start(length_cm * 10U)) screen = 1U;
    } else if ((pressed & 4U) != 0U) {
      (void)PendulumApp_RequestQ2Cancel();
    }

    OLED_Clear();
    OLED_SetFont(u8g2_font_5x7_tf);
    OLED_DrawUTF8(2, 1, "Q2 VARIABLE LINE");
    OLED_DrawHLine(1, 11, 126U);
    char line[28];
    if (screen == 0U) {
      (void)snprintf(line, sizeof(line), "LENGTH %lu cm", (unsigned long)length_cm);
      OLED_DrawUTF8(2, 20, line);
      OLED_DrawUTF8(2, 34, "KEY2 +  KEY3 -");
      OLED_DrawUTF8(2, 48, "KEY1 START");
    } else if (screen == 1U) {
      const float roll = g_display_state.telemetry.roll_deg * 0.01745329252F;
      const float pitch = g_display_state.telemetry.pitch_deg * 0.01745329252F;
      const float actual_cm = 2.0F * fabsf(APP_PIVOT_GROUND_HEIGHT_M *
          tanf(pitch) / fmaxf(0.1F, cosf(roll))) * 100.0F;
      (void)snprintf(line, sizeof(line), "TARGET %lu cm", (unsigned long)length_cm);
      OLED_DrawUTF8(2, 18, line);
      const long actual_tenths = (long)(actual_cm * 10.0F);
      (void)snprintf(line, sizeof(line), "ACTUAL %ld.%ld cm",
          actual_tenths / 10L, actual_tenths % 10L);
      OLED_DrawUTF8(2, 30, line);
      (void)snprintf(line, sizeof(line), "RUN %lu ms",
          (unsigned long)g_display_state.telemetry.q1_run_ms);
      OLED_DrawUTF8(2, 42, line);
      OLED_DrawUTF8(2, 54, "KEY3 STOP  UART PID");
    } else {
      const char *result = (g_display_state.telemetry.q1_state == PROBLEM_Q2_FINISHED) ?
          "TIME UP" : (g_display_state.telemetry.q1_state == PROBLEM_Q2_STOPPED) ?
          "STOPPED" : "FAULT";
      (void)snprintf(line, sizeof(line), "%s R%u", result,
          (unsigned)g_display_state.telemetry.q1_reason);
      OLED_DrawUTF8(2, 22, line);
      OLED_DrawUTF8(2, 36, "KEY1 RESTART");
      OLED_DrawUTF8(2, 50, "KEY2+/KEY3-");
    }
    (void)OLED_Update();
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(50U));
  }
#else
  QueueHandle_t display_queue = (QueueHandle_t)argument;
  PendulumDisplayState incoming;
  KK_UI_EventId event;
  TickType_t wake_time = xTaskGetTickCount();

  if (OLED_Init() != OLED_OK ||
      OLED_SetContrast((uint8_t)g_contrast) != OLED_OK ||
      KK_UI_Init(&app) != KK_UI_OK) {
    for (;;) {
      vTaskDelay(pdMS_TO_TICKS(1000U));
    }
  }

  for (;;) {
    if (xQueueReceive(display_queue, &incoming, 0U) == pdPASS) {
      g_display_state = incoming;
      KK_UI_Invalidate();
    }
    (void)KK_UI_Update(HAL_GetTick(), UI_ReadKeys());
    while (KK_UI_PollEvent(&event)) {
      if (event == UI_CONTRAST_CHANGED_EVENT) {
        (void)OLED_SetContrast((uint8_t)g_contrast);
      }
    }
    vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(10U));
  }
#endif
}
