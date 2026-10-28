#include "pendulum_ui.h"

#include "FreeRTOS.h"
#include "main.h"
#include "pendulum_app.h"
#include "queue.h"
#include "task.h"

#include "kk_oled.h"
#include "kk_ui.h"
#include "kk_ui_draw.h"

#include <stdio.h>
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
static int32_t g_contrast = 128;

static const KK_UI_MenuItem root_items[] = {
    {"Live Status", KK_UI_MENU_PAGE, UI_PAGE_STATUS},
    {"IMU Data", KK_UI_MENU_PAGE, UI_PAGE_SENSORS},
    {"Calibration", KK_UI_MENU_PAGE, UI_PAGE_CALIBRATION},
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
  return (g_display_state.calibrated != 0U) ? "READY" : "CALIBRATING";
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
    if (g_display_state.calibrated != 0U) {
      (void)snprintf(line, sizeof(line), "CAL: COMPLETE");
    } else if (g_display_state.calibration_samples == 0U) {
      (void)snprintf(line, sizeof(line), "CAL: HOLD STILL");
    } else {
      (void)snprintf(line, sizeof(line), "CAL: %lu/5000",
                     (unsigned long)g_display_state.calibration_samples);
    }
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
    UI_DrawHeader("STARTUP CALIBRATION");
    if (g_display_state.calibrated != 0U) {
      (void)snprintf(line, sizeof(line), "COMPLETE");
    } else if (g_display_state.calibration_samples == 0U) {
      (void)snprintf(line, sizeof(line), "HOLD STILL");
    } else {
      (void)snprintf(line, sizeof(line), "CALIBRATING");
    }
    OLED_DrawUTF8(x, 15, line);
    (void)snprintf(line, sizeof(line), "SAMPLES %lu/5000",
                   (unsigned long)g_display_state.calibration_samples);
    OLED_DrawUTF8(x, 27, line);
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
      const char *name = (axis == 0U) ? "X" : (axis == 1U) ? "Y" : "Z";
      UI_FormatTenths(line, sizeof(line), name,
                      g_display_state.gyro_bias_dps[axis]);
      OLED_DrawUTF8((int16_t)(x + axis * 39U), 39, line);
    }
    KK_UI_DrawProgressBar(x, 53, 120U, 8U,
                          g_display_state.calibration_samples, 5000U);
  }
  OLED_ResetClipWindow();
}

static KK_UI_Input UI_ReadKeys(void)
{
  KK_UI_Input input = {0};
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) {
    input.keys |= KK_UI_KEY_UP;
  }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) {
    input.keys |= KK_UI_KEY_DOWN;
  }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
    input.keys |= KK_UI_KEY_OK;
  }
  return input;
}

void PendulumUI_Task(void *argument)
{
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
}
