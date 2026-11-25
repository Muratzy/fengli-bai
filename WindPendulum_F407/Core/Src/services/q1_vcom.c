#include "services/q1_vcom.h"
#include "app_config.h"

#include <stdio.h>
#include <string.h>

static const char *const actions[] = {"INFO", "START", "STOP", "RESET"};

static uint16_t Crc16(const char *line)
{
  uint16_t crc = 0xFFFFU;
  for (; *line != '\0'; ++line) {
    crc ^= (uint16_t)(uint8_t)*line << 8U;
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      crc = (uint16_t)((crc << 1U) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    }
  }
  return crc;
}

static bool ReadUnsigned(const char *field, uint32_t max, uint32_t *value)
{
  *value = 0U;
  if (field == NULL || *field == '\0') return false;
  for (; *field != '\0'; ++field) {
    if (*field < '0' || *field > '9') return false;
    const uint32_t digit = (uint32_t)(*field - '0');
    if (*value > (max - digit) / 10U) return false;
    *value = *value * 10U + digit;
  }
  return true;
}

bool Q1Vcom_Parse(char *line, Q1Vcom_Request *request)
{
  char *suffix = strchr(line, '*');
  if (suffix == NULL || strlen(suffix + 1) != 4U) return false;
  *suffix = '\0';
  uint16_t received = 0U;
  for (const char *hex = suffix + 1; *hex != '\0'; ++hex) {
    if (*hex >= '0' && *hex <= '9') {
      received = (uint16_t)((received << 4U) | (unsigned)(*hex - '0'));
    } else if (*hex >= 'A' && *hex <= 'F') {
      received = (uint16_t)((received << 4U) | (unsigned)(*hex - 'A' + 10));
    } else {
      return false;
    }
  }
  if (Crc16(line) != received) return false;

  char *fields[10];
  unsigned count = 0U;
  char *field = line;
  for (;;) {
    if (count == sizeof(fields) / sizeof(fields[0])) return false;
    fields[count++] = field;
    char *comma = strchr(field, ',');
    if (comma == NULL) break;
    *comma = '\0';
    field = comma + 1;
  }
  if (count < 3U || strcmp(fields[0], "!WP1") != 0 ||
      !ReadUnsigned(fields[1], UINT32_MAX, &request->id) || request->id == 0U) return false;
  unsigned action = 0U;
  while (action < sizeof(actions) / sizeof(actions[0]) &&
         strcmp(fields[2], actions[action]) != 0) ++action;
  if (action >= sizeof(actions) / sizeof(actions[0])) return false;
  request->action = (Q1Vcom_Action)action;
  if (request->action != Q1_VCOM_START) return count == 3U;
  if (count != 10U) return false;

  uint32_t values[7];
  for (unsigned i = 0U; i < 7U; ++i) {
    if (!ReadUnsigned(fields[3U + i], 1000000U, &values[i])) return false;
  }
  request->tuning = (Q1Swing_Tuning){
      .startup_kp = (float)values[0] / 1000.0F,
      .startup_kd = (float)values[1] / 1000.0F,
      .amplitude_kp = (float)values[2] / 1000.0F,
      .amplitude_ki = (float)values[3] / 1000.0F,
      .amplitude_kd = (float)values[4] / 1000.0F,
      .damping_compensation = (float)values[5] / 1000.0F,
      .target_amplitude_deg = (float)values[6] / 1000.0F};
  return true;
}

static unsigned long Milli(float value)
{
  return (unsigned long)(value * 1000.0F + 0.5F);
}

int Q1Vcom_FormatReply(char *line, size_t size, const Q1Vcom_Reply *reply)
{
  const unsigned long id = (unsigned long)reply->request.id;
  const Q1Vcom_Action action = reply->request.action;
  int length;
  if (reply->error != NULL) {
    length = snprintf(line, size, "!WP1,%lu,ERR,%s,%s", id, actions[action], reply->error);
  } else if (action == Q1_VCOM_INFO || action == Q1_VCOM_START) {
    const ProblemQ1_Config config = ProblemQ1_DefaultConfig();
    const Q1Swing_Tuning tuning = (action == Q1_VCOM_START) ?
        reply->request.tuning : Q1Swing_DefaultTuning();
    const float target = tuning.target_amplitude_deg > 0.0F ?
        tuning.target_amplitude_deg : ProblemQ1_TargetAmplitudeDeg(&config);
    length = snprintf(line, size,
        "!WP1,%lu,OK,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"
        "%lu,%lu,%lu,%u,%lu,%lu,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%lu,%u,%lu",
        id, actions[action], Milli(tuning.startup_kp), Milli(tuning.startup_kd),
        Milli(tuning.amplitude_kp), Milli(tuning.amplitude_ki), Milli(tuning.amplitude_kd),
        Milli(tuning.damping_compensation), Milli(target),
        (unsigned long)config.swing_period_ms, (unsigned long)config.startup_ramp_ms,
        (unsigned long)config.duration_ms, APP_Q1_PWM_MAX_PERMILLE,
        Milli(APP_Q1_RATE_FILTER_TAU_S), Milli(APP_Q1_AMPLITUDE_D_FILTER_S),
        APP_Q1_AMPLITUDE_HOLD_MS, APP_Q1_AMPLITUDE_BLEND_MS,
        Milli(APP_Q1_AMPLITUDE_SWITCH_RATIO), Milli(APP_Q1_AMPLITUDE_I_LIMIT),
        Milli(APP_Q1_START_ANGLE_DEG), Milli(APP_Q1_STOP_PITCH_DEG),
        Milli(APP_Q1_STOP_ROLL_DEG), Milli(APP_Q1_STOP_RATE_DPS),
        Milli(APP_Q1_AMPLITUDE_GAIN_LIMIT), Milli(config.ground_height_m),
        Milli(APP_Q1_PITCH_KI), APP_Q1_CONTROL_MODE, Milli(APP_Q1_INTEGRAL_LIMIT),
        APP_Q1_TELEMETRY_PERIOD_MS, Milli(APP_Q1_AMPLITUDE_RELEASE_RATIO));
  } else {
    length = snprintf(line, size, "!WP1,%lu,OK,%s", id, actions[action]);
  }
  if (length < 0 || (size_t)length + 8U > size) return 0;
  const unsigned crc = Crc16(line);
  const int suffix = snprintf(line + length, size - (size_t)length, "*%04X\r\n", crc);
  return length + suffix;
}
