#include "services/q1_vcom.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint16_t TestCrc16(const char *text)
{
  uint16_t crc = 0xFFFFU;
  for (; *text != '\0'; ++text) {
    crc ^= (uint16_t)(uint8_t)*text << 8U;
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      crc = (uint16_t)((crc << 1U) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    }
  }
  return crc;
}

static void AddCrc(char *frame, size_t capacity)
{
  const size_t length = strlen(frame);
  assert(length + 6U < capacity);
  (void)snprintf(frame + length, capacity - length, "*%04X",
                 (unsigned)TestCrc16(frame));
}

static void TestStartParsing(void)
{
  char frame[Q1_VCOM_LINE_SIZE] = "!WP1,123,START,28000,7000,2000,0,0,500,16858";
  AddCrc(frame, sizeof(frame));
  char bad_frame[Q1_VCOM_LINE_SIZE];
  memcpy(bad_frame, frame, strlen(frame) + 1U);
  bad_frame[strlen(bad_frame) - 2U] =
      (bad_frame[strlen(bad_frame) - 2U] == '0') ? '1' : '0';
  Q1Vcom_Request request = {0};
  assert(Q1Vcom_Parse(frame, &request));
  assert(request.id == 123U && request.action == Q1_VCOM_START);
  assert(request.tuning.startup_kp == 28.0F);
  assert(request.tuning.startup_kd == 7.0F);
  assert(request.tuning.amplitude_kp == 2.0F);
  assert(request.tuning.amplitude_ki == 0.0F);
  assert(request.tuning.amplitude_kd == 0.0F);
  assert(request.tuning.damping_compensation == 0.5F);
  assert(request.tuning.target_amplitude_deg == 16.858F);
  assert(!Q1Vcom_Parse(bad_frame, &request));
}

static void TestReplyFormatting(void)
{
  char line[256];
  Q1Vcom_Reply reply = {
      .request = {.id = 456U, .action = Q1_VCOM_STOP}, .error = NULL};
  const int length = Q1Vcom_FormatReply(line, sizeof(line), &reply);
  assert(length > 0);
  assert(strcmp(line, "!WP1,456,OK,STOP*") != 0);
  assert(line[length - 2] == '\r' && line[length - 1] == '\n');
  char *star = strchr(line, '*');
  assert(star != NULL);
  *star = '\0';
  unsigned parsed_crc = 0U;
  assert(sscanf(star + 1, "%4X", &parsed_crc) == 1);
  assert(parsed_crc == TestCrc16(line));
}

int main(void)
{
  TestStartParsing();
  TestReplyFormatting();
  puts("Q1 VCOM protocol tests passed");
  return 0;
}
