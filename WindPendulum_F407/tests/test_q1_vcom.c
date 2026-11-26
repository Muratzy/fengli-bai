#include "services/q1_vcom.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
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

static void TestQ3Protocol(void)
{
  char frame[Q1_VCOM_LINE_SIZE] =
      "!WP1,789,START,28000,7000,2000,0,0,500,7180,15000,3,40,250";
  AddCrc(frame, sizeof(frame));
  Q1Vcom_Request request = {0};
  assert(Q1Vcom_Parse(frame, &request));
  assert(request.action == Q1_VCOM_START && request.problem_id == PROBLEM_Q3);
  assert(request.direction_deg == 40U && request.line_length_mm == 250U);
  assert(request.tuning.duration_ms == 15000U);

  char info[Q1_VCOM_LINE_SIZE] = "!WP1,790,Q3INFO";
  AddCrc(info, sizeof(info));
  assert(Q1Vcom_Parse(info, &request));
  assert(request.action == Q1_VCOM_Q3INFO);
  Q1Vcom_Reply reply = {.request = request};
  const int length = Q1Vcom_FormatReply(info, sizeof(info), &reply);
  assert(length > 0);
  assert(strstr(info, "!WP1,790,OK,Q3INFO,3,") == info);
  char *star = strchr(info, '*');
  assert(star != NULL);
  *star = '\0';
  assert((unsigned)strtoul(star + 1, NULL, 16) == TestCrc16(info));

  reply.request = (Q1Vcom_Request){.id = 789U, .action = Q1_VCOM_START,
      .problem_id = PROBLEM_Q3, .direction_deg = 40U, .line_length_mm = 250U,
      .tuning = {.startup_kp = 28.0F, .startup_kd = 7.0F,
        .amplitude_kp = 2.0F, .damping_compensation = 0.5F,
        .target_amplitude_deg = 7.18F, .duration_ms = 15000U}};
  assert(Q1Vcom_FormatReply(info, sizeof(info), &reply) > 0);
  assert(strstr(info, ",3,40,250*") != NULL);
  reply.error = "MIXER";
  assert(Q1Vcom_FormatReply(info, sizeof(info), &reply) > 0);
  assert(strstr(info, "!WP1,789,ERR,START,MIXER*") == info);
  assert(strstr(info, ",3,40,250") == NULL);
}

static void TestQ4Protocol(void)
{
  char line[Q1_VCOM_LINE_SIZE] = "!WP1,900,Q4INFO";
  AddCrc(line, sizeof(line));
  Q1Vcom_Request request = {0};
  assert(Q1Vcom_Parse(line, &request));
  assert(request.action == Q1_VCOM_Q4INFO && request.problem_id == PROBLEM_Q4);

  strcpy(line, "!WP1,901,Q4ARM,1");
  AddCrc(line, sizeof(line));
  assert(!Q1Vcom_Parse(line, &request));
  strcpy(line, "!WP1,901,Q4ARM");
  AddCrc(line, sizeof(line));
  assert(Q1Vcom_Parse(line, &request));
  assert(request.action == Q1_VCOM_Q4ARM && request.problem_id == PROBLEM_Q4);
  Q1Vcom_Reply reply = {.request = request, .telemetry_period_ms = 1000U};
  assert(Q1Vcom_FormatReply(line, sizeof(line), &reply) > 0);
  assert(strstr(line, "!WP1,901,OK,Q4ARM,4,") == line);
  assert(strstr(line, ",20000,5000,30,45,150,8000,2,5,300,50000,240000,10000,8000,850,") != NULL);
  assert(strstr(line, ",1000*") != NULL);
  char *star = strchr(line, '*');
  *star = '\0';
  assert((unsigned)strtoul(star + 1, NULL, 16) == TestCrc16(line));
  reply.error = "BUSY";
  assert(Q1Vcom_FormatReply(line, sizeof(line), &reply) > 0);
  assert(strstr(line, "!WP1,901,ERR,Q4ARM,BUSY*") == line);
}

int main(void)
{
  TestStartParsing();
  TestReplyFormatting();
  TestQ3Protocol();
  TestQ4Protocol();
  puts("Q1 VCOM protocol tests passed");
  return 0;
}
