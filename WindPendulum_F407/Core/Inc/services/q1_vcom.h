#ifndef Q1_VCOM_H
#define Q1_VCOM_H

#include "problems/q1_swing.h"
#include <stddef.h>

typedef enum { Q1_VCOM_INFO, Q1_VCOM_START, Q1_VCOM_STOP, Q1_VCOM_RESET } Q1Vcom_Action;

typedef struct {
  uint32_t id;
  Q1Vcom_Action action;
  Q1Swing_Tuning tuning;
} Q1Vcom_Request;

typedef struct {
  Q1Vcom_Request request;
  const char *error;
  uint32_t telemetry_period_ms;
} Q1Vcom_Reply;

#define Q1_VCOM_LINE_SIZE 256U

bool Q1Vcom_Parse(char *line, Q1Vcom_Request *request);
int Q1Vcom_FormatReply(char *line, size_t size, const Q1Vcom_Reply *reply);

#endif
