#ifndef PROBLEM_API_H
#define PROBLEM_API_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  PROBLEM_NONE = 0, PROBLEM_Q1 = 1, PROBLEM_Q2, PROBLEM_Q3,
  PROBLEM_Q4, PROBLEM_Q5, PROBLEM_Q6, PROBLEM_Q7
} ProblemId;

typedef enum {
  PROBLEM_REQUEST_ACCEPTED = 0, PROBLEM_REQUEST_INVALID_ID,
  PROBLEM_REQUEST_INVALID_PARAMS, PROBLEM_REQUEST_NOT_IMPLEMENTED,
  PROBLEM_REQUEST_BUSY, PROBLEM_REQUEST_MIXER_UNCALIBRATED
} ProblemRequestResult;

typedef enum {
  PROBLEM_STATUS_IDLE = 0, PROBLEM_STATUS_RUNNING, PROBLEM_STATUS_FINISHED,
  PROBLEM_STATUS_STOPPED, PROBLEM_STATUS_FAULT, PROBLEM_STATUS_QUEUED,
  PROBLEM_STATUS_STOPPING
} ProblemStatusCode;

typedef struct {
  uint32_t line_length_mm; /* Q1 fixed 600; Q2 300..600; Q3 200..600. */
  uint32_t direction_deg; /* Q3: up=0, clockwise positive, 10 degree steps. */
  uint32_t radius_mm; /* Reserved for Q5/Q6. */
} ProblemParams;

typedef struct {
  ProblemId id;
  const char *name;
  bool implemented;
} ProblemDescriptor;

typedef struct {
  ProblemId id;
  ProblemStatusCode state;
  uint32_t reason; /* Trial protocol reason, see Q1Swing_Reason. */
  uint32_t elapsed_ms;
  uint32_t line_length_mm;
  uint32_t direction_deg;
} ProblemStatus;

const ProblemDescriptor *ProblemApi_Describe(ProblemId id);
ProblemParams ProblemApi_DefaultParams(ProblemId id);
ProblemRequestResult ProblemApi_ValidateStart(ProblemId id,
                                             const ProblemParams *params);
/* Existing Q1/Q2 SWD encodings remain fixed. Unsupported IDs return zero. */
uint32_t ProblemApi_StartCommand(ProblemId id);
ProblemId ProblemApi_DecodeStartCommand(uint32_t command);

#endif
