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
  PROBLEM_REQUEST_BUSY
} ProblemRequestResult;

typedef enum {
  PROBLEM_STATUS_IDLE = 0, PROBLEM_STATUS_RUNNING, PROBLEM_STATUS_FINISHED,
  PROBLEM_STATUS_STOPPED, PROBLEM_STATUS_FAULT, PROBLEM_STATUS_QUEUED,
  PROBLEM_STATUS_STOPPING
} ProblemStatusCode;

typedef struct {
  uint32_t line_length_mm; /* Q1 fixed 600 for this trial; Q2 300..600. */
  uint32_t direction_deg; /* Reserved for Q3. Up=0, clockwise positive. */
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
  uint32_t reason; /* Trial protocol reason values, 0..8 currently. */
  uint32_t elapsed_ms;
  uint32_t line_length_mm;
} ProblemStatus;

const ProblemDescriptor *ProblemApi_Describe(ProblemId id);
ProblemParams ProblemApi_DefaultParams(ProblemId id);
ProblemRequestResult ProblemApi_ValidateStart(ProblemId id,
                                             const ProblemParams *params);
/* Existing Q1/Q2 SWD encodings remain fixed. Unsupported IDs return zero. */
uint32_t ProblemApi_StartCommand(ProblemId id);

#endif
