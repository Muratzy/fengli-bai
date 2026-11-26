#include "problems/problem_api.h"
#include "app_config.h"
#include <stddef.h>

static const ProblemDescriptor descriptors[] = {
    {PROBLEM_Q1, "Q1 line swing", true},
    {PROBLEM_Q2, "Q2 variable line swing", true},
    {PROBLEM_Q3, "Q3 direction line", true},
    {PROBLEM_Q4, "Q4 braking", true},
    {PROBLEM_Q5, "Q5 circle", false},
    {PROBLEM_Q6, "Q6 disturbance recovery", false},
    {PROBLEM_Q7, "Q7 extension", false}};

const ProblemDescriptor *ProblemApi_Describe(ProblemId id)
{
  if (id < PROBLEM_Q1 || id > PROBLEM_Q7) return NULL;
  return &descriptors[(uint32_t)id - 1U];
}

ProblemParams ProblemApi_DefaultParams(ProblemId id)
{
  ProblemParams params = {0};
  if (id == PROBLEM_Q1) params.line_length_mm = (uint32_t)(APP_Q1_LINE_LENGTH_M * 1000.0F + 0.5F);
  if (id == PROBLEM_Q2) params.line_length_mm = APP_Q2_DEFAULT_LENGTH_MM;
  if (id == PROBLEM_Q3) params.line_length_mm = APP_Q3_DEFAULT_LENGTH_MM;
  if (id == PROBLEM_Q5 || id == PROBLEM_Q6) params.radius_mm = 250U;
  return params;
}

ProblemRequestResult ProblemApi_ValidateStart(ProblemId id,
                                             const ProblemParams *params)
{
  const ProblemDescriptor *descriptor = ProblemApi_Describe(id);
  if (descriptor == NULL) return PROBLEM_REQUEST_INVALID_ID;
  if (!descriptor->implemented) return PROBLEM_REQUEST_NOT_IMPLEMENTED;
  const ProblemParams defaults = ProblemApi_DefaultParams(id);
  const uint32_t length = (params == NULL) ? defaults.line_length_mm : params->line_length_mm;
  if ((id == PROBLEM_Q1 && length != defaults.line_length_mm) ||
      (id == PROBLEM_Q2 &&
       (length < APP_Q2_MIN_LENGTH_MM || length > APP_Q2_MAX_LENGTH_MM)) ||
      (id == PROBLEM_Q3 &&
       (length < APP_Q3_MIN_LENGTH_MM || length > APP_Q3_MAX_LENGTH_MM)))
    return PROBLEM_REQUEST_INVALID_PARAMS;
  if (id == PROBLEM_Q3 &&
      (params != NULL && (params->direction_deg >= 360U ||
                          params->direction_deg % APP_Q3_DIRECTION_STEP_DEG != 0U)))
    return PROBLEM_REQUEST_INVALID_PARAMS;
  if ((id == PROBLEM_Q3 || id == PROBLEM_Q4) && APP_Q3_ROLL_SIGN == 0)
    return PROBLEM_REQUEST_MIXER_UNCALIBRATED;
  return PROBLEM_REQUEST_ACCEPTED;
}

uint32_t ProblemApi_StartCommand(ProblemId id)
{
  if (id == PROBLEM_Q1) return 0x5131474FU;
  if (id == PROBLEM_Q2) return 0x5132474FU;
  if (id == PROBLEM_Q3) return 0x5133474FU;
  if (id == PROBLEM_Q4) return 0x5134474FU;
  return 0U;
}

ProblemId ProblemApi_DecodeStartCommand(uint32_t command)
{
  if (command == ProblemApi_StartCommand(PROBLEM_Q1)) return PROBLEM_Q1;
  if (command == ProblemApi_StartCommand(PROBLEM_Q2)) return PROBLEM_Q2;
  if (command == ProblemApi_StartCommand(PROBLEM_Q3)) return PROBLEM_Q3;
  if (command == ProblemApi_StartCommand(PROBLEM_Q4)) return PROBLEM_Q4;
  return PROBLEM_NONE;
}
