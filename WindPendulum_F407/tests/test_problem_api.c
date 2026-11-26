#include "problems/problem_api.h"
#include "app_config.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

int main(void)
{
  for (uint32_t number = 1U; number <= 7U; ++number) {
    const ProblemId id = (ProblemId)number;
    const ProblemDescriptor *descriptor = ProblemApi_Describe(id);
    assert(descriptor != NULL && descriptor->id == id && descriptor->name != NULL);
    assert(descriptor->implemented == (number <= 4U));
    if (id == PROBLEM_Q3 || id == PROBLEM_Q4) {
      assert(ProblemApi_ValidateStart(id, NULL) == (APP_Q3_ROLL_SIGN == 0 ?
          PROBLEM_REQUEST_MIXER_UNCALIBRATED : PROBLEM_REQUEST_ACCEPTED));
    } else {
      assert(ProblemApi_ValidateStart(id, NULL) == ((number <= 2U) ?
          PROBLEM_REQUEST_ACCEPTED : PROBLEM_REQUEST_NOT_IMPLEMENTED));
    }
    const uint32_t command = ProblemApi_StartCommand(id);
    assert(ProblemApi_DecodeStartCommand(command) == (number <= 4U ? id : PROBLEM_NONE));
    if (number > 4U) assert(command == 0U);
  }
  assert(ProblemApi_Describe(PROBLEM_NONE) == NULL);
  assert(ProblemApi_Describe((ProblemId)8) == NULL);
  assert(ProblemApi_ValidateStart(PROBLEM_NONE, NULL) == PROBLEM_REQUEST_INVALID_ID);
  ProblemParams params = ProblemApi_DefaultParams(PROBLEM_Q2);
  assert(params.line_length_mm == APP_Q2_DEFAULT_LENGTH_MM);
  params.line_length_mm = 300U;
  assert(ProblemApi_ValidateStart(PROBLEM_Q2, &params) == PROBLEM_REQUEST_ACCEPTED);
  params.line_length_mm = 601U;
  assert(ProblemApi_ValidateStart(PROBLEM_Q2, &params) == PROBLEM_REQUEST_INVALID_PARAMS);
  assert(ProblemApi_ValidateStart(PROBLEM_Q1, &params) == PROBLEM_REQUEST_INVALID_PARAMS);
  params = ProblemApi_DefaultParams(PROBLEM_Q1);
  assert(params.line_length_mm == 600U);
  assert(ProblemApi_ValidateStart(PROBLEM_Q1, &params) == PROBLEM_REQUEST_ACCEPTED);
  assert(ProblemApi_StartCommand(PROBLEM_Q1) == 0x5131474FU);
  assert(ProblemApi_StartCommand(PROBLEM_Q2) == 0x5132474FU);
  assert(ProblemApi_StartCommand(PROBLEM_Q3) == 0x5133474FU);
  params = ProblemApi_DefaultParams(PROBLEM_Q3);
  assert(params.direction_deg == 0U && params.line_length_mm == APP_Q3_DEFAULT_LENGTH_MM);
  params.direction_deg = 15U;
  assert(ProblemApi_ValidateStart(PROBLEM_Q3, &params) == PROBLEM_REQUEST_INVALID_PARAMS);
  params.direction_deg = 360U;
  assert(ProblemApi_ValidateStart(PROBLEM_Q3, &params) == PROBLEM_REQUEST_INVALID_PARAMS);
  params.direction_deg = 350U;
  params.line_length_mm = APP_Q3_MIN_LENGTH_MM - 1U;
  assert(ProblemApi_ValidateStart(PROBLEM_Q3, &params) == PROBLEM_REQUEST_INVALID_PARAMS);
  params.line_length_mm = APP_Q3_MAX_LENGTH_MM + 1U;
  assert(ProblemApi_ValidateStart(PROBLEM_Q3, &params) == PROBLEM_REQUEST_INVALID_PARAMS);
  puts("Problem API tests passed");
  return 0;
}
