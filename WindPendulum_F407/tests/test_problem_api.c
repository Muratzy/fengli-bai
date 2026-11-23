#include "problems/problem_api.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

int main(void)
{
  for (uint32_t number = 1U; number <= 7U; ++number) {
    const ProblemId id = (ProblemId)number;
    const ProblemDescriptor *descriptor = ProblemApi_Describe(id);
    assert(descriptor != NULL && descriptor->id == id && descriptor->name != NULL);
    assert(descriptor->implemented == (number <= 2U));
    assert(ProblemApi_ValidateStart(id, NULL) == ((number <= 2U) ?
        PROBLEM_REQUEST_ACCEPTED : PROBLEM_REQUEST_NOT_IMPLEMENTED));
    if (number > 2U) assert(ProblemApi_StartCommand(id) == 0U);
  }
  assert(ProblemApi_Describe(PROBLEM_NONE) == NULL);
  assert(ProblemApi_Describe((ProblemId)8) == NULL);
  assert(ProblemApi_ValidateStart(PROBLEM_NONE, NULL) == PROBLEM_REQUEST_INVALID_ID);
  ProblemParams params = ProblemApi_DefaultParams(PROBLEM_Q2);
  assert(params.line_length_mm == 600U);
  params.line_length_mm = 300U;
  assert(ProblemApi_ValidateStart(PROBLEM_Q2, &params) == PROBLEM_REQUEST_ACCEPTED);
  params.line_length_mm = 601U;
  assert(ProblemApi_ValidateStart(PROBLEM_Q2, &params) == PROBLEM_REQUEST_INVALID_PARAMS);
  assert(ProblemApi_ValidateStart(PROBLEM_Q1, &params) == PROBLEM_REQUEST_INVALID_PARAMS);
  params = ProblemApi_DefaultParams(PROBLEM_Q1);
  assert(params.line_length_mm == 500U);
  assert(ProblemApi_ValidateStart(PROBLEM_Q1, &params) == PROBLEM_REQUEST_ACCEPTED);
  assert(ProblemApi_StartCommand(PROBLEM_Q1) == 0x5131474FU);
  assert(ProblemApi_StartCommand(PROBLEM_Q2) == 0x5132474FU);
  puts("Problem API tests passed");
  return 0;
}
