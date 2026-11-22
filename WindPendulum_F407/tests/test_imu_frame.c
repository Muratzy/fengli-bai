#include "services/imu_frame.h"
#include "app_config.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

static const ImuFrame_Config config = {
    {APP_IMU_FRAME_RIGHT, APP_IMU_FRAME_UP, APP_IMU_FRAME_NORMAL}};

static void Near(float actual, float expected, float tolerance)
{
  assert(fabsf(actual - expected) < tolerance);
}

static void TestRotationAndValidity(void)
{
  float result[3];
  assert(ImuFrame_ConfigValid(&config));
  /* A sensor vector along any calibrated axis becomes a unit body axis. */
  for (unsigned axis = 0U; axis < 3U; ++axis)
  {
    assert(ImuFrame_Rotate(&config, config.axis[axis], result));
    for (unsigned other = 0U; other < 3U; ++other)
    {
      Near(result[other], (axis == other) ? 1.0F : 0.0F, 0.00001F);
    }
  }
  ImuFrame_Config invalid = config;
  for (unsigned col = 0U; col < 3U; ++col)
  {
    invalid.axis[0][col] *= -1.0F;
  }
  assert(!ImuFrame_ConfigValid(&invalid)); /* Reject a reflected gyro frame. */
  invalid = config;
  invalid.axis[0][0] = NAN;
  assert(!ImuFrame_ConfigValid(&invalid));
  assert(!ImuFrame_ConfigValid(NULL));
  const float not_finite[3] = {0.0F, NAN, 1.0F};
  assert(!ImuFrame_Rotate(&config, not_finite, result));
  assert(!ImuFrame_Rotate(NULL, config.axis[0], result));
}

static void TestPolar(void)
{
  float direction;
  float tilt;
  const float poses[4][3] = {{20.0F, 0.0F, 0.0F}, {0.0F, 20.0F, 90.0F},
                            {-20.0F, 0.0F, 180.0F}, {0.0F, -20.0F, 270.0F}};
  for (unsigned pose = 0U; pose < 4U; ++pose)
  {
    assert(ImuFrame_Polar(poses[pose][0], poses[pose][1], &direction, &tilt));
    Near(direction, poses[pose][2], 0.0001F);
    Near(tilt, 20.0F, 0.0001F);
  }
  assert(!ImuFrame_Polar(0.0F, 0.0F, &direction, &tilt));
  Near(tilt, 0.0F, 0.0001F);
  assert(!ImuFrame_Polar(0.99F, 0.0F, &direction, &tilt));
  assert(ImuFrame_Polar(1.01F, 0.0F, &direction, &tilt));
  assert(ImuFrame_Polar(20.0F, -20.0F, &direction, &tilt));
  assert(direction > 300.0F && direction < 330.0F);
  assert(!ImuFrame_Polar(NAN, 0.0F, &direction, &tilt));
  assert(!ImuFrame_Polar(0.0F, 0.0F, NULL, &tilt));
}

static void TestMeasuredPoses(void)
{
  /* Mean sensor accelerations from the five saved manual poses, in g.
   * Hand placement only establishes the frame, with about 5 degrees scatter. */
  const float sensor[5][3] = {
      {0.01138244F, -0.06413778F, -0.98754760F},
      {-0.10903750F, -0.17724310F, -0.96773300F},
      {0.15257984F, -0.21227297F, -0.95438245F},
      {0.14741332F, 0.04943350F, -0.97742294F},
      {-0.10229394F, 0.09533618F, -0.97995650F}};
  for (unsigned pose = 0U; pose < 5U; ++pose)
  {
    float body[3];
    float direction;
    float tilt;
    assert(ImuFrame_Rotate(&config, sensor[pose], body));
    const float up = atan2f(body[1], body[2]) * 57.295779513F;
    const float right = atan2f(body[0], sqrtf(body[1] * body[1] + body[2] * body[2])) * 57.295779513F;
    const bool valid = ImuFrame_Polar(up, right, &direction, &tilt);
    if (pose == 0U)
    {
      assert(!valid && tilt < 0.01F);
    }
    else
    {
      const float expected = (float)(pose - 1U) * 90.0F;
      const float error = remainderf(direction - expected, 360.0F);
      assert(valid && fabsf(error) < 5.1F);
      assert(tilt > 9.0F && tilt < 13.0F);
    }
  }
}

int main(void)
{
  TestRotationAndValidity();
  TestPolar();
  TestMeasuredPoses();
  puts("IMU frame tests passed");
  return 0;
}
