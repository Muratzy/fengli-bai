#include "services/imu_frame.h"

#include <math.h>
#include <stddef.h>

#define IMU_FRAME_RAD_PER_DEG 0.01745329251994329577F
#define IMU_FRAME_DEG_PER_RAD 57.29577951308232F

bool ImuFrame_ConfigValid(const ImuFrame_Config *config)
{
  if (config == NULL)
  {
    return false;
  }
  for (unsigned row = 0U; row < 3U; ++row)
  {
    for (unsigned other = row; other < 3U; ++other)
    {
      float dot = 0.0F;
      for (unsigned col = 0U; col < 3U; ++col)
      {
        dot += config->axis[row][col] * config->axis[other][col];
      }
      if (!isfinite(dot) || fabsf(dot - ((row == other) ? 1.0F : 0.0F)) > 0.002F)
      {
        return false;
      }
    }
  }
  const float (*a)[3] = config->axis;
  const float determinant = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) +
      a[0][1] * (a[1][2] * a[2][0] - a[1][0] * a[2][2]) +
      a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
  return isfinite(determinant) && determinant > 0.998F;
}

bool ImuFrame_Rotate(const ImuFrame_Config *config, const float sensor[3],
                      float pendulum[3])
{
  if ((config == NULL) || (sensor == NULL) || (pendulum == NULL))
  {
    return false;
  }
  float result[3] = {0.0F, 0.0F, 0.0F};
  for (unsigned row = 0U; row < 3U; ++row)
  {
    for (unsigned col = 0U; col < 3U; ++col)
    {
      result[row] += config->axis[row][col] * sensor[col];
    }
    if (!isfinite(result[row]))
    {
      return false;
    }
  }
  for (unsigned row = 0U; row < 3U; ++row)
  {
    pendulum[row] = result[row];
  }
  return true;
}

bool ImuFrame_Polar(float up_roll_deg, float right_pitch_deg,
                     float *direction_deg, float *tilt_deg)
{
  if ((direction_deg == NULL) || (tilt_deg == NULL))
  {
    return false;
  }
  *direction_deg = 0.0F;
  *tilt_deg = 0.0F;
  if (!isfinite(up_roll_deg) || !isfinite(right_pitch_deg))
  {
    return false;
  }
  const float roll = up_roll_deg * IMU_FRAME_RAD_PER_DEG;
  const float pitch = right_pitch_deg * IMU_FRAME_RAD_PER_DEG;
  const float right = sinf(pitch);
  const float up = cosf(pitch) * sinf(roll);
  const float normal = cosf(pitch) * cosf(roll);
  const float planar = sqrtf(right * right + up * up);
  *tilt_deg = atan2f(planar, normal) * IMU_FRAME_DEG_PER_RAD;
  if (*tilt_deg < 1.0F)
  {
    return false;
  }
  float direction = atan2f(right, up) * IMU_FRAME_DEG_PER_RAD;
  if (direction < 0.0F)
  {
    direction += 360.0F;
  }
  *direction_deg = (direction >= 360.0F) ? 0.0F : direction;
  return true;
}
