#ifndef IMU_FRAME_H
#define IMU_FRAME_H

#include <stdbool.h>

typedef struct {
  /* Rows: right, up, center gravity; an orthonormal right-handed frame. */
  float axis[3][3];
} ImuFrame_Config;

bool ImuFrame_ConfigValid(const ImuFrame_Config *config);
bool ImuFrame_Rotate(const ImuFrame_Config *config, const float sensor[3],
                      float pendulum[3]);
/* Roll is positive toward up, pitch is positive toward right.
 * Return false below 1 degree tilt, where planar direction is ambiguous.
 * Direction is clockwise: up=0, right=90, down=180, left=270. */
bool ImuFrame_Polar(float up_roll_deg, float right_pitch_deg,
                     float *direction_deg, float *tilt_deg);

#endif
