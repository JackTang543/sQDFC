/*
 * File: rk4_AltEst3.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-05-04 15:49:56
 */

/* Include Files */
#include "rk4_AltEst3.h"
#include "norm.h"
#include "rk4_AltEst3_data.h"
#include "rk4_AltEst3_initialize.h"
#include <math.h>

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  float quat[4];
} struct_T;
#endif /* typedef_struct_T */

/* Variable Definitions */
static struct_T rk4;

/* Function Definitions */
/*
 * rk4 calculate quaternion derivative equation(3-axis) pure integral
 *
 * Arguments    : const float gyr[3]
 *                float dt
 *                float eul[3]
 *                float quat[4]
 * Return Type  : void
 */
void rk4_AltEst3(const float gyr[3], float dt, float eul[3], float quat[4])
{
  float workspace_tmp[16];
  float k1[4];
  float k2[4];
  float k3[4];
  float f;
  float f1;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  int i;
  if (!isInitialized_rk4_AltEst3) {
    rk4_AltEst3_initialize();
  }
  /*  get gyroscope data,from deg/s to rad/s */
  gyr_x = 0.0174532924F * gyr[0];
  gyr_y = 0.0174532924F * gyr[1];
  gyr_z = 0.0174532924F * gyr[2];
  /*  quaternion derivative equation：q_dot = 1/2 * omega * q */
  /*  calculate rk4 */
  workspace_tmp[0] = 0.0F;
  workspace_tmp[4] = -gyr_x;
  workspace_tmp[8] = -gyr_y;
  workspace_tmp[12] = -gyr_z;
  workspace_tmp[1] = gyr_x;
  workspace_tmp[5] = 0.0F;
  workspace_tmp[9] = gyr_z;
  workspace_tmp[13] = -gyr_y;
  workspace_tmp[2] = gyr_y;
  workspace_tmp[6] = -gyr_z;
  workspace_tmp[10] = 0.0F;
  workspace_tmp[14] = gyr_x;
  workspace_tmp[3] = gyr_z;
  workspace_tmp[7] = gyr_y;
  workspace_tmp[11] = -gyr_x;
  workspace_tmp[15] = 0.0F;
  gyr_x = 0.5F * dt;
  for (i = 0; i < 4; i++) {
    f = ((0.5F * workspace_tmp[i] * rk4.quat[0] +
          0.5F * workspace_tmp[i + 4] * rk4.quat[1]) +
         0.5F * workspace_tmp[i + 8] * rk4.quat[2]) +
        0.5F * workspace_tmp[i + 12] * rk4.quat[3];
    k1[i] = f;
    k2[i] = rk4.quat[i] + gyr_x * f;
  }
  f = k2[0];
  gyr_y = k2[1];
  gyr_z = k2[2];
  f1 = k2[3];
  for (i = 0; i < 4; i++) {
    k2[i] =
        ((0.5F * workspace_tmp[i] * f + 0.5F * workspace_tmp[i + 4] * gyr_y) +
         0.5F * workspace_tmp[i + 8] * gyr_z) +
        0.5F * workspace_tmp[i + 12] * f1;
  }
  f = rk4.quat[0] + gyr_x * k2[0];
  gyr_y = rk4.quat[1] + gyr_x * k2[1];
  gyr_z = rk4.quat[2] + gyr_x * k2[2];
  f1 = rk4.quat[3] + gyr_x * k2[3];
  for (i = 0; i < 4; i++) {
    k3[i] =
        ((0.5F * workspace_tmp[i] * f + 0.5F * workspace_tmp[i + 4] * gyr_y) +
         0.5F * workspace_tmp[i + 8] * gyr_z) +
        0.5F * workspace_tmp[i + 12] * f1;
  }
  f = rk4.quat[0] + dt * k3[0];
  gyr_y = rk4.quat[1] + dt * k3[1];
  gyr_z = rk4.quat[2] + dt * k3[2];
  f1 = rk4.quat[3] + dt * k3[3];
  for (i = 0; i < 4; i++) {
    quat[i] = rk4.quat[i] + dt *
                                (((k1[i] + 2.0F * k2[i]) + 2.0F * k3[i]) +
                                 (((0.5F * workspace_tmp[i] * f +
                                    0.5F * workspace_tmp[i + 4] * gyr_y) +
                                   0.5F * workspace_tmp[i + 8] * gyr_z) +
                                  0.5F * workspace_tmp[i + 12] * f1)) /
                                6.0F;
  }
  /*  quaternion normalize */
  gyr_x = b_norm(quat);
  f = quat[0] / gyr_x;
  quat[0] = f;
  rk4.quat[0] = f;
  f = quat[1] / gyr_x;
  quat[1] = f;
  rk4.quat[1] = f;
  f = quat[2] / gyr_x;
  quat[2] = f;
  rk4.quat[2] = f;
  f = quat[3] / gyr_x;
  quat[3] = f;
  rk4.quat[3] = f;
  /*  output */
  gyr_x = quat[2] * quat[2];
  eul[0] = atan2f(2.0F * (quat[0] * quat[1] + quat[2] * f),
                  1.0F - 2.0F * (quat[1] * quat[1] + gyr_x));
  eul[1] = asinf(2.0F * (quat[0] * quat[2] - quat[1] * f));
  eul[2] = atan2f(2.0F * (quat[0] * f + quat[1] * quat[2]),
                  1.0F - 2.0F * (gyr_x + f * f));
  eul[0] *= 57.2957802F;
  eul[1] *= 57.2957802F;
  eul[2] *= 57.2957802F;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void rk4_AltEst3_init(void)
{
  rk4.quat[0] = 1.0F;
  rk4.quat[1] = 0.0F;
  rk4.quat[2] = 0.0F;
  rk4.quat[3] = 0.0F;
}

/*
 * File trailer for rk4_AltEst3.c
 *
 * [EOF]
 */
