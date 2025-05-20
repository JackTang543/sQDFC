/*
 * File: ekf_AltEst6.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-05-04 12:04:24
 */

/* Include Files */
#include "ekf_AltEst6.h"
#include "dot.h"
#include "ekf_AltEst6_data.h"
#include "ekf_AltEst6_initialize.h"
#include "mldivide.h"
#include "mrdivide_helper.h"
#include "norm.h"
#include <math.h>

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  float x_est[4];
  float P[16];
  float Q[16];
  float R[9];
  float M[16];
  float lambda;
  float R_eff[9];
  float state[5];
} struct_T;
#endif /* typedef_struct_T */

/* Variable Definitions */
static bool ekf_not_empty;

/* Function Definitions */
/*
 * by Sightseer inHNIP9607 250504 v1.2
 *
 * Arguments    : const float gyr[3]
 *                const float acc[3]
 *                float acc_notrust_factor
 *                float dt
 *                float eul[3]
 *                float quat[4]
 *                float state[5]
 * Return Type  : void
 */
void ekf_AltEst6(const float gyr[3], const float acc[3],
                 float acc_notrust_factor, float dt, float eul[3],
                 float quat[4], float state[5])
{
  static struct_T ekf;
  static const float fv[16] = {0.01F, 0.0F, 0.0F, 0.0F, 0.0F,  0.01F,
                               0.0F,  0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
                               0.0F,  0.0F, 0.0F, 0.01F};
  static const float a[9] = {100000.0F, 0.0F, 0.0F, 0.0F,     100000.0F,
                             0.0F,      0.0F, 0.0F, 100000.0F};
  static const short iv1[9] = {1000, 0, 0, 0, 1000, 0, 0, 0, 1000};
  static const signed char iv[16] = {30, 0, 0,  0, 0, 30, 0, 0,
                                     0,  0, 30, 0, 0, 0,  0, 30};
  static const signed char iv2[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                      0, 0, 1, 0, 0, 0, 0, 0};
  static const signed char iv4[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                      0, 0, 1, 0, 0, 0, 0, 1};
  static const signed char iv3[9] = {100, 0, 0, 0, 100, 0, 0, 0, 100};
  float F[16];
  float P_pred[16];
  float b_F[16];
  float b_gamma[16];
  float c_gamma[16];
  float H[12];
  float K[12];
  float b_H[12];
  float b_nis_tmp[9];
  float nis_tmp[9];
  float k2[4];
  float k3[4];
  float q_pred[4];
  float quat_init[4];
  float k[3];
  float v_b[3];
  float F_tmp;
  float acc_norm;
  float b_F_tmp;
  float f;
  float f1;
  float f2;
  float f3;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  float y;
  int gamma_tmp;
  int i;
  int i1;
  int i2;
  if (!isInitialized_ekf_AltEst6) {
    ekf_AltEst6_initialize();
  }
  /*  Parameter notate:(NED) */
  /*  gyr: [gyr_x,gyr_y,gyr_z] */
  /*  acc: [acc_x,acc_y,acc_z] */
  /*  acc_notrust_factor:0~1 manual select trust level */
  /*  0:more trust accelerator */
  /*  1:almost not trust accelerator */
  /*  if more than 1.001,invalid data(automatic trust:nis and acc norm) */
  /*  dt:delta t,unit:second */
  /*  Return state */
  /*  1: trace(R) / 3 */
  /*  2: trace(P) / 4 */
  /*  3: chi-square * 10000 */
  /*  4: trace(err) / 3 * 100; */
  /*  5: abs(acc_norm - 9.81) */
  if (!ekf_not_empty) {
    /*  x estimation vector(q0 q1 q2 q3 hamiltionian quaternion) */
    ekf.x_est[0] = 1.0F;
    ekf.x_est[1] = 0.0F;
    ekf.x_est[2] = 0.0F;
    ekf.x_est[3] = 0.0F;
    ekf_not_empty = true;
    /*  predict covariance matrix */
    /*  process noise */
    for (i = 0; i < 16; i++) {
      ekf.P[i] = iv[i];
      ekf.Q[i] = fv[i];
    }
    /*  measurement noise */
    for (i = 0; i < 9; i++) {
      ekf.R[i] = iv1[i];
    }
    /*  masking accelerator yaw correction, caution: inaccurate under large
     * attitude changes! */
    for (i = 0; i < 16; i++) {
      ekf.M[i] = iv2[i];
    }
    /*  Initialization for adaptive R scaling */
    /*  Scaling factor */
    ekf.lambda = 2.0F;
    /*  Max value for lambda */
    /*  Scaling rate when exceeding threshold */
    /*  Scaling rate when below threshold */
    /*  Threshold for chi-square statistic */
    /*  Acceleration deviation to detect overload */
    /*  used for measurement correction */
    /*  First run,set direction is accelerate */
    /*  Vector alignment method to initialize quaternion */
    acc_norm = b_norm(acc);
    /*  Prevent division by zero */
    if (acc_norm > 0.001F) {
      /*  Unitization gravity vector(body axis) */
      v_b[0] = acc[0] / acc_norm;
      v_b[1] = acc[1] / acc_norm;
      v_b[2] = acc[2] / acc_norm;
      /*  expect gravity direction */
      /*  rotation axis */
      k[0] = v_b[1];
      k[1] = 0.0F - v_b[0];
      k[2] = 0.0F;
      acc_norm = b_norm(k);
      /*  almost flat plane */
      if (acc_norm < 0.001F) {
        quat_init[0] = 1.0F;
        quat_init[1] = 0.0F;
        quat_init[2] = 0.0F;
        quat_init[3] = 0.0F;
      } else {
        /*  unit axis */
        /*  rotation angular */
        gyr_x = acosf(dot(v_b)) / 2.0F;
        gyr_z = sinf(gyr_x);
        quat_init[0] = cosf(gyr_x);
        quat_init[1] = v_b[1] / acc_norm * gyr_z;
        quat_init[2] = (0.0F - v_b[0]) / acc_norm * gyr_z;
        quat_init[3] = 0.0F / acc_norm * gyr_z;
      }
      ekf.x_est[0] = quat_init[0];
      ekf.x_est[1] = quat_init[1];
      ekf.x_est[2] = quat_init[2];
      ekf.x_est[3] = quat_init[3];
    }
  }
  /*  get gyroscope data,from deg/s to rad/s */
  gyr_x = 0.0174532924F * gyr[0];
  gyr_y = 0.0174532924F * gyr[1];
  gyr_z = 0.0174532924F * gyr[2];
  /*  get accelerator data,m/s^2 */
  /*  -----------------prediction----------------- */
  /*  quaternion derivative equation：q_dot = 1/2 * omega * q */
  /*  calculate rk4 */
  F[0] = 0.0F;
  F[4] = -gyr_x;
  F[8] = -gyr_y;
  F[12] = -gyr_z;
  F[1] = gyr_x;
  F[5] = 0.0F;
  F[9] = gyr_z;
  F[13] = -gyr_y;
  F[2] = gyr_y;
  F[6] = -gyr_z;
  F[10] = 0.0F;
  F[14] = gyr_x;
  F[3] = gyr_z;
  F[7] = gyr_y;
  F[11] = -gyr_x;
  F[15] = 0.0F;
  acc_norm = 0.5F * dt;
  for (i = 0; i < 4; i++) {
    f = ((0.5F * F[i] * ekf.x_est[0] + 0.5F * F[i + 4] * ekf.x_est[1]) +
         0.5F * F[i + 8] * ekf.x_est[2]) +
        0.5F * F[i + 12] * ekf.x_est[3];
    quat_init[i] = f;
    k2[i] = ekf.x_est[i] + acc_norm * f;
  }
  f = k2[0];
  f1 = k2[1];
  f2 = k2[2];
  f3 = k2[3];
  for (i = 0; i < 4; i++) {
    k2[i] = ((0.5F * F[i] * f + 0.5F * F[i + 4] * f1) + 0.5F * F[i + 8] * f2) +
            0.5F * F[i + 12] * f3;
  }
  f = ekf.x_est[0] + acc_norm * k2[0];
  f1 = ekf.x_est[1] + acc_norm * k2[1];
  f2 = ekf.x_est[2] + acc_norm * k2[2];
  f3 = ekf.x_est[3] + acc_norm * k2[3];
  for (i = 0; i < 4; i++) {
    k3[i] = ((0.5F * F[i] * f + 0.5F * F[i + 4] * f1) + 0.5F * F[i + 8] * f2) +
            0.5F * F[i + 12] * f3;
  }
  f = ekf.x_est[0] + dt * k3[0];
  f1 = ekf.x_est[1] + dt * k3[1];
  f2 = ekf.x_est[2] + dt * k3[2];
  f3 = ekf.x_est[3] + dt * k3[3];
  for (i = 0; i < 4; i++) {
    q_pred[i] =
        ekf.x_est[i] + dt *
                           (((quat_init[i] + 2.0F * k2[i]) + 2.0F * k3[i]) +
                            (((0.5F * F[i] * f + 0.5F * F[i + 4] * f1) +
                              0.5F * F[i + 8] * f2) +
                             0.5F * F[i + 12] * f3)) /
                           6.0F;
  }
  /*  quaternion normalize */
  y = c_norm(q_pred);
  /*  -----------------calculate process model jacobian matrix-----------------
   */
  F_tmp = dt * gyr_x;
  b_F_tmp = dt * gyr_z;
  acc_norm = dt * gyr_y;
  gyr_x = -acc_norm / 2.0F;
  gyr_z = -b_F_tmp / 2.0F;
  gyr_y = F_tmp / 2.0F;
  b_F_tmp /= 2.0F;
  acc_norm /= 2.0F;
  F_tmp = -F_tmp / 2.0F;
  F[0] = 1.0F;
  F[4] = F_tmp;
  F[8] = gyr_x;
  F[12] = gyr_z;
  F[1] = gyr_y;
  F[5] = 1.0F;
  F[9] = b_F_tmp;
  F[13] = gyr_x;
  F[2] = acc_norm;
  F[6] = gyr_z;
  F[10] = 1.0F;
  F[14] = gyr_y;
  F[3] = b_F_tmp;
  F[7] = acc_norm;
  F[11] = F_tmp;
  F[15] = 1.0F;
  /*  noise transfer matrix */
  acc_norm = -ekf.x_est[3] / 2.0F;
  gyr_x = ekf.x_est[0] / 2.0F;
  gyr_z = -ekf.x_est[1] / 2.0F;
  gyr_y = -ekf.x_est[2] / 2.0F;
  b_gamma[0] = gyr_z;
  b_gamma[4] = gyr_y;
  b_gamma[8] = acc_norm;
  b_gamma[12] = 0.0F;
  b_gamma[1] = gyr_x;
  b_gamma[5] = acc_norm;
  b_gamma[9] = ekf.x_est[2] / 2.0F;
  b_gamma[13] = 0.0F;
  b_gamma[2] = ekf.x_est[3] / 2.0F;
  b_gamma[6] = gyr_x;
  b_gamma[10] = gyr_z;
  b_gamma[14] = 0.0F;
  b_gamma[3] = gyr_y;
  b_gamma[7] = ekf.x_est[1] / 2.0F;
  b_gamma[11] = gyr_x;
  b_gamma[15] = 0.0F;
  /*  predict covariance prediction */
  for (i = 0; i < 4; i++) {
    q_pred[i] /= y;
    f = b_gamma[i];
    f1 = b_gamma[i + 4];
    f2 = b_gamma[i + 8];
    f3 = b_gamma[i + 12];
    b_F_tmp = F[i];
    gyr_z = F[i + 4];
    F_tmp = F[i + 8];
    acc_norm = F[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      gamma_tmp = i + i2;
      c_gamma[gamma_tmp] =
          ((f * ekf.Q[i2] + f1 * ekf.Q[i2 + 1]) + f2 * ekf.Q[i2 + 2]) +
          f3 * ekf.Q[i2 + 3];
      b_F[gamma_tmp] = ((b_F_tmp * ekf.P[i2] + gyr_z * ekf.P[i2 + 1]) +
                        F_tmp * ekf.P[i2 + 2]) +
                       acc_norm * ekf.P[i2 + 3];
    }
    f = b_F[i];
    f1 = b_F[i + 4];
    f2 = b_F[i + 8];
    f3 = b_F[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      P_pred[i + (i1 << 2)] =
          ((f * F[i1] + f1 * F[i1 + 4]) + f2 * F[i1 + 8]) + f3 * F[i1 + 12];
    }
  }
  for (i = 0; i < 4; i++) {
    f = c_gamma[i];
    f1 = c_gamma[i + 4];
    f2 = c_gamma[i + 8];
    f3 = c_gamma[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      F[i + (i1 << 2)] =
          ((f * b_gamma[i1] + f1 * b_gamma[i1 + 4]) + f2 * b_gamma[i1 + 8]) +
          f3 * b_gamma[i1 + 12];
    }
  }
  for (i = 0; i < 16; i++) {
    P_pred[i] += F[i];
  }
  /*  -----------------calculate update model----------------- */
  /*  normalize accelerometer */
  y = b_norm(acc);
  /*  observation vector a = [0;0;g] */
  /*  error from accelerator */
  v_b[0] = 2.0F * (ekf.x_est[1] * ekf.x_est[3] - ekf.x_est[0] * ekf.x_est[2]);
  v_b[1] = 2.0F * (ekf.x_est[2] * ekf.x_est[3] + ekf.x_est[0] * ekf.x_est[1]);
  v_b[2] =
      1.0F - 2.0F * (ekf.x_est[1] * ekf.x_est[1] + ekf.x_est[2] * ekf.x_est[2]);
  /*  observation jacobian matrix */
  acc_norm = 2.0F * ekf.x_est[1];
  gyr_x = 2.0F * ekf.x_est[3];
  gyr_z = 2.0F * ekf.x_est[0];
  gyr_y = -2.0F * ekf.x_est[2];
  H[0] = gyr_y;
  H[3] = gyr_x;
  H[6] = -2.0F * ekf.x_est[0];
  H[9] = acc_norm;
  H[1] = acc_norm;
  H[4] = gyr_z;
  H[7] = gyr_x;
  H[10] = 2.0F * ekf.x_est[2];
  H[2] = gyr_z;
  H[5] = -2.0F * ekf.x_est[1];
  H[8] = gyr_y;
  H[11] = gyr_x;
  /*  -----------------adaptive R scaling based on NIS----------------- */
  /*  NIS (chi-square statistic) */
  /*  r_k = err' * inv(H * P_pred * H' + ekf.R) * err; */
  /*  current innovation covariance */
  /*  S = H * P_pred * H' + ekf.R; */
  /*  nis = err' * (S \ err); */
  for (i = 0; i < 3; i++) {
    v_b[i] = acc[i] / y - v_b[i];
    for (i1 = 0; i1 < 4; i1++) {
      gamma_tmp = i + 3 * i1;
      K[i1 + (i << 2)] = H[gamma_tmp];
      i2 = i1 << 2;
      b_H[gamma_tmp] = ((H[i] * P_pred[i2] + H[i + 3] * P_pred[i2 + 1]) +
                        H[i + 6] * P_pred[i2 + 2]) +
                       H[i + 9] * P_pred[i2 + 3];
    }
  }
  for (i = 0; i < 3; i++) {
    f = b_H[i];
    f1 = b_H[i + 3];
    f2 = b_H[i + 6];
    f3 = b_H[i + 9];
    for (i1 = 0; i1 < 3; i1++) {
      i2 = i1 << 2;
      nis_tmp[i + 3 * i1] =
          ((f * K[i2] + f1 * K[i2 + 1]) + f2 * K[i2 + 2]) + f3 * K[i2 + 3];
    }
  }
  for (i = 0; i < 9; i++) {
    b_nis_tmp[i] = nis_tmp[i] + ekf.R[i];
  }
  mldivide(b_nis_tmp, v_b, k);
  gyr_y = (v_b[0] * k[0] + v_b[1] * k[1]) + v_b[2] * k[2];
  /*  Overload detection based on acceleration norm deviation from 9.81 m/s^2 */
  gyr_x = fabsf(sqrtf((acc[0] * acc[0] + acc[1] * acc[1]) + acc[2] * acc[2]) -
                9.81F);
  if ((gyr_x > 2.0F) || (gyr_y > 2.0E-5F)) {
    ekf.lambda =
        fminf(ekf.lambda * (0.1F * (gyr_y / 2.0E-5F - 1.0F) + 1.0F), 500.0F);
  } else if (ekf.lambda > 1.0F) {
    ekf.lambda *= 0.95F;
  }
  /*  Ensure that R_eff does not go below a certain threshold */
  /*  Minimum value for R_eff */
  /*  Update effective R */
  for (gamma_tmp = 0; gamma_tmp < 9; gamma_tmp++) {
    ekf.R_eff[gamma_tmp] = fmaxf(ekf.lambda * ekf.R[gamma_tmp], iv3[gamma_tmp]);
  }
  /*  weighted measurement level */
  if ((acc_notrust_factor < 1.001F) && (acc_notrust_factor > 0.0F)) {
    /* ekf.R_eff = diag([100000 * acc_notrust_factor;100000 *
     * acc_notrust_factor;100000 * acc_notrust_factor]); */
    for (gamma_tmp = 0; gamma_tmp < 9; gamma_tmp++) {
      ekf.R_eff[gamma_tmp] = a[gamma_tmp] * acc_notrust_factor;
      ekf.R_eff[gamma_tmp] = fmaxf(ekf.R_eff[gamma_tmp], iv3[gamma_tmp]);
    }
  }
  /*  -----------------measurement update----------------- */
  for (i = 0; i < 4; i++) {
    f = P_pred[i];
    f1 = P_pred[i + 4];
    f2 = P_pred[i + 8];
    f3 = P_pred[i + 12];
    for (i1 = 0; i1 < 3; i1++) {
      i2 = i1 << 2;
      b_H[i + i2] =
          ((f * K[i2] + f1 * K[i2 + 1]) + f2 * K[i2 + 2]) + f3 * K[i2 + 3];
    }
  }
  for (i = 0; i < 9; i++) {
    nis_tmp[i] += ekf.R_eff[i];
  }
  mrdiv(b_H, nis_tmp, K);
  for (i = 0; i < 4; i++) {
    f = 0.0F;
    f1 = ekf.M[i];
    f2 = ekf.M[i + 4];
    f3 = ekf.M[i + 8];
    b_F_tmp = ekf.M[i + 12];
    for (i1 = 0; i1 < 3; i1++) {
      i2 = i1 << 2;
      gyr_z = ((f1 * K[i2] + f2 * K[i2 + 1]) + f3 * K[i2 + 2]) +
              b_F_tmp * K[i2 + 3];
      b_H[i + i2] = gyr_z;
      f += gyr_z * v_b[i1];
    }
    ekf.x_est[i] = q_pred[i] + f;
    f = K[i];
    f1 = K[i + 4];
    f2 = K[i + 8];
    for (i1 = 0; i1 < 4; i1++) {
      gamma_tmp = i + (i1 << 2);
      F[gamma_tmp] =
          (float)iv4[gamma_tmp] -
          ((f * H[3 * i1] + f1 * H[3 * i1 + 1]) + f2 * H[3 * i1 + 2]);
    }
    f3 = F[i];
    b_F_tmp = F[i + 4];
    gyr_z = F[i + 8];
    F_tmp = F[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      i2 = i1 << 2;
      b_F[i + i2] = ((f3 * P_pred[i2] + b_F_tmp * P_pred[i2 + 1]) +
                     gyr_z * P_pred[i2 + 2]) +
                    F_tmp * P_pred[i2 + 3];
    }
    for (i1 = 0; i1 < 3; i1++) {
      b_H[i + (i1 << 2)] =
          (f * ekf.R_eff[3 * i1] + f1 * ekf.R_eff[3 * i1 + 1]) +
          f2 * ekf.R_eff[3 * i1 + 2];
    }
  }
  for (i = 0; i < 4; i++) {
    f = b_F[i];
    f1 = b_F[i + 4];
    f2 = b_F[i + 8];
    f3 = b_F[i + 12];
    for (i1 = 0; i1 < 4; i1++) {
      b_gamma[i + (i1 << 2)] =
          ((f * F[i1] + f1 * F[i1 + 4]) + f2 * F[i1 + 8]) + f3 * F[i1 + 12];
    }
  }
  for (i = 0; i < 4; i++) {
    f = b_H[i];
    f1 = b_H[i + 4];
    f2 = b_H[i + 8];
    for (i1 = 0; i1 < 4; i1++) {
      F[i + (i1 << 2)] = (f * K[i1] + f1 * K[i1 + 4]) + f2 * K[i1 + 8];
    }
  }
  for (i = 0; i < 16; i++) {
    ekf.P[i] = b_gamma[i] + F[i];
  }
  /*  -----------------output----------------- */
  acc_norm = ekf.x_est[2] * ekf.x_est[2];
  eul[0] =
      atan2f(2.0F * (ekf.x_est[0] * ekf.x_est[1] + ekf.x_est[2] * ekf.x_est[3]),
             1.0F - 2.0F * (ekf.x_est[1] * ekf.x_est[1] + acc_norm));
  eul[1] =
      asinf(2.0F * (ekf.x_est[0] * ekf.x_est[2] - ekf.x_est[1] * ekf.x_est[3]));
  eul[2] =
      atan2f(2.0F * (ekf.x_est[0] * ekf.x_est[3] + ekf.x_est[1] * ekf.x_est[2]),
             1.0F - 2.0F * (acc_norm + ekf.x_est[3] * ekf.x_est[3]));
  eul[0] *= 57.2957802F;
  eul[1] *= 57.2957802F;
  eul[2] *= 57.2957802F;
  quat[0] = ekf.x_est[0];
  quat[1] = ekf.x_est[1];
  quat[2] = ekf.x_est[2];
  quat[3] = ekf.x_est[3];
  ekf.state[0] = ((ekf.R_eff[0] + ekf.R_eff[4]) + ekf.R_eff[8]) / 3.0F;
  ekf.state[1] = (((ekf.P[0] + ekf.P[5]) + ekf.P[10]) + ekf.P[15]) / 4.0F;
  ekf.state[2] = gyr_y * 10000.0F;
  ekf.state[3] = ((v_b[0] + v_b[1]) + v_b[2]) / 3.0F * 100.0F;
  ekf.state[4] = gyr_x;
  for (i = 0; i < 5; i++) {
    state[i] = ekf.state[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ekf_AltEst6_init(void)
{
  ekf_not_empty = false;
}

/*
 * File trailer for ekf_AltEst6.c
 *
 * [EOF]
 */
