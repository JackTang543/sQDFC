/*
 * File: mrdivide_helper.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-05-04 12:04:24
 */

/* Include Files */
#include "mrdivide_helper.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const float A[12]
 *                const float B[9]
 *                float Y[12]
 * Return Type  : void
 */
void mrdiv(const float A[12], const float B[9], float Y[12])
{
  float b_A[9];
  float a21;
  float c_Y_tmp;
  float d_Y_tmp;
  float e_Y_tmp;
  float f_Y_tmp;
  float maxval;
  int Y_tmp;
  int b_Y_tmp;
  int r1;
  int r2;
  int r3;
  int rtemp;
  for (rtemp = 0; rtemp < 9; rtemp++) {
    b_A[rtemp] = B[rtemp];
  }
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabsf(B[0]);
  a21 = fabsf(B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabsf(B[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = B[r2] / B[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (fabsf(b_A[r3 + 3]) > fabsf(b_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  rtemp = r1 << 2;
  Y[rtemp] = A[0] / b_A[r1];
  Y_tmp = r2 << 2;
  maxval = b_A[r1 + 3];
  Y[Y_tmp] = A[4] - Y[rtemp] * maxval;
  b_Y_tmp = r3 << 2;
  a21 = b_A[r1 + 6];
  Y[b_Y_tmp] = A[8] - Y[rtemp] * a21;
  c_Y_tmp = b_A[r2 + 3];
  Y[Y_tmp] /= c_Y_tmp;
  d_Y_tmp = b_A[r2 + 6];
  Y[b_Y_tmp] -= Y[Y_tmp] * d_Y_tmp;
  e_Y_tmp = b_A[r3 + 6];
  Y[b_Y_tmp] /= e_Y_tmp;
  f_Y_tmp = b_A[r3 + 3];
  Y[Y_tmp] -= Y[b_Y_tmp] * f_Y_tmp;
  Y[rtemp] -= Y[b_Y_tmp] * b_A[r3];
  Y[rtemp] -= Y[Y_tmp] * b_A[r2];
  Y[rtemp + 1] = A[1] / b_A[r1];
  Y[Y_tmp + 1] = A[5] - Y[rtemp + 1] * maxval;
  Y[b_Y_tmp + 1] = A[9] - Y[rtemp + 1] * a21;
  Y[Y_tmp + 1] /= c_Y_tmp;
  Y[b_Y_tmp + 1] -= Y[Y_tmp + 1] * d_Y_tmp;
  Y[b_Y_tmp + 1] /= e_Y_tmp;
  Y[Y_tmp + 1] -= Y[b_Y_tmp + 1] * f_Y_tmp;
  Y[rtemp + 1] -= Y[b_Y_tmp + 1] * b_A[r3];
  Y[rtemp + 1] -= Y[Y_tmp + 1] * b_A[r2];
  Y[rtemp + 2] = A[2] / b_A[r1];
  Y[Y_tmp + 2] = A[6] - Y[rtemp + 2] * maxval;
  Y[b_Y_tmp + 2] = A[10] - Y[rtemp + 2] * a21;
  Y[Y_tmp + 2] /= c_Y_tmp;
  Y[b_Y_tmp + 2] -= Y[Y_tmp + 2] * d_Y_tmp;
  Y[b_Y_tmp + 2] /= e_Y_tmp;
  Y[Y_tmp + 2] -= Y[b_Y_tmp + 2] * f_Y_tmp;
  Y[rtemp + 2] -= Y[b_Y_tmp + 2] * b_A[r3];
  Y[rtemp + 2] -= Y[Y_tmp + 2] * b_A[r2];
  Y[rtemp + 3] = A[3] / b_A[r1];
  Y[Y_tmp + 3] = A[7] - Y[rtemp + 3] * maxval;
  Y[b_Y_tmp + 3] = A[11] - Y[rtemp + 3] * a21;
  Y[Y_tmp + 3] /= c_Y_tmp;
  Y[b_Y_tmp + 3] -= Y[Y_tmp + 3] * d_Y_tmp;
  Y[b_Y_tmp + 3] /= e_Y_tmp;
  Y[Y_tmp + 3] -= Y[b_Y_tmp + 3] * f_Y_tmp;
  Y[rtemp + 3] -= Y[b_Y_tmp + 3] * b_A[r3];
  Y[rtemp + 3] -= Y[Y_tmp + 3] * b_A[r2];
}

/*
 * File trailer for mrdivide_helper.c
 *
 * [EOF]
 */
