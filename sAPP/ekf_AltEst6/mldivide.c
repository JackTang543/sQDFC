/*
 * File: mldivide.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-05-04 12:04:24
 */

/* Include Files */
#include "mldivide.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const float A[9]
 *                const float B[3]
 *                float Y[3]
 * Return Type  : void
 */
void mldivide(const float A[9], const float B[3], float Y[3])
{
  float b_A[9];
  float a21;
  float maxval;
  int r1;
  int r2;
  int r3;
  for (r1 = 0; r1 < 9; r1++) {
    b_A[r1] = A[r1];
  }
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabsf(A[0]);
  a21 = fabsf(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabsf(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (fabsf(b_A[r3 + 3]) > fabsf(b_A[r2 + 3])) {
    int rtemp;
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  Y[1] = B[r2] - B[r1] * b_A[r2];
  Y[2] = (B[r3] - B[r1] * b_A[r3]) - Y[1] * b_A[r3 + 3];
  Y[2] /= b_A[r3 + 6];
  Y[0] = B[r1] - Y[2] * b_A[r1 + 6];
  Y[1] -= Y[2] * b_A[r2 + 6];
  Y[1] /= b_A[r2 + 3];
  Y[0] -= Y[1] * b_A[r1 + 3];
  Y[0] /= b_A[r1];
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
