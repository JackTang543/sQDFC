/*
 * File: ekf_AltEst6.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-05-04 12:04:24
 */

#ifndef EKF_ALTEST6_H
#define EKF_ALTEST6_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void ekf_AltEst6(const float gyr[3], const float acc[3],
                        float acc_notrust_factor, float dt, float eul[3],
                        float quat[4], float state[5]);

void ekf_AltEst6_init(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ekf_AltEst6.h
 *
 * [EOF]
 */
