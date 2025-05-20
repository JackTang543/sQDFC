/*
 * File: rk4_AltEst3.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2025-05-04 15:49:56
 */

#ifndef RK4_ALTEST3_H
#define RK4_ALTEST3_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void rk4_AltEst3(const float gyr[3], float dt, float eul[3],
                        float quat[4]);

void rk4_AltEst3_init(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for rk4_AltEst3.h
 *
 * [EOF]
 */
