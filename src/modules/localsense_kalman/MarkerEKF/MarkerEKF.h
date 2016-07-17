/*
 * File: MarkerEKF.h
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 14-Jul-2016 16:27:02
 */

#ifndef __MARKEREKF_H__
#define __MARKEREKF_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "MarkerEKF_types.h"

/* Function Declarations */
extern void MarkerEKF(float dt, const float z[4], float q_a, float q_v, float
                      q_x, float r_a, float r_x, float xa_apo[6], float Pa_apo
                      [36]);
extern void MarkerEKF_init(float localsense_initx,float localsense_inity);
extern void Q_not_empty_init(void);
extern void R_not_empty_init(void);

#endif

/*
 * File trailer for MarkerEKF.h
 *
 * [EOF]
 */
