/*
 * File: MarkerEKF_initialize.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 14-Jul-2016 16:27:02
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "MarkerEKF.h"
#include "MarkerEKF_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void MarkerEKF_initialize(float localsense_initx,float localsense_inity)
{
 // rt_InitInfAndNaN(8U);
  R_not_empty_init();
  Q_not_empty_init();
  MarkerEKF_init(localsense_initx,localsense_inity);
}

/*
 * File trailer for MarkerEKF_initialize.c
 *
 * [EOF]
 */
