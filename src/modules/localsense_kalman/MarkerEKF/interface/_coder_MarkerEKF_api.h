/* 
 * File: _coder_MarkerEKF_api.h 
 *  
 * MATLAB Coder version            : 2.7 
 * C/C++ source code generated on  : 14-Jul-2016 16:27:02 
 */

#ifndef ___CODER_MARKEREKF_API_H__
#define ___CODER_MARKEREKF_API_H__
/* Include Files */ 
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */ 
extern void MarkerEKF_initialize(emlrtContext *aContext);
extern void MarkerEKF_terminate(void);
extern void MarkerEKF_atexit(void);
extern void MarkerEKF_api(const mxArray *prhs[7], const mxArray *plhs[2]);
extern void MarkerEKF(real32_T dt, real32_T z[4], real32_T q_a, real32_T q_v, real32_T q_x, real32_T r_a, real32_T r_x, real32_T xa_apo[6], real32_T Pa_apo[36]);
extern void MarkerEKF_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_MarkerEKF_api.h 
 *  
 * [EOF] 
 */
