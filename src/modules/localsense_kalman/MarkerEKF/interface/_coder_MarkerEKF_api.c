/*
 * File: _coder_MarkerEKF_api.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 14-Jul-2016 16:27:02
 */

/* Include Files */
#include "_coder_MarkerEKF_api.h"

/* Function Declarations */
static real32_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char_T *identifier);
static real32_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real32_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z,
  const char_T *identifier))[4];
static real32_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[4];
static const mxArray *emlrt_marshallOut(const real32_T u[6]);
static const mxArray *b_emlrt_marshallOut(const real32_T u[36]);
static real32_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static real32_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4];

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void MarkerEKF_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void MarkerEKF_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void MarkerEKF_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  MarkerEKF_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[7]
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void MarkerEKF_api(const mxArray *prhs[7], const mxArray *plhs[2])
{
  real32_T (*xa_apo)[6];
  real32_T (*Pa_apo)[36];
  real32_T dt;
  real32_T (*z)[4];
  real32_T q_a;
  real32_T q_v;
  real32_T q_x;
  real32_T r_a;
  real32_T r_x;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  xa_apo = (real32_T (*)[6])mxMalloc(sizeof(real32_T [6]));
  Pa_apo = (real32_T (*)[36])mxMalloc(sizeof(real32_T [36]));
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  dt = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "dt");
  z = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "z");
  q_a = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "q_a");
  q_v = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "q_v");
  q_x = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "q_x");
  r_a = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "r_a");
  r_x = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "r_x");

  /* Invoke the target function */
  MarkerEKF(dt, *z, q_a, q_v, q_x, r_a, r_x, *xa_apo, *Pa_apo);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*xa_apo);
  plhs[1] = b_emlrt_marshallOut(*Pa_apo);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *dt
 *                const char_T *identifier
 * Return Type  : real32_T
 */
static real32_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char_T *identifier)
{
  real32_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(dt), &thisId);
  emlrtDestroyArray(&dt);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T
 */
static real32_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real32_T y;
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *z
 *                const char_T *identifier
 * Return Type  : real32_T (*)[4]
 */
static real32_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z,
  const char_T *identifier))[4]
{
  real32_T (*y)[4];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(z), &thisId);
  emlrtDestroyArray(&z);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T (*)[4]
 */
  static real32_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[4]
{
  real32_T (*y)[4];
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const real32_T u[6]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real32_T u[6])
{
  const mxArray *y;
  static const int32_T iv0[1] = { 0 };

  const mxArray *m0;
  static const int32_T iv1[1] = { 6 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const real32_T u[36]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real32_T u[36])
{
  const mxArray *y;
  static const int32_T iv2[2] = { 0, 0 };

  const mxArray *m1;
  static const int32_T iv3[2] = { 6, 6 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv2, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u);
  emlrtSetDimensions((mxArray *)m1, iv3, 2);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T
 */
static real32_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  real32_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 0U, 0);
  ret = *(real32_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T (*)[4]
 */
static real32_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4]
{
  real32_T (*ret)[4];
  int32_T iv4[1];
  iv4[0] = 4;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 1U, iv4);
  ret = (real32_T (*)[4])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * File trailer for _coder_MarkerEKF_api.c
 *
 * [EOF]
 */
