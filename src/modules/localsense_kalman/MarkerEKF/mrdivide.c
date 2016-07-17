/*
 * File: mrdivide.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 14-Jul-2016 16:27:02
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "MarkerEKF.h"
#include "mrdivide.h"

/* Function Definitions */

/*
 * Arguments    : float A[24]
 *                const float B[16]
 * Return Type  : void
 */
void mrdivide(float A[24], const float B[16])
{
  float b_A[16];
  signed char ipiv[4];
  int k;
  int j;
  int c;
  int kBcol;
  int ix;
  float temp;
  float s;
  int i;
  int jp;
  int jAcol;
  memcpy(&b_A[0], &B[0], sizeof(float) << 4);
  for (k = 0; k < 4; k++) {
    ipiv[k] = (signed char)(1 + k);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    kBcol = 0;
    ix = c;
    temp = (real32_T)fabs(b_A[c]);
    for (k = 2; k <= 4 - j; k++) {
      ix++;
      s = (real32_T)fabs(b_A[ix]);
      if (s > temp) {
        kBcol = k - 1;
        temp = s;
      }
    }

    if (b_A[c + kBcol] != 0.0F) {
      if (kBcol != 0) {
        ipiv[j] = (signed char)((j + kBcol) + 1);
        ix = j;
        kBcol += j;
        for (k = 0; k < 4; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[kBcol];
          b_A[kBcol] = temp;
          ix += 4;
          kBcol += 4;
        }
      }

      k = (c - j) + 4;
      for (i = c + 1; i + 1 <= k; i++) {
        b_A[i] /= b_A[c];
      }
    }

    jp = c;
    jAcol = c + 4;
    for (kBcol = 1; kBcol <= 3 - j; kBcol++) {
      temp = b_A[jAcol];
      if (b_A[jAcol] != 0.0F) {
        ix = c + 1;
        k = (jp - j) + 8;
        for (i = 5 + jp; i + 1 <= k; i++) {
          b_A[i] += b_A[ix] * -temp;
          ix++;
        }
      }

      jAcol += 4;
      jp += 4;
    }
  }

  for (j = 0; j < 4; j++) {
    jp = 6 * j;
    jAcol = j << 2;
    for (k = 1; k <= j; k++) {
      kBcol = 6 * (k - 1);
      if (b_A[(k + jAcol) - 1] != 0.0F) {
        for (i = 0; i < 6; i++) {
          A[i + jp] -= b_A[(k + jAcol) - 1] * A[i + kBcol];
        }
      }
    }

    temp = 1.0F / b_A[j + jAcol];
    for (i = 0; i < 6; i++) {
      A[i + jp] *= temp;
    }
  }

  for (j = 3; j > -1; j += -1) {
    jp = 6 * j;
    jAcol = (j << 2) - 1;
    for (k = j + 2; k < 5; k++) {
      kBcol = 6 * (k - 1);
      if (b_A[k + jAcol] != 0.0F) {
        for (i = 0; i < 6; i++) {
          A[i + jp] -= b_A[k + jAcol] * A[i + kBcol];
        }
      }
    }
  }

  for (kBcol = 2; kBcol > -1; kBcol += -1) {
    if (ipiv[kBcol] != kBcol + 1) {
      jp = ipiv[kBcol] - 1;
      for (jAcol = 0; jAcol < 6; jAcol++) {
        temp = A[jAcol + 6 * kBcol];
        A[jAcol + 6 * kBcol] = A[jAcol + 6 * jp];
        A[jAcol + 6 * jp] = temp;
      }
    }
  }
}

/*
 * File trailer for mrdivide.c
 *
 * [EOF]
 */
