/*
 * File: MarkerEKF.c
 *
 * MATLAB Coder version            : 2.7
 * C/C++ source code generated on  : 14-Jul-2016 16:27:02
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "MarkerEKF.h"
#include "mrdivide.h"

/* Variable Definitions */
static float x_apo[6];
static float P_apo[36];
static float Q[36];
static boolean_T Q_not_empty;
static float R[16];
static boolean_T R_not_empty;

/* Function Definitions */

/*
 * 输出
 *  %xa_apo状态变量最优估计，6维，分别为惯性坐标系下的X轴与Y轴的加速度，速度以及位置
 *  ax：惯性坐标系下的X轴加速度
 *  ay：惯性坐标系下的Y轴加速度
 *  vx：惯性坐标系下的X轴速度
 *  vy：惯性坐标系下的Y轴速度
 *  x：惯性坐标系下的X轴坐标
 *  y：惯性坐标系下的Y轴坐标
 * %Pa_apo协方差矩阵最优估计
 * Arguments    : float dt
 *                const float z[4]
 *                float q_a
 *                float q_v
 *                float q_x
 *                float r_a
 *                float r_x
 *                float xa_apo[6]
 *                float Pa_apo[36]
 * Return Type  : void
 */
void MarkerEKF(float dt, const float z[4], float q_a, float q_v, float q_x,
               float r_a, float r_x, float xa_apo[6], float Pa_apo[36])
{
  float ak[2];
  float c;
  int i;
  float x_apr[6];
  static const signed char iv0[6] = { 1, 0, 0, 0, 0, 0 };

  float A[36];
  static const signed char iv1[6] = { 0, 1, 0, 0, 0, 0 };

  float v[6];
  float b_A[36];
  int i0;
  int i1;
  float P_apr[36];
  float b_v[4];
  float K_k[24];
  static const signed char iv2[24] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  float fv0[24];
  static const signed char iv3[24] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  float fv1[16];
  signed char I[36];

  /* 输入 */
  /* persitent x_apo，上一时刻的状态的最优估计 */
  /* 6维，分别为惯性坐标系下的X轴与Y轴的加速度，速度以及位置 */
  /* persitent p_apo，上一时刻的协方差的最优估计 */
  /* Z，测量变量，4维 */
  /*  axz：惯性坐标系下的X轴加速度测量值 */
  /*  ayz：惯性坐标系下的Y轴加速度测量值 */
  /*  xz：惯性坐标系下的X轴位置测量值 */
  /*  yz：惯性坐标系下的Y轴位置测量值 */
  /* dt: dt 积分的步长 */
  /* % init */
  /* 给定初始协方差矩阵，迭代过程会收敛% */
  /*  ax：惯性坐标系下的X轴加速度 */
  /*  ay：惯性坐标系下的Y轴加速度 */
  /*  vx：惯性坐标系下的X轴速度 */
  /*  vy：惯性坐标系下的Y轴速度 */
  /*  x：惯性坐标系下的X轴坐标 */
  /*  y：惯性坐标系下的Y轴坐标 */
  /* % prediction section */
  /* 公式1 */
  /* X(k|k-1)=A X(k-1|k-1)+B U(k) ……….. (1) */
  /* 系统控制变量的输入为U=0 */
  /*  ax(k|k-1)=ax(k-1|k-1) */
  /*  ay(k|k-1)=ay(k-1|k-1) */
  /*  vx(k|k-1)=vx(k-1|k-1)+ax(k-1|k-1)*dt; */
  /*  vy(k|k-1)=vy(k-1|k-1)+ay(k-1|k-1)*dt; */
  /*  x(k|k-1)=x(k-1|k-1)+vx(k-1|k-1)*dt+1/2*ax(k-1|k-1)*dt^2; */
  /*  y(k|k-1)=y(k-1|k-1)+vy(k-1|k-1)*dt+1/2*ay(k-1|k-1)*dt^2; */
  /* A=[1 0 0 0 0 0; */
  /*  0 1 0 0 0 0; */
  /*  dt 0 1 0 0 0; */
  /*  0 dt 0 1 0 0; */
  /*  0 0 dt 0 1 0; */
  /*  0 0 0 dt 0 1; */
  /*  ] */
  ak[0] = x_apo[0];
  ak[1] = x_apo[1];
  c = dt * dt;

  /*  rk=[x;y]+dt*vk; */
  for (i = 0; i < 2; i++) {
    x_apr[i] = ak[i];
  }

  x_apr[2] = x_apo[2] + dt * x_apo[0];
  x_apr[3] = x_apo[3] + dt * x_apo[1];
  x_apr[4] = (x_apo[4] + x_apo[2] * dt) + 0.5F * x_apo[0] * c;
  x_apr[5] = (x_apo[5] + x_apo[3] * dt) + 0.5F * x_apo[1] * c;

  /* x_apr当前时刻的状态估计 */
  /* 公式2 更新协方差矩阵的预测 */
  /*  P(k|k-1)=A P(k-1|k-1) A’+Q */
  /*  A=[1 0 0 0 0 0; */
  /*     0 1 0 0 0 0; */
  /*     dt 0 1 0 0 0; */
  /*     0 dt 0 1 0 0; */
  /*     0 0 dt 0 1 0; */
  /*     0 0 0 dt 0 1;]; */
  for (i = 0; i < 6; i++) {
    A[6 * i] = iv0[i];
    A[1 + 6 * i] = iv1[i];
  }

  A[2] = dt;
  A[8] = 0.0F;
  A[14] = 1.0F;
  A[20] = 0.0F;
  A[26] = 0.0F;
  A[32] = 0.0F;
  A[3] = 0.0F;
  A[9] = dt;
  A[15] = 0.0F;
  A[21] = 1.0F;
  A[27] = 0.0F;
  A[33] = 0.0F;
  A[4] = 0.5F * (dt * dt);
  A[10] = 0.0F;
  A[16] = dt;
  A[22] = 0.0F;
  A[28] = 1.0F;
  A[34] = 0.0F;
  A[5] = 0.0F;
  A[11] = 0.5F * (dt * dt);
  A[17] = 0.0F;
  A[23] = dt;
  A[29] = 0.0F;
  A[35] = 1.0F;
  if (!Q_not_empty) {
    v[0] = q_a;
    v[1] = q_a;
    v[2] = q_v;
    v[3] = q_v;
    v[4] = q_x;
    v[5] = q_x;
    memset(&Q[0], 0, 36U * sizeof(float));
    for (i = 0; i < 6; i++) {
      Q[i + 6 * i] = v[i];
    }

    Q_not_empty = true;
  }

  for (i = 0; i < 6; i++) {
    for (i0 = 0; i0 < 6; i0++) {
      b_A[i + 6 * i0] = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        b_A[i + 6 * i0] += A[i + 6 * i1] * P_apo[i1 + 6 * i0];
      }
    }
  }

  for (i = 0; i < 6; i++) {
    for (i0 = 0; i0 < 6; i0++) {
      c = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        c += b_A[i + 6 * i1] * A[i0 + 6 * i1];
      }

      P_apr[i + 6 * i0] = c + Q[i + 6 * i0];
    }
  }

  /* % update */
  /* 先公式4，再公式3  */
  /*  Kg(k)= P(k|k-1) H’ [ (H P(k|k-1) H’ + R)]^-1  */
  /* H 为测量矩阵 */
  if (!R_not_empty) {
    b_v[0] = r_a;
    b_v[1] = r_a;
    b_v[2] = r_x;
    b_v[3] = r_x;
    memset(&R[0], 0, sizeof(float) << 4);
    for (i = 0; i < 4; i++) {
      R[i + (i << 2)] = b_v[i];
    }

    R_not_empty = true;
  }

  for (i = 0; i < 6; i++) {
    for (i0 = 0; i0 < 4; i0++) {
      K_k[i + 6 * i0] = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        K_k[i + 6 * i0] += P_apr[i + 6 * i1] * (float)iv2[i1 + 6 * i0];
      }
    }
  }

  for (i = 0; i < 4; i++) {
    for (i0 = 0; i0 < 6; i0++) {
      fv0[i + (i0 << 2)] = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        fv0[i + (i0 << 2)] += (float)iv3[i + (i1 << 2)] * P_apr[i1 + 6 * i0];
      }
    }
  }

  for (i = 0; i < 4; i++) {
    for (i0 = 0; i0 < 4; i0++) {
      c = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        c += fv0[i + (i1 << 2)] * (float)iv2[i1 + 6 * i0];
      }

      fv1[i + (i0 << 2)] = c + R[i + (i0 << 2)];
    }
  }

  mrdivide(K_k, fv1);

  /*    */
  /*   公式3，更新最优状态估计 */
  /*  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1)) ……… (3) */
  for (i = 0; i < 4; i++) {
    c = 0.0F;
    for (i0 = 0; i0 < 6; i0++) {
      c += (float)iv3[i + (i0 << 2)] * x_apr[i0];
    }

    b_v[i] = z[i] - c;
  }

  for (i = 0; i < 6; i++) {
    c = 0.0F;
    for (i0 = 0; i0 < 4; i0++) {
      c += K_k[i + 6 * i0] * b_v[i0];
    }

    x_apo[i] = x_apr[i] + c;
  }

  /*    */
  /*   公式5，更新协方差矩阵估计 */
  /*  P(k|k)=（I-Kg(k) H）P(k|k-1) */
  for (i = 0; i < 36; i++) {
    I[i] = 0;
  }

  for (i = 0; i < 6; i++) {
    I[i + 6 * i] = 1;
  }

  for (i = 0; i < 6; i++) {
    for (i0 = 0; i0 < 6; i0++) {
      c = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        c += K_k[i + 6 * i1] * (float)iv3[i1 + (i0 << 2)];
      }

      A[i + 6 * i0] = (float)I[i + 6 * i0] - c;
    }
  }

  for (i = 0; i < 6; i++) {
    for (i0 = 0; i0 < 6; i0++) {
      P_apo[i + 6 * i0] = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        P_apo[i + 6 * i0] += A[i + 6 * i1] * P_apr[i1 + 6 * i0];
      }
    }

    xa_apo[i] = x_apo[i];
  }

  memcpy(&Pa_apo[0], &P_apo[0], 36U * sizeof(float));
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void MarkerEKF_init(void)
{
  int i;
  static const signed char iv4[6] = { 0, 0, 0, 0, 10, 10 };

  for (i = 0; i < 6; i++) {
    x_apo[i] = iv4[i];
  }

  for (i = 0; i < 36; i++) {
    P_apo[i] = 1.0F;
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void Q_not_empty_init(void)
{
  Q_not_empty = false;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void R_not_empty_init(void)
{
  R_not_empty = false;
}

/*
 * File trailer for MarkerEKF.c
 *
 * [EOF]
 */
