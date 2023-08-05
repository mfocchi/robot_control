/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * integrate_dynamics.c
 *
 * Code generation for function 'integrate_dynamics'
 *
 */

/* Include files */
#include "integrate_dynamics.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void b_anon(real_T params_b, const real_T params_p_a1[3], const real_T
            params_p_a2[3], real_T params_g, real_T params_m, const real_T x[6],
            real_T u1, real_T u2, real_T varargout_1[6])
{
  real_T b_x[9];
  real_T c_x[9];
  real_T J_tmp[3];
  real_T p[3];
  real_T a;
  real_T a_tmp;
  real_T a_tmp_tmp;
  real_T b_a_tmp;
  real_T b_p_tmp;
  real_T b_px_tmp;
  real_T b_x_tmp;
  real_T b_x_tmp_tmp;
  real_T c_p_tmp;
  real_T c_x_tmp_tmp;
  real_T n_pz_l1;
  real_T p_tmp;
  real_T params_m_idx_0_tmp;
  real_T px_l1_sinpsi;
  real_T px_l1_tmp;
  real_T px_tmp;
  real_T px_tmp_tmp;
  real_T py2b;
  real_T pz_tmp;
  real_T t;
  real_T t1;
  real_T t2;
  real_T t3;
  real_T x_tmp;
  real_T x_tmp_tmp;
  real_T y;
  int32_T p1;
  int32_T p2;
  int32_T p3;

  /*  because we have time invariant system t wont be used */
  /*  %Retrieving states */
  t = params_b * params_b;
  a_tmp = x[1] * x[1];
  a_tmp_tmp = x[2] * x[2];
  b_a_tmp = (t + a_tmp) - a_tmp_tmp;
  px_tmp_tmp = 4.0 * t;
  px_tmp = muDoubleScalarSin(x[0]);
  t1 = muDoubleScalarSqrt(1.0 - b_a_tmp * b_a_tmp / (px_tmp_tmp * a_tmp));
  b_px_tmp = x[1] * px_tmp;
  t3 = b_px_tmp * t1;
  t2 = b_a_tmp / (2.0 * params_b);
  pz_tmp = muDoubleScalarCos(x[0]);
  t1 *= -x[1] * pz_tmp;
  px_l1_tmp = t3 / x[1];
  n_pz_l1 = -t1 / x[1];
  px_l1_sinpsi = px_l1_tmp / px_tmp;
  py2b = t2 * 2.0 * params_b;

  /*  mass equation and rope constraints  */
  b_a_tmp = ((x[4] * t - x[4] * a_tmp) + 2.0 * x[5] * x[1] * x[2]) - x[4] *
    a_tmp_tmp;
  J_tmp[0] = t3 - params_p_a1[0];
  p[0] = t3 - params_p_a2[0];
  J_tmp[1] = t2 - params_p_a1[1];
  p[1] = t2 - params_p_a2[1];
  J_tmp[2] = t1 - params_p_a1[2];
  p[2] = t1 - params_p_a2[2];
  a = 1.0 / params_m;
  x_tmp = x[1] * n_pz_l1;
  b_x[0] = x_tmp;
  b_x_tmp = 2.0 * t;
  x_tmp_tmp = muDoubleScalarPower(x[1], 3.0);
  b_x_tmp_tmp = py2b * py2b;
  c_x_tmp_tmp = b_x_tmp * x_tmp_tmp;
  t2 = py2b / (t * x[1]) - b_x_tmp_tmp / c_x_tmp_tmp;
  b_x[3] = px_l1_tmp - b_px_tmp * t2 / (2.0 * px_l1_sinpsi);
  t1 = x[2] * py2b;
  t3 = b_x_tmp * x[1] * px_l1_sinpsi;
  b_x[6] = t1 * px_tmp / t3;
  b_x[1] = 0.0;
  b_x[4] = x[1] / params_b;
  b_x[7] = -x[2] / params_b;
  b_x[2] = x[1] * px_l1_tmp;
  b_x[5] = x[1] * pz_tmp * t2 / (2.0 * px_l1_sinpsi) - n_pz_l1;
  b_x[8] = -(t1 * pz_tmp) / t3;
  memcpy(&c_x[0], &b_x[0], 9U * sizeof(real_T));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  if (muDoubleScalarAbs(b_x[2]) > muDoubleScalarAbs(x_tmp)) {
    p1 = 6;
    p3 = 0;
    c_x[0] = b_x[2];
    c_x[2] = x_tmp;
    c_x[3] = b_x[5];
    c_x[5] = b_x[3];
    c_x[6] = b_x[8];
    c_x[8] = b_x[6];
  }

  c_x[1] /= c_x[0];
  c_x[2] /= c_x[0];
  c_x[4] -= c_x[1] * c_x[3];
  c_x[5] -= c_x[2] * c_x[3];
  c_x[7] -= c_x[1] * c_x[6];
  c_x[8] -= c_x[2] * c_x[6];
  if (muDoubleScalarAbs(c_x[5]) > muDoubleScalarAbs(c_x[4])) {
    p2 = p3;
    p3 = 3;
    t1 = c_x[1];
    c_x[1] = c_x[2];
    c_x[2] = t1;
    t1 = c_x[4];
    c_x[4] = c_x[5];
    c_x[5] = t1;
    t1 = c_x[7];
    c_x[7] = c_x[8];
    c_x[8] = t1;
  }

  c_x[5] /= c_x[4];
  c_x[8] -= c_x[5] * c_x[7];
  t3 = (c_x[5] * c_x[1] - c_x[2]) / c_x[8];
  t2 = -(c_x[1] + c_x[7] * t3) / c_x[4];
  b_x[p1] = ((1.0 - c_x[3] * t2) - c_x[6] * t3) / c_x[0];
  b_x[p1 + 1] = t2;
  b_x[p1 + 2] = t3;
  t3 = -c_x[5] / c_x[8];
  t2 = (1.0 - c_x[7] * t3) / c_x[4];
  b_x[p2] = -(c_x[3] * t2 + c_x[6] * t3) / c_x[0];
  b_x[p2 + 1] = t2;
  b_x[p2 + 2] = t3;
  t3 = 1.0 / c_x[8];
  t2 = -c_x[7] * t3 / c_x[4];
  b_x[p3] = -(c_x[3] * t2 + c_x[6] * t3) / c_x[0];
  b_x[p3 + 1] = t2;
  b_x[p3 + 2] = t3;
  t1 = 3.3121686421112381E-170;
  t3 = 3.3121686421112381E-170;
  t2 = muDoubleScalarAbs(J_tmp[0]);
  if (t2 > 3.3121686421112381E-170) {
    b_px_tmp = 1.0;
    t1 = t2;
  } else {
    t = t2 / 3.3121686421112381E-170;
    b_px_tmp = t * t;
  }

  t2 = muDoubleScalarAbs(p[0]);
  if (t2 > 3.3121686421112381E-170) {
    y = 1.0;
    t3 = t2;
  } else {
    t = t2 / 3.3121686421112381E-170;
    y = t * t;
  }

  t2 = muDoubleScalarAbs(J_tmp[1]);
  if (t2 > t1) {
    t = t1 / t2;
    b_px_tmp = b_px_tmp * t * t + 1.0;
    t1 = t2;
  } else {
    t = t2 / t1;
    b_px_tmp += t * t;
  }

  t2 = muDoubleScalarAbs(p[1]);
  if (t2 > t3) {
    t = t3 / t2;
    y = y * t * t + 1.0;
    t3 = t2;
  } else {
    t = t2 / t3;
    y += t * t;
  }

  t2 = muDoubleScalarAbs(J_tmp[2]);
  if (t2 > t1) {
    t = t1 / t2;
    b_px_tmp = b_px_tmp * t * t + 1.0;
    t1 = t2;
  } else {
    t = t2 / t1;
    b_px_tmp += t * t;
  }

  t2 = muDoubleScalarAbs(p[2]);
  if (t2 > t3) {
    t = t3 / t2;
    y = y * t * t + 1.0;
    t3 = t2;
  } else {
    t = t2 / t3;
    y += t * t;
  }

  b_px_tmp = t1 * muDoubleScalarSqrt(b_px_tmp);
  y = t3 * muDoubleScalarSqrt(y);
  params_m_idx_0_tmp = params_m * 0.0;
  for (p1 = 0; p1 < 3; p1++) {
    J_tmp[p1] = J_tmp[p1] / b_px_tmp * u1 + p[p1] / y * u2;
  }

  t3 = x[4] * x[4];
  p_tmp = x[5] * x[5];
  b_p_tmp = x[3] * x[3];
  c_p_tmp = (((((4.0 * muDoubleScalarPower(x[1], 4.0) * t3 - 8.0 * x_tmp_tmp *
                 x[2] * x[4] * x[5]) + 4.0 * a_tmp * a_tmp_tmp * p_tmp) - 6.0 *
               a_tmp * t3 * py2b) - 2.0 * a_tmp * p_tmp * py2b) + 8.0 * x[1] *
             x[2] * x[4] * x[5] * py2b) + 3.0 * t3 * b_x_tmp_tmp;
  x_tmp_tmp = px_tmp_tmp * x_tmp_tmp * px_l1_sinpsi;
  a_tmp_tmp = 16.0 * muDoubleScalarPower(params_b, 4.0) * muDoubleScalarPower(x
    [1], 5.0) * muDoubleScalarPower(px_l1_sinpsi, 3.0);
  px_tmp_tmp = x[4] * py2b;
  y = c_x_tmp_tmp * px_l1_sinpsi;
  b_px_tmp = x[3] * py2b;
  t1 = b_x_tmp * a_tmp * px_l1_sinpsi;
  t2 = b_a_tmp * b_a_tmp;
  t = a * (params_m_idx_0_tmp + J_tmp[0]) - (((((2.0 * x[4] * n_pz_l1 * x[3] -
    x[1] * b_p_tmp * px_l1_tmp) - px_tmp * c_p_tmp / x_tmp_tmp) - b_x_tmp_tmp *
    px_tmp * t2 / a_tmp_tmp) + b_px_tmp * pz_tmp * b_a_tmp / t1) + px_tmp_tmp *
    px_tmp * b_a_tmp / y);
  t3 = a * (params_m_idx_0_tmp + J_tmp[1]) - (t3 - p_tmp) / params_b;
  t1 = a * (params_m * -params_g + J_tmp[2]) - (((((x_tmp * b_p_tmp + 2.0 * x[4]
    * x[3] * px_l1_tmp) + pz_tmp * c_p_tmp / x_tmp_tmp) + b_x_tmp_tmp * pz_tmp *
    t2 / a_tmp_tmp) - px_tmp_tmp * pz_tmp * b_a_tmp / y) + b_px_tmp * px_tmp *
    b_a_tmp / t1);
  varargout_1[0] = x[3];
  varargout_1[1] = x[4];
  varargout_1[2] = x[5];
  for (p1 = 0; p1 < 3; p1++) {
    varargout_1[p1 + 3] = (b_x[p1] * t + b_x[p1 + 3] * t3) + b_x[p1 + 6] * t1;
  }
}

/* End of code generation (integrate_dynamics.c) */
