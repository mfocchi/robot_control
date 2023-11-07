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
  real_T absxk;
  real_T b_a_tmp;
  real_T b_px_tmp;
  real_T b_x_tmp;
  real_T b_y;
  real_T c_a_tmp;
  real_T n_bar_tmp_idx_0;
  real_T n_bar_tmp_idx_1;
  real_T n_bar_tmp_idx_2;
  real_T n_par_tmp_idx_0;
  real_T n_par_tmp_idx_1;
  real_T n_par_tmp_idx_2;
  real_T n_pz_l1;
  real_T p_tmp;
  real_T px_l1_sinpsi;
  real_T px_l1_tmp;
  real_T px_tmp;
  real_T px_tmp_tmp;
  real_T py;
  real_T py2b;
  real_T pz_tmp;
  real_T scale;
  real_T t;
  real_T t1;
  real_T t3;
  real_T x_tmp;
  real_T x_tmp_tmp;
  real_T y;
  int32_T p1;
  int32_T p2;
  int32_T p3;

  /*  because we have time invariant system t wont be used */
  /*  %Retrieving states */
  a_tmp = params_b * params_b;
  b_a_tmp = x[1] * x[1];
  a_tmp_tmp = x[2] * x[2];
  c_a_tmp = (a_tmp + b_a_tmp) - a_tmp_tmp;
  px_tmp_tmp = 4.0 * a_tmp;
  px_tmp = muDoubleScalarSin(x[0]);
  t1 = muDoubleScalarSqrt(1.0 - c_a_tmp * c_a_tmp / (px_tmp_tmp * b_a_tmp));
  b_px_tmp = x[1] * px_tmp;
  t3 = b_px_tmp * t1;
  py = c_a_tmp / (2.0 * params_b);
  pz_tmp = muDoubleScalarCos(x[0]);
  t1 *= -x[1] * pz_tmp;
  px_l1_tmp = t3 / x[1];
  n_pz_l1 = -t1 / x[1];
  px_l1_sinpsi = px_l1_tmp / px_tmp;
  py2b = py * 2.0 * params_b;

  /*  mass equation and rope constraints  */
  c_a_tmp = ((x[4] * a_tmp - x[4] * b_a_tmp) + 2.0 * x[5] * x[1] * x[2]) - x[4] *
    a_tmp_tmp;

  /*  extra force is applied perpendicularly to the rope plane (TODO */
  /*  make it generic) */
  scale = 3.3121686421112381E-170;
  J_tmp[0] = t3 - params_p_a1[0];
  p[0] = t3 - params_p_a2[0];
  n_par_tmp_idx_2 = params_p_a1[0] - params_p_a2[0];
  n_par_tmp_idx_0 = n_par_tmp_idx_2;
  absxk = muDoubleScalarAbs(n_par_tmp_idx_2);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  J_tmp[1] = py - params_p_a1[1];
  p[1] = py - params_p_a2[1];
  n_par_tmp_idx_2 = params_p_a1[1] - params_p_a2[1];
  n_par_tmp_idx_1 = n_par_tmp_idx_2;
  absxk = muDoubleScalarAbs(n_par_tmp_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  J_tmp[2] = t1 - params_p_a1[2];
  p[2] = t1 - params_p_a2[2];
  n_par_tmp_idx_2 = params_p_a1[2] - params_p_a2[2];
  absxk = muDoubleScalarAbs(n_par_tmp_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * muDoubleScalarSqrt(y);
  n_par_tmp_idx_0 /= y;
  t3 = p[0] / x[2];
  n_par_tmp_idx_1 /= y;
  py = p[1] / x[2];
  n_par_tmp_idx_2 /= y;
  t1 = p[2] / x[2];
  n_bar_tmp_idx_0 = n_par_tmp_idx_1 * t1 - n_par_tmp_idx_2 * py;
  n_bar_tmp_idx_1 = n_par_tmp_idx_2 * t3 - n_par_tmp_idx_0 * t1;
  n_bar_tmp_idx_2 = n_par_tmp_idx_0 * py - n_par_tmp_idx_1 * t3;
  a = 1.0 / params_m;
  x_tmp = x[1] * n_pz_l1;
  b_x[0] = x_tmp;
  b_x_tmp = 2.0 * a_tmp;
  n_par_tmp_idx_2 = muDoubleScalarPower(x[1], 3.0);
  x_tmp_tmp = py2b * py2b;
  n_par_tmp_idx_0 = b_x_tmp * n_par_tmp_idx_2;
  py = py2b / (a_tmp * x[1]) - x_tmp_tmp / n_par_tmp_idx_0;
  b_x[3] = px_l1_tmp - b_px_tmp * py / (2.0 * px_l1_sinpsi);
  t1 = x[2] * py2b;
  t3 = b_x_tmp * x[1] * px_l1_sinpsi;
  b_x[6] = t1 * px_tmp / t3;
  b_x[1] = 0.0;
  b_x[4] = x[1] / params_b;
  b_x[7] = -x[2] / params_b;
  b_x[2] = x[1] * px_l1_tmp;
  b_x[5] = x[1] * pz_tmp * py / (2.0 * px_l1_sinpsi) - n_pz_l1;
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
  t1 = -(c_x[1] + c_x[7] * t3) / c_x[4];
  b_x[p1] = ((1.0 - c_x[3] * t1) - c_x[6] * t3) / c_x[0];
  b_x[p1 + 1] = t1;
  b_x[p1 + 2] = t3;
  t3 = -c_x[5] / c_x[8];
  t1 = (1.0 - c_x[7] * t3) / c_x[4];
  b_x[p2] = -(c_x[3] * t1 + c_x[6] * t3) / c_x[0];
  b_x[p2 + 1] = t1;
  b_x[p2 + 2] = t3;
  t3 = 1.0 / c_x[8];
  t1 = -c_x[7] * t3 / c_x[4];
  b_x[p3] = -(c_x[3] * t1 + c_x[6] * t3) / c_x[0];
  b_x[p3 + 1] = t1;
  b_x[p3 + 2] = t3;
  scale = 3.3121686421112381E-170;
  t1 = 3.3121686421112381E-170;
  t3 = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(J_tmp[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = muDoubleScalarAbs(p[0]);
  if (absxk > 3.3121686421112381E-170) {
    py = 1.0;
    t1 = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    py = t * t;
  }

  absxk = muDoubleScalarAbs(n_bar_tmp_idx_0);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    t3 = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  absxk = muDoubleScalarAbs(J_tmp[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = muDoubleScalarAbs(p[1]);
  if (absxk > t1) {
    t = t1 / absxk;
    py = py * t * t + 1.0;
    t1 = absxk;
  } else {
    t = absxk / t1;
    py += t * t;
  }

  absxk = muDoubleScalarAbs(n_bar_tmp_idx_1);
  if (absxk > t3) {
    t = t3 / absxk;
    b_y = b_y * t * t + 1.0;
    t3 = absxk;
  } else {
    t = absxk / t3;
    b_y += t * t;
  }

  absxk = muDoubleScalarAbs(J_tmp[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = muDoubleScalarAbs(p[2]);
  if (absxk > t1) {
    t = t1 / absxk;
    py = py * t * t + 1.0;
    t1 = absxk;
  } else {
    t = absxk / t1;
    py += t * t;
  }

  absxk = muDoubleScalarAbs(n_bar_tmp_idx_2);
  if (absxk > t3) {
    t = t3 / absxk;
    b_y = b_y * t * t + 1.0;
    t3 = absxk;
  } else {
    t = absxk / t3;
    b_y += t * t;
  }

  y = scale * muDoubleScalarSqrt(y);
  py = t1 * muDoubleScalarSqrt(py);
  b_y = t3 * muDoubleScalarSqrt(b_y);
  n_par_tmp_idx_1 = params_m * 0.0;
  for (p1 = 0; p1 < 3; p1++) {
    J_tmp[p1] = J_tmp[p1] / y * u1 + p[p1] / py * u2;
  }

  scale = x[4] * x[4];
  b_px_tmp = x[5] * x[5];
  p_tmp = x[3] * x[3];
  y = (((((4.0 * muDoubleScalarPower(x[1], 4.0) * scale - 8.0 * n_par_tmp_idx_2 *
           x[2] * x[4] * x[5]) + 4.0 * b_a_tmp * a_tmp_tmp * b_px_tmp) - 6.0 *
         b_a_tmp * scale * py2b) - 2.0 * b_a_tmp * b_px_tmp * py2b) + 8.0 * x[1]
       * x[2] * x[4] * x[5] * py2b) + 3.0 * scale * x_tmp_tmp;
  absxk = px_tmp_tmp * n_par_tmp_idx_2 * px_l1_sinpsi;
  t = 16.0 * muDoubleScalarPower(params_b, 4.0) * muDoubleScalarPower(x[1], 5.0)
    * muDoubleScalarPower(px_l1_sinpsi, 3.0);
  a_tmp = x[4] * py2b;
  n_par_tmp_idx_2 = n_par_tmp_idx_0 * px_l1_sinpsi;
  py = x[3] * py2b;
  t1 = b_x_tmp * b_a_tmp * px_l1_sinpsi;
  t3 = c_a_tmp * c_a_tmp;
  n_par_tmp_idx_0 = a * ((n_par_tmp_idx_1 + J_tmp[0]) + 0.0 * (n_bar_tmp_idx_0 /
    b_y)) - (((((2.0 * x[4] * n_pz_l1 * x[3] - x[1] * p_tmp * px_l1_tmp) -
                px_tmp * y / absxk) - x_tmp_tmp * px_tmp * t3 / t) + py * pz_tmp
              * c_a_tmp / t1) + a_tmp * px_tmp * c_a_tmp / n_par_tmp_idx_2);
  n_par_tmp_idx_1 = a * ((n_par_tmp_idx_1 + J_tmp[1]) + 0.0 * (n_bar_tmp_idx_1 /
    b_y)) - (scale - b_px_tmp) / params_b;
  n_par_tmp_idx_2 = a * ((params_m * -params_g + J_tmp[2]) + 0.0 *
    (n_bar_tmp_idx_2 / b_y)) - (((((x_tmp * p_tmp + 2.0 * x[4] * x[3] *
    px_l1_tmp) + pz_tmp * y / absxk) + x_tmp_tmp * pz_tmp * t3 / t) - a_tmp *
    pz_tmp * c_a_tmp / n_par_tmp_idx_2) + py * px_tmp * c_a_tmp / t1);
  varargout_1[0] = x[3];
  varargout_1[1] = x[4];
  varargout_1[2] = x[5];
  for (p1 = 0; p1 < 3; p1++) {
    varargout_1[p1 + 3] = (b_x[p1] * n_par_tmp_idx_0 + b_x[p1 + 3] *
      n_par_tmp_idx_1) + b_x[p1 + 6] * n_par_tmp_idx_2;
  }
}

/* End of code generation (integrate_dynamics.c) */
