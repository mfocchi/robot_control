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
void b_anon(real_T params_m, real_T params_b, const real_T params_p_a1[3], const
            real_T params_p_a2[3], real_T params_g, real_T params_T_th, real_T t,
            const real_T x[6], real_T u1, real_T u2, const real_T u3[3], real_T
            varargout_1[6])
{
  real_T b_x[9];
  real_T c_x[9];
  real_T Ftot[3];
  real_T J_tmp[3];
  real_T p[3];
  real_T a;
  real_T a_tmp;
  real_T a_tmp_tmp;
  real_T absxk;
  real_T b_a_tmp;
  real_T b_px_tmp;
  real_T b_scale;
  real_T b_t;
  real_T b_x_tmp;
  real_T b_x_tmp_tmp;
  real_T b_y;
  real_T c_a_tmp;
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
  real_T t1;
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
  absxk = b_px_tmp * t1;
  py = c_a_tmp / (2.0 * params_b);
  pz_tmp = muDoubleScalarCos(x[0]);
  t1 *= -x[1] * pz_tmp;
  px_l1_tmp = absxk / x[1];
  n_pz_l1 = -t1 / x[1];
  px_l1_sinpsi = px_l1_tmp / px_tmp;
  py2b = py * 2.0 * params_b;

  /*  mass equation and rope constraints  */
  c_a_tmp = ((x[4] * a_tmp - x[4] * b_a_tmp) + 2.0 * x[5] * x[1] * x[2]) - x[4] *
    a_tmp_tmp;
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  b_t = absxk - params_p_a1[0];
  J_tmp[0] = b_t;
  p_tmp = absxk - params_p_a2[0];
  p[0] = p_tmp;
  absxk = muDoubleScalarAbs(b_t);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    y = b_t * b_t;
  }

  absxk = muDoubleScalarAbs(p_tmp);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    b_y = b_t * b_t;
  }

  b_t = py - params_p_a1[1];
  J_tmp[1] = b_t;
  p_tmp = py - params_p_a2[1];
  p[1] = p_tmp;
  absxk = muDoubleScalarAbs(b_t);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }

  absxk = muDoubleScalarAbs(p_tmp);
  if (absxk > b_scale) {
    b_t = b_scale / absxk;
    b_y = b_y * b_t * b_t + 1.0;
    b_scale = absxk;
  } else {
    b_t = absxk / b_scale;
    b_y += b_t * b_t;
  }

  b_t = t1 - params_p_a1[2];
  J_tmp[2] = b_t;
  p_tmp = t1 - params_p_a2[2];
  p[2] = p_tmp;
  absxk = muDoubleScalarAbs(b_t);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }

  absxk = muDoubleScalarAbs(p_tmp);
  if (absxk > b_scale) {
    b_t = b_scale / absxk;
    b_y = b_y * b_t * b_t + 1.0;
    b_scale = absxk;
  } else {
    b_t = absxk / b_scale;
    b_y += b_t * b_t;
  }

  y = scale * muDoubleScalarSqrt(y);
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  Ftot[0] = params_m * 0.0;
  Ftot[1] = params_m * 0.0;
  Ftot[2] = params_m * -params_g;
  t1 = 0.0;
  scale = 3.3121686421112381E-170;
  for (p1 = 0; p1 < 3; p1++) {
    b_t = J_tmp[p1] / y * u1 + p[p1] / b_y * u2;
    J_tmp[p1] = b_t;
    Ftot[p1] += b_t;
    absxk = muDoubleScalarAbs(u3[p1]);
    if (absxk > scale) {
      b_t = scale / absxk;
      t1 = t1 * b_t * b_t + 1.0;
      scale = absxk;
    } else {
      b_t = absxk / scale;
      t1 += b_t * b_t;
    }
  }

  t1 = scale * muDoubleScalarSqrt(t1);
  if (t1 > 0.0) {
    if (t <= params_T_th) {
      p[0] = u3[0];
      p[1] = u3[1];
      p[2] = u3[2];

      /*  make sure is a column vector */
    } else {
      p[0] = 0.0;
      p[1] = 0.0;
      p[2] = 0.0;
    }

    Ftot[0] += p[0];
    Ftot[1] += p[1];
    Ftot[2] += p[2];
  }

  a = 1.0 / params_m;
  x_tmp = x[1] * n_pz_l1;
  b_x[0] = x_tmp;
  b_x_tmp = 2.0 * a_tmp;
  b_scale = muDoubleScalarPower(x[1], 3.0);
  x_tmp_tmp = py2b * py2b;
  b_x_tmp_tmp = b_x_tmp * b_scale;
  b_t = py2b / (a_tmp * x[1]) - x_tmp_tmp / b_x_tmp_tmp;
  b_x[3] = px_l1_tmp - b_px_tmp * b_t / (2.0 * px_l1_sinpsi);
  t1 = x[2] * py2b;
  absxk = b_x_tmp * x[1] * px_l1_sinpsi;
  b_x[6] = t1 * px_tmp / absxk;
  b_x[1] = 0.0;
  b_x[4] = x[1] / params_b;
  b_x[7] = -x[2] / params_b;
  b_x[2] = x[1] * px_l1_tmp;
  b_x[5] = x[1] * pz_tmp * b_t / (2.0 * px_l1_sinpsi) - n_pz_l1;
  b_x[8] = -(t1 * pz_tmp) / absxk;
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
  t1 = (c_x[5] * c_x[1] - c_x[2]) / c_x[8];
  absxk = -(c_x[1] + c_x[7] * t1) / c_x[4];
  b_x[p1] = ((1.0 - c_x[3] * absxk) - c_x[6] * t1) / c_x[0];
  b_x[p1 + 1] = absxk;
  b_x[p1 + 2] = t1;
  t1 = -c_x[5] / c_x[8];
  absxk = (1.0 - c_x[7] * t1) / c_x[4];
  b_x[p2] = -(c_x[3] * absxk + c_x[6] * t1) / c_x[0];
  b_x[p2 + 1] = absxk;
  b_x[p2 + 2] = t1;
  t1 = 1.0 / c_x[8];
  absxk = -c_x[7] * t1 / c_x[4];
  b_x[p3] = -(c_x[3] * absxk + c_x[6] * t1) / c_x[0];
  b_x[p3 + 1] = absxk;
  b_x[p3 + 2] = t1;
  scale = x[4] * x[4];
  a_tmp = x[5] * x[5];
  b_px_tmp = x[3] * x[3];
  py = (((((4.0 * muDoubleScalarPower(x[1], 4.0) * scale - 8.0 * b_scale * x[2] *
            x[4] * x[5]) + 4.0 * b_a_tmp * a_tmp_tmp * a_tmp) - 6.0 * b_a_tmp *
          scale * py2b) - 2.0 * b_a_tmp * a_tmp * py2b) + 8.0 * x[1] * x[2] * x
        [4] * x[5] * py2b) + 3.0 * scale * x_tmp_tmp;
  p_tmp = px_tmp_tmp * b_scale * px_l1_sinpsi;
  y = 16.0 * muDoubleScalarPower(params_b, 4.0) * muDoubleScalarPower(x[1], 5.0)
    * muDoubleScalarPower(px_l1_sinpsi, 3.0);
  b_y = x[4] * py2b;
  b_scale = b_x_tmp_tmp * px_l1_sinpsi;
  b_t = x[3] * py2b;
  t1 = b_x_tmp * b_a_tmp * px_l1_sinpsi;
  absxk = c_a_tmp * c_a_tmp;
  Ftot[0] = a * Ftot[0] - (((((2.0 * x[4] * n_pz_l1 * x[3] - x[1] * b_px_tmp *
    px_l1_tmp) - px_tmp * py / p_tmp) - x_tmp_tmp * px_tmp * absxk / y) + b_t *
    pz_tmp * c_a_tmp / t1) + b_y * px_tmp * c_a_tmp / b_scale);
  Ftot[1] = a * Ftot[1] - (scale - a_tmp) / params_b;
  Ftot[2] = a * Ftot[2] - (((((x_tmp * b_px_tmp + 2.0 * x[4] * x[3] * px_l1_tmp)
    + pz_tmp * py / p_tmp) + x_tmp_tmp * pz_tmp * absxk / y) - b_y * pz_tmp *
    c_a_tmp / b_scale) + b_t * px_tmp * c_a_tmp / t1);
  varargout_1[0] = x[3];
  varargout_1[1] = x[4];
  varargout_1[2] = x[5];
  b_t = Ftot[0];
  p_tmp = Ftot[1];
  t1 = Ftot[2];
  for (p1 = 0; p1 < 3; p1++) {
    varargout_1[p1 + 3] = (b_x[p1] * b_t + b_x[p1 + 3] * p_tmp) + b_x[p1 + 6] *
      t1;
  }
}

/* End of code generation (integrate_dynamics.c) */
