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
  real_T n_bar_tmp[3];
  real_T n_par[3];
  real_T p[3];
  real_T rope2_axis[3];
  real_T a_tmp;
  real_T a_tmp_tmp;
  real_T absxk;
  real_T b_a_tmp;
  real_T b_px_tmp;
  real_T b_rope2_axis_tmp;
  real_T b_scale;
  real_T b_t;
  real_T b_y;
  real_T c_a_tmp;
  real_T c_rope2_axis_tmp;
  real_T c_scale;
  real_T n_pz_l1;
  real_T px_l1_sinpsi;
  real_T px_l1_tmp;
  real_T px_tmp;
  real_T px_tmp_tmp;
  real_T py;
  real_T py2b;
  real_T pz;
  real_T pz_tmp;
  real_T rope2_axis_tmp;
  real_T scale;
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
  t3 = muDoubleScalarSqrt(1.0 - c_a_tmp * c_a_tmp / (px_tmp_tmp * b_a_tmp));
  b_px_tmp = x[1] * px_tmp;
  t1 = b_px_tmp * t3;
  py = c_a_tmp / (2.0 * params_b);
  pz_tmp = muDoubleScalarCos(x[0]);
  pz = -x[1] * pz_tmp * t3;
  px_l1_tmp = t1 / x[1];
  n_pz_l1 = -pz / x[1];
  px_l1_sinpsi = px_l1_tmp / px_tmp;
  py2b = py * 2.0 * params_b;

  /*  mass equation and rope constraints  */
  c_a_tmp = ((x[4] * a_tmp - x[4] * b_a_tmp) + 2.0 * x[5] * x[1] * x[2]) - x[4] *
    a_tmp_tmp;

  /*  extra force is applied perpendicularly to the rope plane (TODO */
  /*  make it generic) */
  scale = 3.3121686421112381E-170;
  x_tmp = params_p_a1[0] - params_p_a2[0];
  n_par[0] = x_tmp;
  absxk = muDoubleScalarAbs(x_tmp);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    y = b_t * b_t;
  }

  x_tmp = params_p_a1[1] - params_p_a2[1];
  n_par[1] = x_tmp;
  absxk = muDoubleScalarAbs(x_tmp);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }

  x_tmp = params_p_a1[2] - params_p_a2[2];
  absxk = muDoubleScalarAbs(x_tmp);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }

  y = scale * muDoubleScalarSqrt(y);
  n_par[0] /= y;
  rope2_axis_tmp = t1 - params_p_a2[0];
  rope2_axis[0] = rope2_axis_tmp / x[2];
  n_par[1] /= y;
  b_rope2_axis_tmp = py - params_p_a2[1];
  rope2_axis[1] = b_rope2_axis_tmp / x[2];
  n_par[2] = x_tmp / y;
  c_rope2_axis_tmp = pz - params_p_a2[2];
  rope2_axis[2] = c_rope2_axis_tmp / x[2];
  n_bar_tmp[0] = n_par[1] * rope2_axis[2] - n_par[2] * rope2_axis[1];
  n_bar_tmp[1] = n_par[2] * rope2_axis[0] - n_par[0] * rope2_axis[2];
  n_bar_tmp[2] = n_par[0] * rope2_axis[1] - n_par[1] * rope2_axis[0];
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  c_scale = 3.3121686421112381E-170;
  x_tmp = t1 - params_p_a1[0];
  n_par[0] = x_tmp;
  p[0] = rope2_axis_tmp;
  absxk = muDoubleScalarAbs(x_tmp);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    y = b_t * b_t;
  }

  absxk = muDoubleScalarAbs(rope2_axis_tmp);
  if (absxk > 3.3121686421112381E-170) {
    t3 = 1.0;
    b_scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    t3 = b_t * b_t;
  }

  absxk = muDoubleScalarAbs(n_bar_tmp[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    c_scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    b_y = b_t * b_t;
  }

  x_tmp = py - params_p_a1[1];
  n_par[1] = x_tmp;
  p[1] = b_rope2_axis_tmp;
  absxk = muDoubleScalarAbs(x_tmp);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }

  absxk = muDoubleScalarAbs(b_rope2_axis_tmp);
  if (absxk > b_scale) {
    b_t = b_scale / absxk;
    t3 = t3 * b_t * b_t + 1.0;
    b_scale = absxk;
  } else {
    b_t = absxk / b_scale;
    t3 += b_t * b_t;
  }

  absxk = muDoubleScalarAbs(n_bar_tmp[1]);
  if (absxk > c_scale) {
    b_t = c_scale / absxk;
    b_y = b_y * b_t * b_t + 1.0;
    c_scale = absxk;
  } else {
    b_t = absxk / c_scale;
    b_y += b_t * b_t;
  }

  x_tmp = pz - params_p_a1[2];
  n_par[2] = x_tmp;
  p[2] = c_rope2_axis_tmp;
  absxk = muDoubleScalarAbs(x_tmp);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }

  absxk = muDoubleScalarAbs(c_rope2_axis_tmp);
  if (absxk > b_scale) {
    b_t = b_scale / absxk;
    t3 = t3 * b_t * b_t + 1.0;
    b_scale = absxk;
  } else {
    b_t = absxk / b_scale;
    t3 += b_t * b_t;
  }

  absxk = muDoubleScalarAbs(n_bar_tmp[2]);
  if (absxk > c_scale) {
    b_t = c_scale / absxk;
    b_y = b_y * b_t * b_t + 1.0;
    c_scale = absxk;
  } else {
    b_t = absxk / c_scale;
    b_y += b_t * b_t;
  }

  y = scale * muDoubleScalarSqrt(y);
  t3 = b_scale * muDoubleScalarSqrt(t3);
  b_y = c_scale * muDoubleScalarSqrt(b_y);
  rope2_axis[0] = params_m * 0.0;
  rope2_axis[1] = params_m * 0.0;
  rope2_axis[2] = params_m * -params_g;
  t1 = 0.0;
  scale = 3.3121686421112381E-170;
  for (p1 = 0; p1 < 3; p1++) {
    x_tmp = n_par[p1] / y * u1 + p[p1] / t3 * u2;
    p[p1] = x_tmp;
    rope2_axis[p1] = (rope2_axis[p1] + x_tmp) + 0.0 * (n_bar_tmp[p1] / b_y);
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
      n_par[0] = u3[0];
      n_par[1] = u3[1];
      n_par[2] = u3[2];

      /*  make sure is a column vector */
    } else {
      n_par[0] = 0.0;
      n_par[1] = 0.0;
      n_par[2] = 0.0;
    }

    rope2_axis[0] += n_par[0];
    rope2_axis[1] += n_par[1];
    rope2_axis[2] += n_par[2];
  }

  y = 1.0 / params_m;
  x_tmp = x[1] * n_pz_l1;
  b_x[0] = x_tmp;
  b_t = 2.0 * a_tmp;
  pz = muDoubleScalarPower(x[1], 3.0);
  x_tmp_tmp = py2b * py2b;
  absxk = b_t * pz;
  b_y = py2b / (a_tmp * x[1]) - x_tmp_tmp / absxk;
  b_x[3] = px_l1_tmp - b_px_tmp * b_y / (2.0 * px_l1_sinpsi);
  t1 = x[2] * py2b;
  t3 = b_t * x[1] * px_l1_sinpsi;
  b_x[6] = t1 * px_tmp / t3;
  b_x[1] = 0.0;
  b_x[4] = x[1] / params_b;
  b_x[7] = -x[2] / params_b;
  b_x[2] = x[1] * px_l1_tmp;
  b_x[5] = x[1] * pz_tmp * b_y / (2.0 * px_l1_sinpsi) - n_pz_l1;
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
  c_scale = x[4] * x[4];
  py = x[5] * x[5];
  scale = x[3] * x[3];
  b_scale = (((((4.0 * muDoubleScalarPower(x[1], 4.0) * c_scale - 8.0 * pz * x[2]
                 * x[4] * x[5]) + 4.0 * b_a_tmp * a_tmp_tmp * py) - 6.0 *
               b_a_tmp * c_scale * py2b) - 2.0 * b_a_tmp * py * py2b) + 8.0 * x
             [1] * x[2] * x[4] * x[5] * py2b) + 3.0 * c_scale * x_tmp_tmp;
  pz = px_tmp_tmp * pz * px_l1_sinpsi;
  b_rope2_axis_tmp = 16.0 * muDoubleScalarPower(params_b, 4.0) *
    muDoubleScalarPower(x[1], 5.0) * muDoubleScalarPower(px_l1_sinpsi, 3.0);
  c_rope2_axis_tmp = x[4] * py2b;
  b_y = absxk * px_l1_sinpsi;
  t3 = x[3] * py2b;
  t1 = b_t * b_a_tmp * px_l1_sinpsi;
  rope2_axis_tmp = c_a_tmp * c_a_tmp;
  rope2_axis[0] = y * rope2_axis[0] - (((((2.0 * x[4] * n_pz_l1 * x[3] - x[1] *
    scale * px_l1_tmp) - px_tmp * b_scale / pz) - x_tmp_tmp * px_tmp *
    rope2_axis_tmp / b_rope2_axis_tmp) + t3 * pz_tmp * c_a_tmp / t1) +
    c_rope2_axis_tmp * px_tmp * c_a_tmp / b_y);
  rope2_axis[1] = y * rope2_axis[1] - (c_scale - py) / params_b;
  rope2_axis[2] = y * rope2_axis[2] - (((((x_tmp * scale + 2.0 * x[4] * x[3] *
    px_l1_tmp) + pz_tmp * b_scale / pz) + x_tmp_tmp * pz_tmp * rope2_axis_tmp /
    b_rope2_axis_tmp) - c_rope2_axis_tmp * pz_tmp * c_a_tmp / b_y) + t3 * px_tmp
    * c_a_tmp / t1);
  varargout_1[0] = x[3];
  varargout_1[1] = x[4];
  varargout_1[2] = x[5];
  x_tmp = rope2_axis[0];
  t3 = rope2_axis[1];
  t1 = rope2_axis[2];
  for (p1 = 0; p1 < 3; p1++) {
    varargout_1[p1 + 3] = (b_x[p1] * x_tmp + b_x[p1 + 3] * t3) + b_x[p1 + 6] *
      t1;
  }
}

/* End of code generation (integrate_dynamics.c) */
