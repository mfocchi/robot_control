/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * dynamics.c
 *
 * Code generation for function 'dynamics'
 *
 */

/* Include files */
#include "dynamics.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void dynamics(real_T t, const real_T x[6], real_T Fr_l, real_T Fr_r,
              const real_T Fleg[3], real_T params_m, real_T params_b,
              const real_T params_p_a1[3], const real_T params_p_a2[3],
              real_T params_g, real_T params_T_th, real_T dxdt[6])
{
  __m128d r;
  __m128d r1;
  real_T b_x[9];
  real_T c_x[9];
  real_T n_bar_tmp[3];
  real_T n_par[3];
  real_T p[3];
  real_T rope2_axis[3];
  real_T a;
  real_T a_tmp;
  real_T a_tmp_tmp;
  real_T absxk;
  real_T b_a_tmp;
  real_T b_px_tmp;
  real_T b_scale;
  real_T b_t;
  real_T b_x_tmp;
  real_T b_y;
  real_T c_a_tmp;
  real_T c_scale;
  real_T c_y;
  real_T n_pz_l1;
  real_T px_l1_sinpsi;
  real_T px_l1_tmp;
  real_T px_tmp;
  real_T px_tmp_tmp;
  real_T py;
  real_T py2b;
  real_T pz_tmp;
  real_T rope2_axis_tmp;
  real_T scale;
  real_T t1;
  real_T t2;
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
  t1 = muDoubleScalarSqrt(1.0 - c_a_tmp * c_a_tmp / (px_tmp_tmp * b_a_tmp));
  px_tmp = muDoubleScalarSin(x[0]);
  b_px_tmp = x[1] * px_tmp;
  t2 = b_px_tmp * t1;
  py = c_a_tmp / (2.0 * params_b);
  pz_tmp = muDoubleScalarCos(x[0]);
  t1 *= -x[1] * pz_tmp;
  px_l1_tmp = t2 / x[1];
  n_pz_l1 = -t1 / x[1];
  px_l1_sinpsi = px_l1_tmp / px_tmp;
  py2b = py * 2.0 * params_b;
  /*  mass equation and rope constraints  */
  c_a_tmp = ((x[4] * a_tmp - x[4] * b_a_tmp) + 2.0 * x[5] * x[1] * x[2]) -
            x[4] * a_tmp_tmp;
  p[0] = t2;
  p[1] = py;
  p[2] = t1;
  /*  extra force is applied perpendicularly to the rope plane (TODO */
  /*  make it generic) */
  scale = 3.3121686421112381E-170;
  absxk = params_p_a1[0] - params_p_a2[0];
  n_par[0] = absxk;
  absxk = muDoubleScalarAbs(absxk);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    y = b_t * b_t;
  }
  absxk = params_p_a1[1] - params_p_a2[1];
  n_par[1] = absxk;
  absxk = muDoubleScalarAbs(absxk);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }
  absxk = params_p_a1[2] - params_p_a2[2];
  n_par[2] = absxk;
  absxk = muDoubleScalarAbs(absxk);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }
  y = scale * muDoubleScalarSqrt(y);
  r = _mm_loadu_pd(&n_par[0]);
  _mm_storeu_pd(&n_par[0], _mm_div_pd(r, _mm_set1_pd(y)));
  r = _mm_loadu_pd(&p[0]);
  _mm_storeu_pd(&rope2_axis[0],
                _mm_div_pd(_mm_sub_pd(r, _mm_loadu_pd(&params_p_a2[0])),
                           _mm_set1_pd(x[2])));
  n_par[2] /= y;
  rope2_axis_tmp = t1 - params_p_a2[2];
  rope2_axis[2] = rope2_axis_tmp / x[2];
  n_bar_tmp[0] = n_par[1] * rope2_axis[2] - rope2_axis[1] * n_par[2];
  n_bar_tmp[1] = rope2_axis[0] * n_par[2] - n_par[0] * rope2_axis[2];
  n_bar_tmp[2] = n_par[0] * rope2_axis[1] - rope2_axis[0] * n_par[1];
  scale = 3.3121686421112381E-170;
  b_scale = 3.3121686421112381E-170;
  c_scale = 3.3121686421112381E-170;
  absxk = t2 - params_p_a1[0];
  n_par[0] = absxk;
  t2 -= params_p_a2[0];
  p[0] = t2;
  absxk = muDoubleScalarAbs(absxk);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    y = b_t * b_t;
  }
  absxk = muDoubleScalarAbs(t2);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    b_scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    b_y = b_t * b_t;
  }
  absxk = muDoubleScalarAbs(n_bar_tmp[0]);
  if (absxk > 3.3121686421112381E-170) {
    c_y = 1.0;
    c_scale = absxk;
  } else {
    b_t = absxk / 3.3121686421112381E-170;
    c_y = b_t * b_t;
  }
  absxk = py - params_p_a1[1];
  n_par[1] = absxk;
  t2 = py - params_p_a2[1];
  p[1] = t2;
  absxk = muDoubleScalarAbs(absxk);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }
  absxk = muDoubleScalarAbs(t2);
  if (absxk > b_scale) {
    b_t = b_scale / absxk;
    b_y = b_y * b_t * b_t + 1.0;
    b_scale = absxk;
  } else {
    b_t = absxk / b_scale;
    b_y += b_t * b_t;
  }
  absxk = muDoubleScalarAbs(n_bar_tmp[1]);
  if (absxk > c_scale) {
    b_t = c_scale / absxk;
    c_y = c_y * b_t * b_t + 1.0;
    c_scale = absxk;
  } else {
    b_t = absxk / c_scale;
    c_y += b_t * b_t;
  }
  absxk = t1 - params_p_a1[2];
  n_par[2] = absxk;
  p[2] = rope2_axis_tmp;
  absxk = muDoubleScalarAbs(absxk);
  if (absxk > scale) {
    b_t = scale / absxk;
    y = y * b_t * b_t + 1.0;
    scale = absxk;
  } else {
    b_t = absxk / scale;
    y += b_t * b_t;
  }
  absxk = muDoubleScalarAbs(rope2_axis_tmp);
  if (absxk > b_scale) {
    b_t = b_scale / absxk;
    b_y = b_y * b_t * b_t + 1.0;
    b_scale = absxk;
  } else {
    b_t = absxk / b_scale;
    b_y += b_t * b_t;
  }
  absxk = muDoubleScalarAbs(n_bar_tmp[2]);
  if (absxk > c_scale) {
    b_t = c_scale / absxk;
    c_y = c_y * b_t * b_t + 1.0;
    c_scale = absxk;
  } else {
    b_t = absxk / c_scale;
    c_y += b_t * b_t;
  }
  y = scale * muDoubleScalarSqrt(y);
  b_y = b_scale * muDoubleScalarSqrt(b_y);
  c_y = c_scale * muDoubleScalarSqrt(c_y);
  rope2_axis[0] = params_m * 0.0;
  rope2_axis[1] = params_m * 0.0;
  rope2_axis[2] = params_m * -params_g;
  t1 = 0.0;
  scale = 3.3121686421112381E-170;
  for (p1 = 0; p1 < 3; p1++) {
    absxk = n_par[p1] / y * Fr_l + p[p1] / b_y * Fr_r;
    p[p1] = absxk;
    rope2_axis[p1] = (rope2_axis[p1] + absxk) + 0.0 * (n_bar_tmp[p1] / c_y);
    absxk = muDoubleScalarAbs(Fleg[p1]);
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
      n_par[0] = Fleg[0];
      n_par[1] = Fleg[1];
      n_par[2] = Fleg[2];
      /*  make sure is a column vector */
    } else {
      n_par[0] = 0.0;
      n_par[1] = 0.0;
      n_par[2] = 0.0;
    }
    r = _mm_loadu_pd(&rope2_axis[0]);
    r1 = _mm_loadu_pd(&n_par[0]);
    _mm_storeu_pd(&rope2_axis[0], _mm_add_pd(r, r1));
    rope2_axis[2] += n_par[2];
  }
  a = 1.0 / params_m;
  x_tmp = x[1] * n_pz_l1;
  b_x[0] = x_tmp;
  b_x_tmp = 2.0 * a_tmp;
  b_t = muDoubleScalarPower(x[1], 3.0);
  x_tmp_tmp = py2b * py2b;
  y = b_x_tmp * b_t;
  absxk = py2b / (a_tmp * x[1]) - x_tmp_tmp / y;
  b_x[3] = px_l1_tmp - b_px_tmp * absxk / (2.0 * px_l1_sinpsi);
  t1 = x[2] * py2b;
  t2 = b_x_tmp * x[1] * px_l1_sinpsi;
  b_x[6] = t1 * px_tmp / t2;
  b_x[1] = 0.0;
  b_x[4] = x[1] / params_b;
  b_x[7] = -x[2] / params_b;
  b_x[2] = x[1] * px_l1_tmp;
  b_x[5] = x[1] * pz_tmp * absxk / (2.0 * px_l1_sinpsi) - n_pz_l1;
  b_x[8] = -(t1 * pz_tmp) / t2;
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
  t1 = (c_x[1] * c_x[5] - c_x[2]) / c_x[8];
  t2 = -(c_x[1] + c_x[7] * t1) / c_x[4];
  b_x[p1] = ((1.0 - c_x[3] * t2) - c_x[6] * t1) / c_x[0];
  b_x[p1 + 1] = t2;
  b_x[p1 + 2] = t1;
  t1 = -c_x[5] / c_x[8];
  t2 = (1.0 - c_x[7] * t1) / c_x[4];
  b_x[p2] = -(c_x[3] * t2 + c_x[6] * t1) / c_x[0];
  b_x[p2 + 1] = t2;
  b_x[p2 + 2] = t1;
  t1 = 1.0 / c_x[8];
  t2 = -c_x[7] * t1 / c_x[4];
  b_x[p3] = -(c_x[3] * t2 + c_x[6] * t1) / c_x[0];
  b_x[p3 + 1] = t2;
  b_x[p3 + 2] = t1;
  c_y = x[4] * x[4];
  py = x[5] * x[5];
  scale = x[3] * x[3];
  b_y = (((((4.0 * muDoubleScalarPower(x[1], 4.0) * c_y -
             8.0 * b_t * x[2] * x[4] * x[5]) +
            4.0 * b_a_tmp * a_tmp_tmp * py) -
           6.0 * b_a_tmp * c_y * py2b) -
          2.0 * b_a_tmp * py * py2b) +
         8.0 * x[1] * x[2] * x[4] * x[5] * py2b) +
        3.0 * c_y * x_tmp_tmp;
  b_t = px_tmp_tmp * b_t * px_l1_sinpsi;
  rope2_axis_tmp = c_a_tmp * c_a_tmp;
  b_scale = 16.0 * muDoubleScalarPower(params_b, 4.0) *
            muDoubleScalarPower(x[1], 5.0) *
            muDoubleScalarPower(px_l1_sinpsi, 3.0);
  c_scale = x[4] * py2b;
  absxk = y * px_l1_sinpsi;
  t2 = x[3] * py2b;
  t1 = b_x_tmp * b_a_tmp * px_l1_sinpsi;
  p[0] = ((((2.0 * x[4] * n_pz_l1 * x[3] - x[1] * scale * px_l1_tmp) -
            px_tmp * b_y / b_t) -
           x_tmp_tmp * px_tmp * rope2_axis_tmp / b_scale) +
          t2 * pz_tmp * c_a_tmp / t1) +
         c_scale * px_tmp * c_a_tmp / absxk;
  p[1] = (c_y - py) / params_b;
  p[2] =
      ((((x_tmp * scale + 2.0 * x[4] * x[3] * px_l1_tmp) + pz_tmp * b_y / b_t) +
        x_tmp_tmp * pz_tmp * rope2_axis_tmp / b_scale) -
       c_scale * pz_tmp * c_a_tmp / absxk) +
      t2 * px_tmp * c_a_tmp / t1;
  r = _mm_loadu_pd(&rope2_axis[0]);
  r1 = _mm_loadu_pd(&p[0]);
  _mm_storeu_pd(&rope2_axis[0], _mm_sub_pd(_mm_mul_pd(_mm_set1_pd(a), r), r1));
  rope2_axis[2] = a * rope2_axis[2] - p[2];
  dxdt[0] = x[3];
  dxdt[1] = x[4];
  dxdt[2] = x[5];
  absxk = rope2_axis[0];
  t2 = rope2_axis[1];
  t1 = rope2_axis[2];
  r = _mm_loadu_pd(&b_x[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(absxk));
  r1 = _mm_loadu_pd(&b_x[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(t2));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&b_x[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(t1));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&dxdt[3], r);
  dxdt[5] = (b_x[2] * absxk + b_x[5] * t2) + b_x[8] * t1;
}

/* End of code generation (dynamics.c) */
