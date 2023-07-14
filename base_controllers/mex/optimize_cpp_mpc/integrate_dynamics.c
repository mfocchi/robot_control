/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
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
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void integrate_dynamics_anonFcn1(real_T params_b, const real_T params_p_a1[3],
                                 const real_T params_p_a2[3], real_T params_g,
                                 real_T params_m, const real_T x[6], real_T u1,
                                 real_T u2, real_T varargout_1[6])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  real_T b_x[9];
  real_T c_x[9];
  real_T J_tmp[3];
  real_T b_params_m[3];
  real_T p[3];
  real_T a;
  real_T a_tmp;
  real_T a_tmp_tmp;
  real_T b_a_tmp;
  real_T b_p_tmp;
  real_T b_px_tmp;
  real_T b_x_tmp;
  real_T b_x_tmp_tmp;
  real_T c_a_tmp;
  real_T c_p_tmp;
  real_T c_x_tmp_tmp;
  real_T n_pz_l1;
  real_T p_tmp;
  real_T px_l1_sinpsi;
  real_T px_l1_tmp;
  real_T px_tmp;
  real_T px_tmp_tmp;
  real_T py2b;
  real_T pz_tmp;
  real_T scale;
  real_T t1;
  real_T t2;
  real_T t3;
  real_T x_tmp;
  real_T x_tmp_tmp;
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
  scale = c_a_tmp / (2.0 * params_b);
  pz_tmp = muDoubleScalarCos(x[0]);
  t3 = -x[1] * pz_tmp * t1;
  px_l1_tmp = t2 / x[1];
  n_pz_l1 = -t3 / x[1];
  px_l1_sinpsi = px_l1_tmp / px_tmp;
  py2b = scale * 2.0 * params_b;
  /*  mass equation and rope constraints  */
  c_a_tmp = ((x[4] * a_tmp - x[4] * b_a_tmp) + 2.0 * x[5] * x[1] * x[2]) -
            x[4] * a_tmp_tmp;
  p[0] = t2;
  p[1] = scale;
  p[2] = t3;
  r = _mm_loadu_pd(&p[0]);
  _mm_storeu_pd(&J_tmp[0], _mm_sub_pd(r, _mm_loadu_pd(&params_p_a1[0])));
  _mm_storeu_pd(&p[0], _mm_sub_pd(r, _mm_loadu_pd(&params_p_a2[0])));
  J_tmp[2] = p[2] - params_p_a1[2];
  p[2] -= params_p_a2[2];
  a = 1.0 / params_m;
  x_tmp = x[1] * n_pz_l1;
  b_x[0] = x_tmp;
  b_x_tmp = 2.0 * a_tmp;
  x_tmp_tmp = muDoubleScalarPower(x[1], 3.0);
  b_x_tmp_tmp = py2b * py2b;
  c_x_tmp_tmp = b_x_tmp * x_tmp_tmp;
  t3 = py2b / (a_tmp * x[1]) - b_x_tmp_tmp / c_x_tmp_tmp;
  b_x[3] = px_l1_tmp - b_px_tmp * t3 / (2.0 * px_l1_sinpsi);
  scale = x[2] * py2b;
  t1 = b_x_tmp * x[1] * px_l1_sinpsi;
  b_x[6] = scale * px_tmp / t1;
  b_x[1] = 0.0;
  b_x[4] = x[1] / params_b;
  b_x[7] = -x[2] / params_b;
  b_x[2] = x[1] * px_l1_tmp;
  b_x[5] = x[1] * pz_tmp * t3 / (2.0 * px_l1_sinpsi) - n_pz_l1;
  b_x[8] = -(scale * pz_tmp) / t1;
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
  t3 = (c_x[1] * c_x[5] - c_x[2]) / c_x[8];
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
  scale = 3.3121686421112381E-170;
  t1 = 3.3121686421112381E-170;
  t3 = muDoubleScalarAbs(J_tmp[0]);
  if (t3 > 3.3121686421112381E-170) {
    a_tmp = 1.0;
    scale = t3;
  } else {
    t2 = t3 / 3.3121686421112381E-170;
    a_tmp = t2 * t2;
  }
  t3 = muDoubleScalarAbs(p[0]);
  if (t3 > 3.3121686421112381E-170) {
    b_px_tmp = 1.0;
    t1 = t3;
  } else {
    t2 = t3 / 3.3121686421112381E-170;
    b_px_tmp = t2 * t2;
  }
  t3 = muDoubleScalarAbs(J_tmp[1]);
  if (t3 > scale) {
    t2 = scale / t3;
    a_tmp = a_tmp * t2 * t2 + 1.0;
    scale = t3;
  } else {
    t2 = t3 / scale;
    a_tmp += t2 * t2;
  }
  t3 = muDoubleScalarAbs(p[1]);
  if (t3 > t1) {
    t2 = t1 / t3;
    b_px_tmp = b_px_tmp * t2 * t2 + 1.0;
    t1 = t3;
  } else {
    t2 = t3 / t1;
    b_px_tmp += t2 * t2;
  }
  t3 = muDoubleScalarAbs(J_tmp[2]);
  if (t3 > scale) {
    t2 = scale / t3;
    a_tmp = a_tmp * t2 * t2 + 1.0;
    scale = t3;
  } else {
    t2 = t3 / scale;
    a_tmp += t2 * t2;
  }
  t3 = muDoubleScalarAbs(p[2]);
  if (t3 > t1) {
    t2 = t1 / t3;
    b_px_tmp = b_px_tmp * t2 * t2 + 1.0;
    t1 = t3;
  } else {
    t2 = t3 / t1;
    b_px_tmp += t2 * t2;
  }
  a_tmp = scale * muDoubleScalarSqrt(a_tmp);
  b_px_tmp = t1 * muDoubleScalarSqrt(b_px_tmp);
  b_params_m[0] = params_m * 0.0;
  b_params_m[1] = params_m * 0.0;
  b_params_m[2] = params_m * -params_g;
  r = _mm_loadu_pd(&J_tmp[0]);
  r = _mm_div_pd(r, _mm_set1_pd(a_tmp));
  r = _mm_mul_pd(r, _mm_set1_pd(u1));
  r1 = _mm_loadu_pd(&p[0]);
  r1 = _mm_div_pd(r1, _mm_set1_pd(b_px_tmp));
  r1 = _mm_mul_pd(r1, _mm_set1_pd(u2));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&J_tmp[0], r);
  J_tmp[2] = J_tmp[2] / a_tmp * u1 + p[2] / b_px_tmp * u2;
  p_tmp = x[4] * x[4];
  b_p_tmp = x[5] * x[5];
  c_p_tmp = x[3] * x[3];
  a_tmp_tmp = (((((4.0 * muDoubleScalarPower(x[1], 4.0) * p_tmp -
                   8.0 * x_tmp_tmp * x[2] * x[4] * x[5]) +
                  4.0 * b_a_tmp * a_tmp_tmp * b_p_tmp) -
                 6.0 * b_a_tmp * p_tmp * py2b) -
                2.0 * b_a_tmp * b_p_tmp * py2b) +
               8.0 * x[1] * x[2] * x[4] * x[5] * py2b) +
              3.0 * p_tmp * b_x_tmp_tmp;
  t2 = px_tmp_tmp * x_tmp_tmp * px_l1_sinpsi;
  a_tmp = c_a_tmp * c_a_tmp;
  b_px_tmp = 16.0 * muDoubleScalarPower(params_b, 4.0) *
             muDoubleScalarPower(x[1], 5.0) *
             muDoubleScalarPower(px_l1_sinpsi, 3.0);
  x_tmp_tmp = x[4] * py2b;
  t3 = c_x_tmp_tmp * px_l1_sinpsi;
  t1 = x[3] * py2b;
  scale = b_x_tmp * b_a_tmp * px_l1_sinpsi;
  p[0] = ((((2.0 * x[4] * n_pz_l1 * x[3] - x[1] * c_p_tmp * px_l1_tmp) -
            px_tmp * a_tmp_tmp / t2) -
           b_x_tmp_tmp * px_tmp * a_tmp / b_px_tmp) +
          t1 * pz_tmp * c_a_tmp / scale) +
         x_tmp_tmp * px_tmp * c_a_tmp / t3;
  p[1] = (p_tmp - b_p_tmp) / params_b;
  p[2] = ((((x_tmp * c_p_tmp + 2.0 * x[4] * x[3] * px_l1_tmp) +
            pz_tmp * a_tmp_tmp / t2) +
           b_x_tmp_tmp * pz_tmp * a_tmp / b_px_tmp) -
          x_tmp_tmp * pz_tmp * c_a_tmp / t3) +
         t1 * px_tmp * c_a_tmp / scale;
  r = _mm_loadu_pd(&b_params_m[0]);
  r1 = _mm_loadu_pd(&J_tmp[0]);
  r2 = _mm_loadu_pd(&p[0]);
  _mm_storeu_pd(&b_params_m[0],
                _mm_sub_pd(_mm_mul_pd(_mm_set1_pd(a), _mm_add_pd(r, r1)), r2));
  b_params_m[2] = a * (b_params_m[2] + J_tmp[2]) - p[2];
  varargout_1[0] = x[3];
  varargout_1[1] = x[4];
  varargout_1[2] = x[5];
  t1 = b_params_m[0];
  t3 = b_params_m[1];
  t2 = b_params_m[2];
  r = _mm_loadu_pd(&b_x[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(t1));
  r1 = _mm_loadu_pd(&b_x[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(t3));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&b_x[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(t2));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&varargout_1[3], r);
  varargout_1[5] = (b_x[2] * t1 + b_x[5] * t3) + b_x[8] * t2;
}

/* End of code generation (integrate_dynamics.c) */
