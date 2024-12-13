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
#include "dynamics.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_mexutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
real_T integrate_dynamics(real_T x0[6], real_T dt, real_T n_steps,
                          const emxArray_real_T *Fr_l,
                          const emxArray_real_T *Fr_r, const real_T Fleg[3],
                          const char_T method[3], real_T params_m,
                          real_T params_b, const real_T params_p_a1[3],
                          const real_T params_p_a2[3], real_T params_g,
                          real_T params_T_th, emxArray_real_T *x_vec,
                          emxArray_real_T *t_vec)
{
  static const int32_T b_iv[2] = {1, 15};
  static const char_T b[3] = {'e', 'u', 'l'};
  static const char_T b_b[3] = {'r', 'k', '4'};
  const mxArray *m;
  const mxArray *y;
  real_T b_x0[6];
  real_T dv[6];
  real_T k_1[6];
  real_T k_2[6];
  real_T k_3[6];
  const real_T *Fr_l_data;
  const real_T *Fr_r_data;
  real_T t_;
  real_T *x_vec_data;
  int32_T b_i;
  int32_T i;
  Fr_r_data = Fr_r->data;
  Fr_l_data = Fr_l->data;
  /* verify is a column vector */
  t_ = 0.0;
  i = x_vec->size[0] * x_vec->size[1];
  x_vec->size[0] = 6;
  x_vec->size[1] = 1;
  emxEnsureCapacity_real_T(x_vec, i);
  x_vec_data = x_vec->data;
  for (i = 0; i < 6; i++) {
    x_vec_data[i] = x0[i];
  }
  i = t_vec->size[0] * t_vec->size[1];
  t_vec->size[0] = 1;
  t_vec->size[1] = 1;
  emxEnsureCapacity_real_T(t_vec, i);
  x_vec_data = t_vec->data;
  x_vec_data[0] = 0.0;
  if (memcmp((char_T *)&method[0], (char_T *)&b[0], 3) == 0) {
    /*  forwatd euler */
    i = (int32_T)(n_steps - 1.0);
    for (b_i = 0; b_i < i; b_i++) {
      __m128d r;
      __m128d r1;
      __m128d r2;
      int32_T i1;
      int32_T i2;
      dynamics(t_, x0, Fr_l_data[b_i], Fr_r_data[b_i], Fleg, params_m, params_b,
               params_p_a1, params_p_a2, params_g, params_T_th, dv);
      /*  we have time invariant dynamics so t wont count */
      t_ += dt;
      i1 = x_vec->size[1];
      i2 = x_vec->size[0] * x_vec->size[1];
      x_vec->size[0] = 6;
      x_vec->size[1]++;
      emxEnsureCapacity_real_T(x_vec, i2);
      x_vec_data = x_vec->data;
      r = _mm_loadu_pd(&dv[0]);
      r1 = _mm_loadu_pd(&x0[0]);
      r2 = _mm_set1_pd(dt);
      r = _mm_add_pd(r1, _mm_mul_pd(r2, r));
      _mm_storeu_pd(&x0[0], r);
      _mm_storeu_pd(&x_vec_data[6 * i1], r);
      r = _mm_loadu_pd(&dv[2]);
      r1 = _mm_loadu_pd(&x0[2]);
      r = _mm_add_pd(r1, _mm_mul_pd(r2, r));
      _mm_storeu_pd(&x0[2], r);
      _mm_storeu_pd(&x_vec_data[6 * i1 + 2], r);
      r = _mm_loadu_pd(&dv[4]);
      r1 = _mm_loadu_pd(&x0[4]);
      r = _mm_add_pd(r1, _mm_mul_pd(r2, r));
      _mm_storeu_pd(&x0[4], r);
      _mm_storeu_pd(&x_vec_data[6 * i1 + 4], r);
      i1 = t_vec->size[1];
      i2 = t_vec->size[0] * t_vec->size[1];
      t_vec->size[1]++;
      emxEnsureCapacity_real_T(t_vec, i2);
      x_vec_data = t_vec->data;
      x_vec_data[i1] = t_;
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else if (memcmp((char_T *)&method[0], (char_T *)&b_b[0], 3) == 0) {
    /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/
     */
    /*  we have  time invariant dynamics so t wont count */
    i = (int32_T)(n_steps - 1.0);
    for (b_i = 0; b_i < i; b_i++) {
      __m128d r;
      __m128d r1;
      __m128d r2;
      __m128d r3;
      __m128d r4;
      __m128d r5;
      __m128d r6;
      __m128d r7;
      real_T d;
      real_T d1;
      real_T k_2_tmp;
      int32_T i1;
      int32_T i2;
      d = Fr_l_data[b_i];
      d1 = Fr_r_data[b_i];
      dynamics(t_, x0, d, d1, Fleg, params_m, params_b, params_p_a1,
               params_p_a2, params_g, params_T_th, k_1);
      r = _mm_loadu_pd(&k_1[0]);
      r1 = _mm_loadu_pd(&x0[0]);
      r2 = _mm_set1_pd(0.5 * dt);
      _mm_storeu_pd(&b_x0[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
      r = _mm_loadu_pd(&k_1[2]);
      r1 = _mm_loadu_pd(&x0[2]);
      _mm_storeu_pd(&b_x0[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
      r = _mm_loadu_pd(&k_1[4]);
      r1 = _mm_loadu_pd(&x0[4]);
      _mm_storeu_pd(&b_x0[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
      k_2_tmp = t_ + 0.5 * dt;
      dynamics(k_2_tmp, b_x0, d, d1, Fleg, params_m, params_b, params_p_a1,
               params_p_a2, params_g, params_T_th, k_2);
      r = _mm_loadu_pd(&k_2[0]);
      r1 = _mm_loadu_pd(&x0[0]);
      _mm_storeu_pd(&b_x0[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
      r = _mm_loadu_pd(&k_2[2]);
      r1 = _mm_loadu_pd(&x0[2]);
      _mm_storeu_pd(&b_x0[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
      r = _mm_loadu_pd(&k_2[4]);
      r1 = _mm_loadu_pd(&x0[4]);
      _mm_storeu_pd(&b_x0[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
      dynamics(k_2_tmp, b_x0, d, d1, Fleg, params_m, params_b, params_p_a1,
               params_p_a2, params_g, params_T_th, k_3);
      r = _mm_loadu_pd(&k_3[0]);
      r1 = _mm_loadu_pd(&x0[0]);
      r2 = _mm_set1_pd(dt);
      _mm_storeu_pd(&b_x0[0], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&k_3[2]);
      r1 = _mm_loadu_pd(&x0[2]);
      _mm_storeu_pd(&b_x0[2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      r = _mm_loadu_pd(&k_3[4]);
      r1 = _mm_loadu_pd(&x0[4]);
      _mm_storeu_pd(&b_x0[4], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
      t_ += dt;
      dynamics(t_, b_x0, d, d1, Fleg, params_m, params_b, params_p_a1,
               params_p_a2, params_g, params_T_th, dv);
      i1 = x_vec->size[1];
      i2 = x_vec->size[0] * x_vec->size[1];
      x_vec->size[0] = 6;
      x_vec->size[1]++;
      emxEnsureCapacity_real_T(x_vec, i2);
      x_vec_data = x_vec->data;
      r = _mm_loadu_pd(&k_2[0]);
      r1 = _mm_loadu_pd(&k_1[0]);
      r3 = _mm_loadu_pd(&k_3[0]);
      r4 = _mm_loadu_pd(&dv[0]);
      r5 = _mm_loadu_pd(&x0[0]);
      r6 = _mm_set1_pd(2.0);
      r7 = _mm_set1_pd(0.16666666666666666);
      r = _mm_add_pd(
          r5,
          _mm_mul_pd(
              _mm_mul_pd(
                  r7, _mm_add_pd(_mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                            _mm_mul_pd(r6, r3)),
                                 r4)),
              r2));
      _mm_storeu_pd(&x0[0], r);
      _mm_storeu_pd(&x_vec_data[6 * i1], r);
      r = _mm_loadu_pd(&k_2[2]);
      r1 = _mm_loadu_pd(&k_1[2]);
      r3 = _mm_loadu_pd(&k_3[2]);
      r4 = _mm_loadu_pd(&dv[2]);
      r5 = _mm_loadu_pd(&x0[2]);
      r = _mm_add_pd(
          r5,
          _mm_mul_pd(
              _mm_mul_pd(
                  r7, _mm_add_pd(_mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                            _mm_mul_pd(r6, r3)),
                                 r4)),
              r2));
      _mm_storeu_pd(&x0[2], r);
      _mm_storeu_pd(&x_vec_data[6 * i1 + 2], r);
      r = _mm_loadu_pd(&k_2[4]);
      r1 = _mm_loadu_pd(&k_1[4]);
      r3 = _mm_loadu_pd(&k_3[4]);
      r4 = _mm_loadu_pd(&dv[4]);
      r5 = _mm_loadu_pd(&x0[4]);
      r = _mm_add_pd(
          r5,
          _mm_mul_pd(
              _mm_mul_pd(
                  r7, _mm_add_pd(_mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                            _mm_mul_pd(r6, r3)),
                                 r4)),
              r2));
      _mm_storeu_pd(&x0[4], r);
      _mm_storeu_pd(&x_vec_data[6 * i1 + 4], r);
      i1 = t_vec->size[1];
      i2 = t_vec->size[0] * t_vec->size[1];
      t_vec->size[1]++;
      emxEnsureCapacity_real_T(t_vec, i2);
      x_vec_data = t_vec->data;
      x_vec_data[i1] = t_;
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  } else {
    y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &cv[0]);
    emlrtAssign(&y, m);
    disp(y, &emlrtMCI);
  }
  return t_;
}

/* End of code generation (integrate_dynamics.c) */
