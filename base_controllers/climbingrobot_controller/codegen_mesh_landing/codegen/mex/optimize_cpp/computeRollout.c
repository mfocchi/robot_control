/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeRollout.c
 *
 * Code generation for function 'computeRollout'
 *
 */

/* Include files */
#include "computeRollout.h"
#include "dynamics.h"
#include "integrate_dynamics.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_mexutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void computeRollout(const real_T x0[6], real_T dt_dyn, real_T N_dyn,
                    const emxArray_real_T *Fr_l, const emxArray_real_T *Fr_r,
                    const real_T Fleg[3], const char_T int_method[3],
                    real_T int_steps, real_T params_m, real_T params_b,
                    const real_T params_p_a1[3], const real_T params_p_a2[3],
                    real_T params_g, real_T params_T_th,
                    emxArray_real_T *states_rough, emxArray_real_T *t_rough)
{
  static const int32_T b_iv[2] = {1, 15};
  static const char_T b[3] = {'e', 'u', 'l'};
  static const char_T b_b[3] = {'r', 'k', '4'};
  const mxArray *m;
  const mxArray *y;
  real_T b_states_rough[6];
  real_T c_states_rough[6];
  real_T k_1[6];
  real_T k_2[6];
  real_T k_3[6];
  const real_T *Fr_l_data;
  const real_T *Fr_r_data;
  real_T *states_rough_data;
  real_T *t_rough_data;
  int32_T b_i;
  int32_T c_i;
  int32_T i;
  int32_T loop_ub;
  int32_T loop_ub_tmp;
  Fr_r_data = Fr_r->data;
  Fr_l_data = Fr_l->data;
  /* init */
  i = states_rough->size[0] * states_rough->size[1];
  states_rough->size[0] = 6;
  loop_ub_tmp = (int32_T)N_dyn;
  states_rough->size[1] = (int32_T)N_dyn;
  emxEnsureCapacity_real_T(states_rough, i);
  states_rough_data = states_rough->data;
  loop_ub = 6 * (int32_T)N_dyn;
  for (i = 0; i < loop_ub; i++) {
    states_rough_data[i] = 0.0;
  }
  i = t_rough->size[0] * t_rough->size[1];
  t_rough->size[0] = 1;
  t_rough->size[1] = (int32_T)N_dyn;
  emxEnsureCapacity_real_T(t_rough, i);
  t_rough_data = t_rough->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    t_rough_data[i] = 0.0;
  }
  if (int_steps == 0.0) {
    for (b_i = 0; b_i < 6; b_i++) {
      k_1[b_i] = x0[b_i];
    }
    integrate_dynamics(k_1, dt_dyn, N_dyn, Fr_l, Fr_r, Fleg, int_method,
                       params_m, params_b, params_p_a1, params_p_a2, params_g,
                       params_T_th, states_rough, t_rough);
  } else {
    real_T dt_step;
    dt_step = dt_dyn / (int_steps - 1.0);
    for (b_i = 0; b_i < loop_ub_tmp; b_i++) {
      if ((uint32_T)b_i + 1U >= 2U) {
        real_T states_rough_tmp;
        /* verify is a column vector */
        states_rough_tmp = t_rough_data[b_i - 1];
        for (i = 0; i < 6; i++) {
          states_rough_data[i + 6 * b_i] = states_rough_data[i + 6 * (b_i - 1)];
        }
        if (memcmp((char_T *)&int_method[0], (char_T *)&b[0], 3) == 0) {
          /*  forwatd euler */
          i = (int32_T)(int_steps - 1.0);
          for (c_i = 0; c_i < i; c_i++) {
            __m128d r;
            __m128d r1;
            __m128d r2;
            dynamics(states_rough_tmp, &states_rough_data[6 * b_i],
                     Fr_l_data[b_i - 1], Fr_r_data[b_i - 1], Fleg, params_m,
                     params_b, params_p_a1, params_p_a2, params_g, params_T_th,
                     c_states_rough);
            r = _mm_loadu_pd(&c_states_rough[0]);
            r1 = _mm_loadu_pd(&states_rough_data[6 * b_i]);
            r2 = _mm_set1_pd(dt_step);
            r = _mm_add_pd(r1, _mm_mul_pd(r2, r));
            _mm_storeu_pd(&c_states_rough[0], r);
            _mm_storeu_pd(&states_rough_data[6 * b_i], r);
            r = _mm_loadu_pd(&c_states_rough[2]);
            loop_ub = 6 * b_i + 2;
            r1 = _mm_loadu_pd(&states_rough_data[loop_ub]);
            r = _mm_add_pd(r1, _mm_mul_pd(r2, r));
            _mm_storeu_pd(&c_states_rough[2], r);
            _mm_storeu_pd(&states_rough_data[loop_ub], r);
            r = _mm_loadu_pd(&c_states_rough[4]);
            loop_ub = 6 * b_i + 4;
            r1 = _mm_loadu_pd(&states_rough_data[loop_ub]);
            r = _mm_add_pd(r1, _mm_mul_pd(r2, r));
            _mm_storeu_pd(&c_states_rough[4], r);
            _mm_storeu_pd(&states_rough_data[loop_ub], r);
            /*  we have time invariant dynamics so t wont count */
            states_rough_tmp += dt_step;
            if (*emlrtBreakCheckR2012bFlagVar != 0) {
              emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
            }
          }
        } else if (memcmp((char_T *)&int_method[0], (char_T *)&b_b[0], 3) ==
                   0) {
          /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/
           */
          /*  we have  time invariant dynamics so t wont count */
          i = (int32_T)(int_steps - 1.0);
          for (c_i = 0; c_i < i; c_i++) {
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
            d = Fr_l_data[b_i - 1];
            d1 = Fr_r_data[b_i - 1];
            dynamics(states_rough_tmp, &states_rough_data[6 * b_i], d, d1, Fleg,
                     params_m, params_b, params_p_a1, params_p_a2, params_g,
                     params_T_th, k_1);
            r = _mm_loadu_pd(&k_1[0]);
            r1 = _mm_loadu_pd(&states_rough_data[6 * b_i]);
            r2 = _mm_set1_pd(0.5 * dt_step);
            _mm_storeu_pd(&b_states_rough[0],
                          _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            r = _mm_loadu_pd(&k_1[2]);
            loop_ub = 6 * b_i + 2;
            r1 = _mm_loadu_pd(&states_rough_data[loop_ub]);
            _mm_storeu_pd(&b_states_rough[2],
                          _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            r = _mm_loadu_pd(&k_1[4]);
            i1 = 6 * b_i + 4;
            r1 = _mm_loadu_pd(&states_rough_data[i1]);
            _mm_storeu_pd(&b_states_rough[4],
                          _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            k_2_tmp = states_rough_tmp + 0.5 * dt_step;
            dynamics(k_2_tmp, b_states_rough, d, d1, Fleg, params_m, params_b,
                     params_p_a1, params_p_a2, params_g, params_T_th, k_2);
            r = _mm_loadu_pd(&k_2[0]);
            r1 = _mm_loadu_pd(&states_rough_data[6 * b_i]);
            _mm_storeu_pd(&b_states_rough[0],
                          _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            r = _mm_loadu_pd(&k_2[2]);
            r1 = _mm_loadu_pd(&states_rough_data[loop_ub]);
            _mm_storeu_pd(&b_states_rough[2],
                          _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            r = _mm_loadu_pd(&k_2[4]);
            r1 = _mm_loadu_pd(&states_rough_data[i1]);
            _mm_storeu_pd(&b_states_rough[4],
                          _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            dynamics(k_2_tmp, b_states_rough, d, d1, Fleg, params_m, params_b,
                     params_p_a1, params_p_a2, params_g, params_T_th, k_3);
            r = _mm_loadu_pd(&k_3[0]);
            r1 = _mm_loadu_pd(&states_rough_data[6 * b_i]);
            r2 = _mm_set1_pd(dt_step);
            _mm_storeu_pd(&b_states_rough[0],
                          _mm_add_pd(r1, _mm_mul_pd(r, r2)));
            r = _mm_loadu_pd(&k_3[2]);
            r1 = _mm_loadu_pd(&states_rough_data[loop_ub]);
            _mm_storeu_pd(&b_states_rough[2],
                          _mm_add_pd(r1, _mm_mul_pd(r, r2)));
            r = _mm_loadu_pd(&k_3[4]);
            r1 = _mm_loadu_pd(&states_rough_data[i1]);
            _mm_storeu_pd(&b_states_rough[4],
                          _mm_add_pd(r1, _mm_mul_pd(r, r2)));
            states_rough_tmp += dt_step;
            dynamics(states_rough_tmp, b_states_rough, d, d1, Fleg, params_m,
                     params_b, params_p_a1, params_p_a2, params_g, params_T_th,
                     c_states_rough);
            r = _mm_loadu_pd(&k_2[0]);
            r1 = _mm_loadu_pd(&k_1[0]);
            r3 = _mm_loadu_pd(&k_3[0]);
            r4 = _mm_loadu_pd(&c_states_rough[0]);
            r5 = _mm_loadu_pd(&states_rough_data[6 * b_i]);
            r6 = _mm_set1_pd(2.0);
            r7 = _mm_set1_pd(0.16666666666666666);
            r = _mm_add_pd(
                r5,
                _mm_mul_pd(
                    _mm_mul_pd(
                        r7,
                        _mm_add_pd(_mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                              _mm_mul_pd(r6, r3)),
                                   r4)),
                    r2));
            _mm_storeu_pd(&k_1[0], r);
            _mm_storeu_pd(&states_rough_data[6 * b_i], r);
            r = _mm_loadu_pd(&k_2[2]);
            r1 = _mm_loadu_pd(&k_1[2]);
            r3 = _mm_loadu_pd(&k_3[2]);
            r4 = _mm_loadu_pd(&c_states_rough[2]);
            r5 = _mm_loadu_pd(&states_rough_data[loop_ub]);
            r = _mm_add_pd(
                r5,
                _mm_mul_pd(
                    _mm_mul_pd(
                        r7,
                        _mm_add_pd(_mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                              _mm_mul_pd(r6, r3)),
                                   r4)),
                    r2));
            _mm_storeu_pd(&k_1[2], r);
            _mm_storeu_pd(&states_rough_data[loop_ub], r);
            r = _mm_loadu_pd(&k_2[4]);
            r1 = _mm_loadu_pd(&k_1[4]);
            r3 = _mm_loadu_pd(&k_3[4]);
            r4 = _mm_loadu_pd(&c_states_rough[4]);
            r5 = _mm_loadu_pd(&states_rough_data[i1]);
            r = _mm_add_pd(
                r5,
                _mm_mul_pd(
                    _mm_mul_pd(
                        r7,
                        _mm_add_pd(_mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                              _mm_mul_pd(r6, r3)),
                                   r4)),
                    r2));
            _mm_storeu_pd(&k_1[4], r);
            _mm_storeu_pd(&states_rough_data[i1], r);
            if (*emlrtBreakCheckR2012bFlagVar != 0) {
              emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
            }
          }
        } else {
          y = NULL;
          m = emlrtCreateCharArray(2, &b_iv[0]);
          emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &cv1[0]);
          emlrtAssign(&y, m);
          disp(y, &b_emlrtMCI);
        }
        t_rough_data[b_i] = states_rough_tmp;
        /*  keep Fr constant            */
      } else {
        for (i = 0; i < 6; i++) {
          states_rough_data[i] = x0[i];
        }
        t_rough_data[0] = 0.0;
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }
  }
}

/* End of code generation (computeRollout.c) */
