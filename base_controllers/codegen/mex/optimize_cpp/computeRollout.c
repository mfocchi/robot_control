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
#include "integrate_dynamics.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtMCInfo emlrtMCI = {
    33,                   /* lineNo */
    13,                   /* colNo */
    "integrate_dynamics", /* fName */
    "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/"
    "optimal_control_2ropes/integrate_dynamics.m" /* pName */
};

/* Function Declarations */
static void disp(const mxArray *m, emlrtMCInfo *location);

/* Function Definitions */
static void disp(const mxArray *m, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = m;
  emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 0, NULL, 1, &pArray, "disp", true,
                        location);
}

void computeRollout(const real_T x0[6], real_T dt_dyn, real_T N_dyn,
                    const emxArray_real_T *Fr_l, const emxArray_real_T *Fr_r,
                    const real_T Fleg[3], const char_T int_method[3],
                    real_T int_steps, real_T params_m, real_T params_b,
                    const real_T params_p_a1[3], const real_T params_p_a2[3],
                    real_T params_g, real_T params_T_th,
                    emxArray_real_T *states_rough, emxArray_real_T *t_rough)
{
  static const int32_T b_iv[2] = {1, 15};
  static const int32_T iv1[2] = {1, 15};
  static const char_T u[15] = {'U', 'n', 'k', 'n', 'o', 'w', 'n', ' ',
                               'm', 'e', 't', 'h', 'o', 'd', '.'};
  static const char_T b[3] = {'r', 'k', '4'};
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  real_T _1[6];
  real_T a__1[6];
  real_T dv[6];
  real_T k_1[6];
  real_T k_2[6];
  real_T k_3[6];
  const real_T *Fr_l_data;
  const real_T *Fr_r_data;
  real_T *states_rough_data;
  real_T *t_rough_data;
  int32_T b_i;
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
    real_T dt_step;
    /* verify is a column vector */
    dt_step = 0.0;
    i = states_rough->size[0] * states_rough->size[1];
    states_rough->size[0] = 6;
    states_rough->size[1] = 1;
    emxEnsureCapacity_real_T(states_rough, i);
    states_rough_data = states_rough->data;
    for (b_i = 0; b_i < 6; b_i++) {
      a__1[b_i] = x0[b_i];
      states_rough_data[b_i] = x0[b_i];
    }
    i = t_rough->size[0] * t_rough->size[1];
    t_rough->size[0] = 1;
    t_rough->size[1] = 1;
    emxEnsureCapacity_real_T(t_rough, i);
    t_rough_data = t_rough->data;
    t_rough_data[0] = 0.0;
    if (memcmp((char_T *)&int_method[0], (char_T *)&b[0], 3) == 0) {
      /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/
       */
      /*  we have  time invariant dynamics so t wont count */
      i = (int32_T)(N_dyn - 1.0);
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
        d = Fr_l_data[b_i];
        d1 = Fr_r_data[b_i];
        integrate_dynamics_anonFcn1(params_m, params_b, params_p_a1,
                                    params_p_a2, params_g, params_T_th, dt_step,
                                    a__1, d, d1, Fleg, k_1);
        r = _mm_loadu_pd(&k_1[0]);
        r1 = _mm_loadu_pd(&a__1[0]);
        r2 = _mm_set1_pd(0.5 * dt_dyn);
        _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
        r = _mm_loadu_pd(&k_1[2]);
        r1 = _mm_loadu_pd(&a__1[2]);
        _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
        r = _mm_loadu_pd(&k_1[4]);
        r1 = _mm_loadu_pd(&a__1[4]);
        _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
        k_2_tmp = dt_step + 0.5 * dt_dyn;
        integrate_dynamics_anonFcn1(params_m, params_b, params_p_a1,
                                    params_p_a2, params_g, params_T_th, k_2_tmp,
                                    _1, d, d1, Fleg, k_2);
        r = _mm_loadu_pd(&k_2[0]);
        r1 = _mm_loadu_pd(&a__1[0]);
        _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
        r = _mm_loadu_pd(&k_2[2]);
        r1 = _mm_loadu_pd(&a__1[2]);
        _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
        r = _mm_loadu_pd(&k_2[4]);
        r1 = _mm_loadu_pd(&a__1[4]);
        _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
        integrate_dynamics_anonFcn1(params_m, params_b, params_p_a1,
                                    params_p_a2, params_g, params_T_th, k_2_tmp,
                                    _1, d, d1, Fleg, k_3);
        r = _mm_loadu_pd(&k_3[0]);
        r1 = _mm_loadu_pd(&a__1[0]);
        r2 = _mm_set1_pd(dt_dyn);
        _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&k_3[2]);
        r1 = _mm_loadu_pd(&a__1[2]);
        _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        r = _mm_loadu_pd(&k_3[4]);
        r1 = _mm_loadu_pd(&a__1[4]);
        _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
        dt_step += dt_dyn;
        integrate_dynamics_anonFcn1(params_m, params_b, params_p_a1,
                                    params_p_a2, params_g, params_T_th, dt_step,
                                    _1, d, d1, Fleg, dv);
        loop_ub = states_rough->size[1];
        loop_ub_tmp = states_rough->size[0] * states_rough->size[1];
        states_rough->size[0] = 6;
        states_rough->size[1]++;
        emxEnsureCapacity_real_T(states_rough, loop_ub_tmp);
        states_rough_data = states_rough->data;
        r = _mm_loadu_pd(&k_2[0]);
        r1 = _mm_loadu_pd(&k_1[0]);
        r3 = _mm_loadu_pd(&k_3[0]);
        r4 = _mm_loadu_pd(&dv[0]);
        r5 = _mm_loadu_pd(&a__1[0]);
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
        _mm_storeu_pd(&a__1[0], r);
        _mm_storeu_pd(&states_rough_data[6 * loop_ub], r);
        r = _mm_loadu_pd(&k_2[2]);
        r1 = _mm_loadu_pd(&k_1[2]);
        r3 = _mm_loadu_pd(&k_3[2]);
        r4 = _mm_loadu_pd(&dv[2]);
        r5 = _mm_loadu_pd(&a__1[2]);
        r = _mm_add_pd(
            r5,
            _mm_mul_pd(
                _mm_mul_pd(
                    r7, _mm_add_pd(_mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                              _mm_mul_pd(r6, r3)),
                                   r4)),
                r2));
        _mm_storeu_pd(&a__1[2], r);
        _mm_storeu_pd(&states_rough_data[6 * loop_ub + 2], r);
        r = _mm_loadu_pd(&k_2[4]);
        r1 = _mm_loadu_pd(&k_1[4]);
        r3 = _mm_loadu_pd(&k_3[4]);
        r4 = _mm_loadu_pd(&dv[4]);
        r5 = _mm_loadu_pd(&a__1[4]);
        r = _mm_add_pd(
            r5,
            _mm_mul_pd(
                _mm_mul_pd(
                    r7, _mm_add_pd(_mm_add_pd(_mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                              _mm_mul_pd(r6, r3)),
                                   r4)),
                r2));
        _mm_storeu_pd(&a__1[4], r);
        _mm_storeu_pd(&states_rough_data[6 * loop_ub + 4], r);
        loop_ub = t_rough->size[1];
        loop_ub_tmp = t_rough->size[0] * t_rough->size[1];
        t_rough->size[1]++;
        emxEnsureCapacity_real_T(t_rough, loop_ub_tmp);
        t_rough_data = t_rough->data;
        t_rough_data[loop_ub] = dt_step;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
        }
      }
    } else {
      b_y = NULL;
      m = emlrtCreateCharArray(2, &iv1[0]);
      emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &u[0]);
      emlrtAssign(&b_y, m);
      disp(b_y, &emlrtMCI);
    }
  } else {
    real_T dt_step;
    dt_step = dt_dyn / (int_steps - 1.0);
    for (b_i = 0; b_i < loop_ub_tmp; b_i++) {
      if ((uint32_T)b_i + 1U >= 2U) {
        real_T a__1_tmp;
        for (i = 0; i < 6; i++) {
          dv[i] = states_rough_data[i + 6 * (b_i - 1)];
        }
        a__1_tmp = t_rough_data[b_i - 1];
        /* verify is a column vector */
        if (memcmp((char_T *)&int_method[0], (char_T *)&b[0], 3) == 0) {
          /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/
           */
          /*  we have  time invariant dynamics so t wont count */
          i = (int32_T)(int_steps - 1.0);
          for (loop_ub = 0; loop_ub < i; loop_ub++) {
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
            d = Fr_l_data[b_i - 1];
            d1 = Fr_r_data[b_i - 1];
            integrate_dynamics_anonFcn1(params_m, params_b, params_p_a1,
                                        params_p_a2, params_g, params_T_th,
                                        a__1_tmp, dv, d, d1, Fleg, k_1);
            r = _mm_loadu_pd(&k_1[0]);
            r1 = _mm_loadu_pd(&dv[0]);
            r2 = _mm_set1_pd(0.5 * dt_step);
            _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            r = _mm_loadu_pd(&k_1[2]);
            r1 = _mm_loadu_pd(&dv[2]);
            _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            r = _mm_loadu_pd(&k_1[4]);
            r1 = _mm_loadu_pd(&dv[4]);
            _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            k_2_tmp = a__1_tmp + 0.5 * dt_step;
            integrate_dynamics_anonFcn1(params_m, params_b, params_p_a1,
                                        params_p_a2, params_g, params_T_th,
                                        k_2_tmp, _1, d, d1, Fleg, k_2);
            r = _mm_loadu_pd(&k_2[0]);
            r1 = _mm_loadu_pd(&dv[0]);
            _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            r = _mm_loadu_pd(&k_2[2]);
            r1 = _mm_loadu_pd(&dv[2]);
            _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            r = _mm_loadu_pd(&k_2[4]);
            r1 = _mm_loadu_pd(&dv[4]);
            _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r2, r)));
            integrate_dynamics_anonFcn1(params_m, params_b, params_p_a1,
                                        params_p_a2, params_g, params_T_th,
                                        k_2_tmp, _1, d, d1, Fleg, k_3);
            r = _mm_loadu_pd(&k_3[0]);
            r1 = _mm_loadu_pd(&dv[0]);
            r2 = _mm_set1_pd(dt_step);
            _mm_storeu_pd(&_1[0], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
            r = _mm_loadu_pd(&k_3[2]);
            r1 = _mm_loadu_pd(&dv[2]);
            _mm_storeu_pd(&_1[2], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
            r = _mm_loadu_pd(&k_3[4]);
            r1 = _mm_loadu_pd(&dv[4]);
            _mm_storeu_pd(&_1[4], _mm_add_pd(r1, _mm_mul_pd(r, r2)));
            a__1_tmp += dt_step;
            integrate_dynamics_anonFcn1(params_m, params_b, params_p_a1,
                                        params_p_a2, params_g, params_T_th,
                                        a__1_tmp, _1, d, d1, Fleg, a__1);
            r = _mm_loadu_pd(&k_2[0]);
            r1 = _mm_loadu_pd(&k_1[0]);
            r3 = _mm_loadu_pd(&k_3[0]);
            r4 = _mm_loadu_pd(&a__1[0]);
            r5 = _mm_loadu_pd(&dv[0]);
            r6 = _mm_set1_pd(2.0);
            r7 = _mm_set1_pd(0.16666666666666666);
            _mm_storeu_pd(
                &dv[0],
                _mm_add_pd(
                    r5, _mm_mul_pd(
                            _mm_mul_pd(
                                r7, _mm_add_pd(
                                        _mm_add_pd(
                                            _mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                            _mm_mul_pd(r6, r3)),
                                        r4)),
                            r2)));
            r = _mm_loadu_pd(&k_2[2]);
            r1 = _mm_loadu_pd(&k_1[2]);
            r3 = _mm_loadu_pd(&k_3[2]);
            r4 = _mm_loadu_pd(&a__1[2]);
            r5 = _mm_loadu_pd(&dv[2]);
            _mm_storeu_pd(
                &dv[2],
                _mm_add_pd(
                    r5, _mm_mul_pd(
                            _mm_mul_pd(
                                r7, _mm_add_pd(
                                        _mm_add_pd(
                                            _mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                            _mm_mul_pd(r6, r3)),
                                        r4)),
                            r2)));
            r = _mm_loadu_pd(&k_2[4]);
            r1 = _mm_loadu_pd(&k_1[4]);
            r3 = _mm_loadu_pd(&k_3[4]);
            r4 = _mm_loadu_pd(&a__1[4]);
            r5 = _mm_loadu_pd(&dv[4]);
            _mm_storeu_pd(
                &dv[4],
                _mm_add_pd(
                    r5, _mm_mul_pd(
                            _mm_mul_pd(
                                r7, _mm_add_pd(
                                        _mm_add_pd(
                                            _mm_add_pd(r1, _mm_mul_pd(r6, r)),
                                            _mm_mul_pd(r6, r3)),
                                        r4)),
                            r2)));
            if (*emlrtBreakCheckR2012bFlagVar != 0) {
              emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
            }
          }
        } else {
          y = NULL;
          m = emlrtCreateCharArray(2, &b_iv[0]);
          emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &u[0]);
          emlrtAssign(&y, m);
          disp(y, &emlrtMCI);
        }
        for (i = 0; i < 6; i++) {
          states_rough_data[i + 6 * b_i] = dv[i];
        }
        t_rough_data[b_i] = a__1_tmp;
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
