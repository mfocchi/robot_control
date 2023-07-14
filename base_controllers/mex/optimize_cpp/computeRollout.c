/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
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
#include <string.h>

/* Variable Definitions */
static emlrtMCInfo emlrtMCI = { 33,    /* lineNo */
  13,                                  /* colNo */
  "integrate_dynamics",                /* fName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/integrate_dynamics.m"/* pName */
};

/* Function Declarations */
static void disp(const mxArray *b, emlrtMCInfo *location);

/* Function Definitions */
static void disp(const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 0, NULL, 1, &pArray, "disp", true,
                        location);
}

void computeRollout(const real_T x0[6], real_T dt_dyn, real_T N_dyn, const
                    emxArray_real_T *Fr_l, const emxArray_real_T *Fr_r, const
                    real_T Fleg[3], const char_T int_method[3], real_T int_steps,
                    real_T params_m, real_T params_b, const real_T params_p_a1[3],
                    const real_T params_p_a2[3], real_T params_g, real_T
                    params_T_th, emxArray_real_T *states_rough, emxArray_real_T *
                    t_rough)
{
  static const int32_T iv[2] = { 1, 15 };

  static const int32_T iv1[2] = { 1, 15 };

  static const char_T u[15] = { 'U', 'n', 'k', 'n', 'o', 'w', 'n', ' ', 'm', 'e',
    't', 'h', 'o', 'd', '.' };

  static const char_T b[3] = { 'r', 'k', '4' };

  emxArray_real_T *b_states_rough;
  emxArray_real_T *b_y;
  emxArray_real_T *y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *m;
  real_T b_unusedU0[6];
  real_T dv[6];
  real_T k_1[6];
  real_T k_2[6];
  real_T k_3[6];
  real_T unusedU0[6];
  real_T a_tmp;
  real_T b_Fr_l;
  real_T b_Fr_r;
  real_T d;
  real_T dt_step;
  real_T k_2_tmp;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T loop_ub;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);

  /* init */
  i = states_rough->size[0] * states_rough->size[1];
  states_rough->size[0] = 6;
  states_rough->size[1] = (int32_T)N_dyn;
  emxEnsureCapacity_real_T(states_rough, i);
  loop_ub = 6 * (int32_T)N_dyn;
  for (i = 0; i < loop_ub; i++) {
    states_rough->data[i] = 0.0;
  }

  i = t_rough->size[0] * t_rough->size[1];
  t_rough->size[0] = 1;
  t_rough->size[1] = (int32_T)N_dyn;
  emxEnsureCapacity_real_T(t_rough, i);
  loop_ub = (int32_T)N_dyn;
  for (i = 0; i < loop_ub; i++) {
    t_rough->data[i] = 0.0;
  }

  if (int_steps == 0.0) {
    /* verify is a column vector */
    b_Fr_l = 0.0;
    i = states_rough->size[0] * states_rough->size[1];
    states_rough->size[0] = 6;
    states_rough->size[1] = 1;
    emxEnsureCapacity_real_T(states_rough, i);
    for (b_i = 0; b_i < 6; b_i++) {
      b_Fr_r = x0[b_i];
      unusedU0[b_i] = b_Fr_r;
      states_rough->data[b_i] = b_Fr_r;
    }

    i = t_rough->size[0] * t_rough->size[1];
    t_rough->size[0] = 1;
    t_rough->size[1] = 1;
    emxEnsureCapacity_real_T(t_rough, i);
    t_rough->data[0] = 0.0;
    if (memcmp(&int_method[0], &b[0], 3) == 0) {
      /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/ */
      /*  we have  time invariant dynamics so t wont count */
      i = (int32_T)(N_dyn - 1.0);
      emxInit_real_T(&b_states_rough, 2, true);
      for (b_i = 0; b_i < i; b_i++) {
        b_Fr_r = Fr_l->data[b_i];
        d = Fr_r->data[b_i];
        b_anon(params_m, params_b, params_p_a1, params_p_a2, params_g,
               params_T_th, b_Fr_l, unusedU0, b_Fr_r, d, Fleg, k_1);
        a_tmp = 0.5 * dt_dyn;
        for (i1 = 0; i1 < 6; i1++) {
          b_unusedU0[i1] = unusedU0[i1] + a_tmp * k_1[i1];
        }

        k_2_tmp = b_Fr_l + 0.5 * dt_dyn;
        b_anon(params_m, params_b, params_p_a1, params_p_a2, params_g,
               params_T_th, k_2_tmp, b_unusedU0, b_Fr_r, d, Fleg, k_2);
        for (i1 = 0; i1 < 6; i1++) {
          b_unusedU0[i1] = unusedU0[i1] + a_tmp * k_2[i1];
        }

        b_anon(params_m, params_b, params_p_a1, params_p_a2, params_g,
               params_T_th, k_2_tmp, b_unusedU0, b_Fr_r, d, Fleg, k_3);
        for (i1 = 0; i1 < 6; i1++) {
          b_unusedU0[i1] = unusedU0[i1] + k_3[i1] * dt_dyn;
        }

        b_Fr_l += dt_dyn;
        b_anon(params_m, params_b, params_p_a1, params_p_a2, params_g,
               params_T_th, b_Fr_l, b_unusedU0, b_Fr_r, d, Fleg, dv);
        for (i1 = 0; i1 < 6; i1++) {
          unusedU0[i1] += 0.16666666666666666 * (((k_1[i1] + 2.0 * k_2[i1]) +
            2.0 * k_3[i1]) + dv[i1]) * dt_dyn;
        }

        i1 = b_states_rough->size[0] * b_states_rough->size[1];
        b_states_rough->size[0] = 6;
        loop_ub = states_rough->size[1];
        b_states_rough->size[1] = states_rough->size[1] + 1;
        emxEnsureCapacity_real_T(b_states_rough, i1);
        for (i1 = 0; i1 < loop_ub; i1++) {
          for (i2 = 0; i2 < 6; i2++) {
            i3 = i2 + 6 * i1;
            b_states_rough->data[i3] = states_rough->data[i3];
          }
        }

        for (i1 = 0; i1 < 6; i1++) {
          b_states_rough->data[i1 + 6 * states_rough->size[1]] = unusedU0[i1];
        }

        i1 = states_rough->size[0] * states_rough->size[1];
        states_rough->size[0] = 6;
        states_rough->size[1] = b_states_rough->size[1];
        emxEnsureCapacity_real_T(states_rough, i1);
        loop_ub = b_states_rough->size[0] * b_states_rough->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          states_rough->data[i1] = b_states_rough->data[i1];
        }

        i1 = t_rough->size[1];
        i2 = t_rough->size[0] * t_rough->size[1];
        t_rough->size[1]++;
        emxEnsureCapacity_real_T(t_rough, i2);
        t_rough->data[i1] = b_Fr_l;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
        }
      }

      emxFree_real_T(&b_states_rough);
    } else {
      c_y = NULL;
      m = emlrtCreateCharArray(2, &iv[0]);
      emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &u[0]);
      emlrtAssign(&c_y, m);
      disp(c_y, &emlrtMCI);
    }
  } else {
    dt_step = dt_dyn / (int_steps - 1.0);
    i = (int32_T)N_dyn;
    emxInit_real_T(&y, 2, true);
    emxInit_real_T(&b_y, 2, true);
    for (b_i = 0; b_i < i; b_i++) {
      if (b_i + 1U >= 2U) {
        i1 = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = (int32_T)int_steps;
        emxEnsureCapacity_real_T(y, i1);
        b_Fr_l = Fr_l->data[b_i - 1];
        loop_ub = (int32_T)int_steps;
        i1 = b_y->size[0] * b_y->size[1];
        b_y->size[0] = 1;
        b_y->size[1] = (int32_T)int_steps;
        emxEnsureCapacity_real_T(b_y, i1);
        b_Fr_r = Fr_r->data[b_i - 1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          y->data[i1] = b_Fr_l;
          b_y->data[i1] = b_Fr_r;
        }

        for (i1 = 0; i1 < 6; i1++) {
          dv[i1] = states_rough->data[i1 + 6 * (b_i - 1)];
        }

        b_Fr_l = t_rough->data[b_i - 1];

        /* verify is a column vector */
        if (memcmp(&int_method[0], &b[0], 3) == 0) {
          /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/ */
          /*  we have  time invariant dynamics so t wont count */
          i1 = (int32_T)(int_steps - 1.0);
          for (loop_ub = 0; loop_ub < i1; loop_ub++) {
            b_Fr_r = y->data[loop_ub];
            d = b_y->data[loop_ub];
            b_anon(params_m, params_b, params_p_a1, params_p_a2, params_g,
                   params_T_th, b_Fr_l, dv, b_Fr_r, d, Fleg, k_1);
            a_tmp = 0.5 * dt_step;
            for (i2 = 0; i2 < 6; i2++) {
              b_unusedU0[i2] = dv[i2] + a_tmp * k_1[i2];
            }

            k_2_tmp = b_Fr_l + 0.5 * dt_step;
            b_anon(params_m, params_b, params_p_a1, params_p_a2, params_g,
                   params_T_th, k_2_tmp, b_unusedU0, b_Fr_r, d, Fleg, k_2);
            for (i2 = 0; i2 < 6; i2++) {
              b_unusedU0[i2] = dv[i2] + a_tmp * k_2[i2];
            }

            b_anon(params_m, params_b, params_p_a1, params_p_a2, params_g,
                   params_T_th, k_2_tmp, b_unusedU0, b_Fr_r, d, Fleg, k_3);
            for (i2 = 0; i2 < 6; i2++) {
              b_unusedU0[i2] = dv[i2] + k_3[i2] * dt_step;
            }

            b_Fr_l += dt_step;
            b_anon(params_m, params_b, params_p_a1, params_p_a2, params_g,
                   params_T_th, b_Fr_l, b_unusedU0, b_Fr_r, d, Fleg, unusedU0);
            for (i2 = 0; i2 < 6; i2++) {
              dv[i2] += 0.16666666666666666 * (((k_1[i2] + 2.0 * k_2[i2]) + 2.0 *
                k_3[i2]) + unusedU0[i2]) * dt_step;
            }

            if (*emlrtBreakCheckR2012bFlagVar != 0) {
              emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
            }
          }
        } else {
          d_y = NULL;
          m = emlrtCreateCharArray(2, &iv1[0]);
          emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &u[0]);
          emlrtAssign(&d_y, m);
          disp(d_y, &emlrtMCI);
        }

        for (i1 = 0; i1 < 6; i1++) {
          states_rough->data[i1 + 6 * b_i] = dv[i1];
        }

        t_rough->data[b_i] = b_Fr_l;

        /*  keep Fr constant            */
      } else {
        for (i1 = 0; i1 < 6; i1++) {
          states_rough->data[i1] = x0[i1];
        }

        t_rough->data[0] = 0.0;
      }

      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
      }
    }

    emxFree_real_T(&b_y);
    emxFree_real_T(&y);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (computeRollout.c) */
