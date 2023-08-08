/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_mpc_no_constraints.c
 *
 * Code generation for function 'optimize_cpp_mpc_no_constraints'
 *
 */

/* Include files */
#include "optimize_cpp_mpc_no_constraints.h"
#include "compressBounds.h"
#include "computeForwardDifferences.h"
#include "diff.h"
#include "driver.h"
#include "factoryConstruct.h"
#include "factoryConstruct1.h"
#include "factoryConstruct2.h"
#include "integrate_dynamics.h"
#include "optimize_cpp_mpc_no_constraints_data.h"
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_internal_types.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "vecnorm.h"
#include "xcopy.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo emlrtRTEI = { 11,  /* lineNo */
  9,                                   /* colNo */
  "cost_mpc",                          /* fName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/mpc/cost_mpc.m"/* pName */
};

static emlrtMCInfo emlrtMCI = { 10,    /* lineNo */
  9,                                   /* colNo */
  "cost_mpc",                          /* fName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/mpc/cost_mpc.m"/* pName */
};

static emlrtMCInfo b_emlrtMCI = { 8,   /* lineNo */
  9,                                   /* colNo */
  "eval_pos_vel_mpc",                  /* fName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/mpc/eval_pos_vel_mpc.m"/* pName */
};

static emlrtMCInfo c_emlrtMCI = { 37,  /* lineNo */
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

real_T anon(const real_T actual_state[6], const emxArray_real_T *ref_com, const
            emxArray_real_T *Fr_l0, const emxArray_real_T *Fr_r0, int64_T mpc_N,
            const char_T params_int_method[3], real_T params_int_steps, real_T
            params_b, const real_T params_p_a1[3], const real_T params_p_a2[3],
            real_T params_g, real_T params_m, real_T params_w1, real_T params_w2,
            real_T params_mpc_dt, const emxArray_real_T *x)
{
  static const int32_T iv[2] = { 1, 72 };

  static const int32_T iv1[2] = { 1, 66 };

  static const int32_T iv2[2] = { 1, 15 };

  static const int32_T iv3[2] = { 1, 15 };

  static const char_T u[72] = { 'c', 'o', 's', 't', '_', 'm', 'p', 'c', ':', 'w',
    'r', 'o', 'n', 'g', ' ', 'r', 'e', 'f', '_', 'c', 'o', 'm', ' ', 'i', 'n',
    'p', 'u', 't', ' ', 'l', 'e', 'n', 'g', 't', 'h', ':', ' ', 'r', 'e', 'f',
    '_', 'c', 'o', 'm', ' ', 's', 'h', 'o', 'u', 'l', 'd', ' ', 'b', 'e', ' ',
    'l', 'o', 'n', 'g', 'e', 'r', ' ', 't', 'h', 'a', 'n', ' ', 'm', 'p', 'c',
    '_', 'N' };

  static const char_T b_u[66] = { 'e', 'v', 'a', 'l', '_', 'p', 'o', 's', '_',
    'm', 'p', 'c', ':', 'w', 'r', 'o', 'n', 'g', ' ', 'i', 'n', 'p', 'u', 't',
    ' ', 'l', 'e', 'n', 'g', 't', 'h', ':', ' ', 'i', 'n', 'p', 'u', 't', ' ',
    's', 'h', 'o', 'u', 'l', 'd', ' ', 'b', 'e', ' ', 'l', 'o', 'n', 'g', 'e',
    'r', ' ', 't', 'h', 'a', 'n', ' ', 'm', 'p', 'c', '_', 'N' };

  static const char_T c_u[15] = { 'U', 'n', 'k', 'n', 'o', 'w', 'n', ' ', 'm',
    'e', 't', 'h', 'o', 'd', '.' };

  static const char_T b[3] = { 'r', 'k', '4' };

  emxArray_real_T *b_p;
  emxArray_real_T *b_ref_com;
  emxArray_real_T *b_states_rough;
  emxArray_real_T *delta_Fr_l;
  emxArray_real_T *delta_Fr_r;
  emxArray_real_T *pdx;
  emxArray_real_T *pdy;
  emxArray_real_T *px;
  emxArray_real_T *py;
  emxArray_real_T *states_rough;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *m;
  const mxArray *y;
  int64_T b_i;
  real_T b_unusedU0[6];
  real_T dv[6];
  real_T k_1[6];
  real_T k_2[6];
  real_T k_3[6];
  real_T unusedU0[6];
  real_T a_tmp;
  real_T b_px;
  real_T b_py;
  real_T d;
  real_T d1;
  real_T dt_step;
  real_T varargout_1;
  real_T *pData;
  int32_T extra_forces_size[2];
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T nx;
  boolean_T guard1 = false;
  boolean_T p;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);

  /*  init for cpp */
  varargout_1 = 0.0;
  if (ref_com->size[1] == 0) {
    nx = 0;
  } else {
    nx = muIntScalarMax_sint32(3, ref_com->size[1]);
  }

  p = false;
  if (mpc_N >= 4503599627370496L) {
    p = true;
  } else {
    if (mpc_N > -4503599627370496L) {
      p = ((real_T)nx < mpc_N);
    }
  }

  if (p) {
    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 72, m, &u[0]);
    emlrtAssign(&y, m);
    disp(y, &emlrtMCI);
    b_y = NULL;
    extra_forces_size[0] = ref_com->size[0];
    extra_forces_size[1] = ref_com->size[1];
    m = emlrtCreateNumericArray(2, &extra_forces_size[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    i = 0;
    for (nx = 0; nx < ref_com->size[1]; nx++) {
      pData[i] = ref_com->data[3 * nx];
      i++;
      pData[i] = ref_com->data[3 * nx + 1];
      i++;
      pData[i] = ref_com->data[3 * nx + 2];
      i++;
    }

    emlrtAssign(&b_y, m);
    emlrtDisplayR2012b(b_y, "ref_com", &emlrtRTEI, emlrtRootTLSGlobal);
  } else {
    if (1L > mpc_N) {
      loop_ub = 0;
    } else {
      loop_ub = (int32_T)mpc_N;
    }

    emxInit_real_T(&delta_Fr_l, 2, true);
    i = delta_Fr_l->size[0] * delta_Fr_l->size[1];
    delta_Fr_l->size[0] = 1;
    delta_Fr_l->size[1] = loop_ub;
    emxEnsureCapacity_real_T(delta_Fr_l, i);
    for (i = 0; i < loop_ub; i++) {
      delta_Fr_l->data[i] = x->data[i];
    }

    b_i = mpc_N << 1;
    if (mpc_N + 1L > b_i) {
      i = 0;
      i1 = 0;
    } else {
      i = (int32_T)(mpc_N + 1L) - 1;
      i1 = (int32_T)b_i;
    }

    emxInit_real_T(&delta_Fr_r, 2, true);
    nx = delta_Fr_r->size[0] * delta_Fr_r->size[1];
    delta_Fr_r->size[0] = 1;
    loop_ub = i1 - i;
    delta_Fr_r->size[1] = loop_ub;
    emxEnsureCapacity_real_T(delta_Fr_r, nx);
    for (i1 = 0; i1 < loop_ub; i1++) {
      delta_Fr_r->data[i1] = x->data[i + i1];
    }

    /*  compute actual state */
    if (1L > mpc_N) {
      loop_ub = 0;
    } else {
      loop_ub = (int32_T)mpc_N;
    }

    emxInit_real_T(&b_p, 2, true);

    /* init values for cpp */
    i = b_p->size[0] * b_p->size[1];
    b_p->size[0] = 3;
    b_p->size[1] = (int32_T)mpc_N;
    emxEnsureCapacity_real_T(b_p, i);
    b_loop_ub = 3 * (int32_T)mpc_N;
    for (i = 0; i < b_loop_ub; i++) {
      b_p->data[i] = 0.0;
    }

    emxInit_real_T(&states_rough, 2, true);
    emxInit_real_T(&px, 2, true);
    emxInit_real_T(&py, 2, true);
    emxInit_real_T(&pdx, 2, true);
    emxInit_real_T(&pdy, 2, true);
    emxInit_real_T(&b_states_rough, 2, true);
    nx = Fr_l0->size[1];
    p = false;
    if (mpc_N >= 4503599627370496L) {
      p = true;
    } else {
      if (mpc_N > -4503599627370496L) {
        p = ((real_T)nx < mpc_N);
      }
    }

    guard1 = false;
    if (p) {
      guard1 = true;
    } else {
      nx = Fr_r0->size[1];
      p = false;
      if (mpc_N >= 4503599627370496L) {
        p = true;
      } else {
        if (mpc_N > -4503599627370496L) {
          p = ((real_T)nx < mpc_N);
        }
      }

      if (p) {
        guard1 = true;
      } else {
        /*  check vectors are row and extract first mpc_N elements */
        if (1L > mpc_N) {
          b_loop_ub = 0;
          nx = 0;
        } else {
          b_loop_ub = (int32_T)mpc_N;
          nx = (int32_T)mpc_N;
        }

        /*  single shooting */
        i = px->size[0] * px->size[1];
        px->size[0] = 1;
        px->size[1] = b_loop_ub;
        emxEnsureCapacity_real_T(px, i);
        for (i = 0; i < b_loop_ub; i++) {
          px->data[i] = Fr_l0->data[i] + delta_Fr_l->data[i];
        }

        i = py->size[0] * py->size[1];
        py->size[0] = 1;
        py->size[1] = nx;
        emxEnsureCapacity_real_T(py, i);
        for (i = 0; i < nx; i++) {
          py->data[i] = Fr_r0->data[i] + delta_Fr_r->data[i];
        }

        /* init */
        i = states_rough->size[0] * states_rough->size[1];
        states_rough->size[0] = 6;
        states_rough->size[1] = (int32_T)mpc_N;
        emxEnsureCapacity_real_T(states_rough, i);
        b_loop_ub = 6 * (int32_T)mpc_N;
        for (i = 0; i < b_loop_ub; i++) {
          states_rough->data[i] = 0.0;
        }

        if (params_int_steps == 0.0) {
          /* verify is a column vector */
          i = states_rough->size[0] * states_rough->size[1];
          states_rough->size[0] = 6;
          states_rough->size[1] = 1;
          emxEnsureCapacity_real_T(states_rough, i);
          for (nx = 0; nx < 6; nx++) {
            d = actual_state[nx];
            unusedU0[nx] = d;
            states_rough->data[nx] = d;
          }

          if (memcmp(&params_int_method[0], &b[0], 3) == 0) {
            /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/ */
            /*  we have  time invariant dynamics so t wont count */
            b_i = 1L;
            while (b_i <= mpc_N - 1L) {
              b_px = px->data[(int32_T)b_i - 1];
              b_py = py->data[(int32_T)b_i - 1];
              b_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                     unusedU0, b_px, b_py, k_1);
              a_tmp = 0.5 * params_mpc_dt;
              for (i = 0; i < 6; i++) {
                b_unusedU0[i] = unusedU0[i] + a_tmp * k_1[i];
              }

              b_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                     b_unusedU0, b_px, b_py, k_2);
              for (i = 0; i < 6; i++) {
                b_unusedU0[i] = unusedU0[i] + a_tmp * k_2[i];
              }

              b_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                     b_unusedU0, b_px, b_py, k_3);
              for (i = 0; i < 6; i++) {
                b_unusedU0[i] = unusedU0[i] + k_3[i] * params_mpc_dt;
              }

              b_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                     b_unusedU0, b_px, b_py, dv);
              for (i = 0; i < 6; i++) {
                unusedU0[i] += 0.16666666666666666 * (((k_1[i] + 2.0 * k_2[i]) +
                  2.0 * k_3[i]) + dv[i]) * params_mpc_dt;
              }

              i = b_states_rough->size[0] * b_states_rough->size[1];
              b_states_rough->size[0] = 6;
              b_states_rough->size[1] = states_rough->size[1] + 1;
              emxEnsureCapacity_real_T(b_states_rough, i);
              b_loop_ub = states_rough->size[1];
              for (i = 0; i < b_loop_ub; i++) {
                for (i1 = 0; i1 < 6; i1++) {
                  nx = i1 + 6 * i;
                  b_states_rough->data[nx] = states_rough->data[nx];
                }
              }

              for (i = 0; i < 6; i++) {
                b_states_rough->data[i + 6 * states_rough->size[1]] = unusedU0[i];
              }

              i = states_rough->size[0] * states_rough->size[1];
              states_rough->size[0] = 6;
              states_rough->size[1] = b_states_rough->size[1];
              emxEnsureCapacity_real_T(states_rough, i);
              b_loop_ub = b_states_rough->size[0] * b_states_rough->size[1];
              for (i = 0; i < b_loop_ub; i++) {
                states_rough->data[i] = b_states_rough->data[i];
              }

              b_i++;
              if (*emlrtBreakCheckR2012bFlagVar != 0) {
                emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
              }
            }
          } else {
            d_y = NULL;
            m = emlrtCreateCharArray(2, &iv2[0]);
            emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &c_u[0]);
            emlrtAssign(&d_y, m);
            disp(d_y, &c_emlrtMCI);
          }
        } else {
          dt_step = params_mpc_dt / (params_int_steps - 1.0);
          b_i = 1L;
          while (b_i <= mpc_N) {
            if (b_i >= 2L) {
              i = pdy->size[0] * pdy->size[1];
              pdy->size[0] = 1;
              pdy->size[1] = (int32_T)params_int_steps;
              emxEnsureCapacity_real_T(pdy, i);
              b_px = px->data[(int32_T)(b_i - 1L) - 1];
              b_loop_ub = (int32_T)params_int_steps;
              i = pdx->size[0] * pdx->size[1];
              pdx->size[0] = 1;
              pdx->size[1] = (int32_T)params_int_steps;
              emxEnsureCapacity_real_T(pdx, i);
              b_py = py->data[(int32_T)(b_i - 1L) - 1];
              for (i = 0; i < b_loop_ub; i++) {
                pdy->data[i] = b_px;
                pdx->data[i] = b_py;
              }

              for (i = 0; i < 6; i++) {
                dv[i] = states_rough->data[i + 6 * ((int32_T)(b_i - 1L) - 1)];
              }

              /* verify is a column vector */
              if (memcmp(&params_int_method[0], &b[0], 3) == 0) {
                /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/ */
                /*  we have  time invariant dynamics so t wont count */
                i = (int32_T)(params_int_steps - 1.0);
                for (nx = 0; nx < i; nx++) {
                  d = pdy->data[nx];
                  d1 = pdx->data[nx];
                  b_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                         dv, d, d1, k_1);
                  a_tmp = 0.5 * dt_step;
                  for (i1 = 0; i1 < 6; i1++) {
                    b_unusedU0[i1] = dv[i1] + a_tmp * k_1[i1];
                  }

                  b_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                         b_unusedU0, d, d1, k_2);
                  for (i1 = 0; i1 < 6; i1++) {
                    b_unusedU0[i1] = dv[i1] + a_tmp * k_2[i1];
                  }

                  b_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                         b_unusedU0, d, d1, k_3);
                  for (i1 = 0; i1 < 6; i1++) {
                    b_unusedU0[i1] = dv[i1] + k_3[i1] * dt_step;
                  }

                  b_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                         b_unusedU0, d, d1, unusedU0);
                  for (i1 = 0; i1 < 6; i1++) {
                    dv[i1] += 0.16666666666666666 * (((k_1[i1] + 2.0 * k_2[i1])
                      + 2.0 * k_3[i1]) + unusedU0[i1]) * dt_step;
                  }

                  if (*emlrtBreakCheckR2012bFlagVar != 0) {
                    emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
                  }
                }
              } else {
                e_y = NULL;
                m = emlrtCreateCharArray(2, &iv3[0]);
                emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &c_u[0]);
                emlrtAssign(&e_y, m);
                disp(e_y, &c_emlrtMCI);
              }

              for (i = 0; i < 6; i++) {
                states_rough->data[i + 6 * ((int32_T)b_i - 1)] = dv[i];
              }

              /*  keep Fr constant            */
            } else {
              for (i = 0; i < 6; i++) {
                states_rough->data[i + 6 * ((int32_T)b_i - 1)] = actual_state[i];
              }
            }

            b_i++;
            if (*emlrtBreakCheckR2012bFlagVar != 0) {
              emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
            }
          }
        }

        b_loop_ub = states_rough->size[1];
        i = px->size[0] * px->size[1];
        px->size[0] = 1;
        px->size[1] = states_rough->size[1];
        emxEnsureCapacity_real_T(px, i);
        for (i = 0; i < b_loop_ub; i++) {
          px->data[i] = 0.0;
        }

        b_loop_ub = states_rough->size[1];
        i = py->size[0] * py->size[1];
        py->size[0] = 1;
        py->size[1] = states_rough->size[1];
        emxEnsureCapacity_real_T(py, i);
        for (i = 0; i < b_loop_ub; i++) {
          py->data[i] = 0.0;
        }

        b_loop_ub = states_rough->size[1];
        i = pdy->size[0] * pdy->size[1];
        pdy->size[0] = 1;
        pdy->size[1] = states_rough->size[1];
        emxEnsureCapacity_real_T(pdy, i);
        for (i = 0; i < b_loop_ub; i++) {
          pdy->data[i] = 0.0;
        }

        i = states_rough->size[1] - 1;
        for (nx = 0; nx <= i; nx++) {
          d = states_rough->data[6 * nx];
          d1 = states_rough->data[6 * nx + 1];
          b_px = states_rough->data[6 * nx + 2];
          b_py = params_b * params_b;
          dt_step = d1 * d1;
          a_tmp = (b_py + dt_step) - b_px * b_px;
          b_px = muDoubleScalarSqrt(1.0 - a_tmp * a_tmp / (4.0 * b_py * dt_step));
          px->data[nx] = d1 * muDoubleScalarSin(d) * b_px;
          py->data[nx] = a_tmp / (2.0 * params_b);
          pdy->data[nx] = -d1 * muDoubleScalarCos(d) * b_px;
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
          }
        }

        i = b_p->size[0] * b_p->size[1];
        b_p->size[0] = 3;
        b_p->size[1] = px->size[1];
        emxEnsureCapacity_real_T(b_p, i);
        b_loop_ub = px->size[1];
        for (i = 0; i < b_loop_ub; i++) {
          b_p->data[3 * i] = px->data[i];
        }

        b_loop_ub = py->size[1];
        for (i = 0; i < b_loop_ub; i++) {
          b_p->data[3 * i + 1] = py->data[i];
        }

        b_loop_ub = pdy->size[1];
        for (i = 0; i < b_loop_ub; i++) {
          b_p->data[3 * i + 2] = pdy->data[i];
        }
      }
    }

    if (guard1) {
      c_y = NULL;
      m = emlrtCreateCharArray(2, &iv1[0]);
      emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 66, m, &b_u[0]);
      emlrtAssign(&c_y, m);
      disp(c_y, &b_emlrtMCI);
    }

    emxFree_real_T(&b_states_rough);
    emxFree_real_T(&pdx);
    emxFree_real_T(&py);
    emxFree_real_T(&states_rough);
    emxInit_real_T(&b_ref_com, 2, true);

    /* p has mpc_N +1 elements  */
    /*  cartesian track */
    /* 1 column /2 row wise */
    /*  %state tracking (is worse than cart tracking) */
    /*  tracking_state = 0.; */
    /*  for i=1:mpc_N */
    /*      state_ref = computeStateFromCartesian(params, ref_com_mpc(:,i)); */
    /*      state = computeStateFromCartesian(params, p(:,i)); */
    /*      tracking_state = tracking_state + norm(state_ref(2:3) -state(2:3));%consider only l1 l2 */
    /*  end */
    /*  smoothnes: minimize jerky control action */
    /*  this creates tracking errors sum(delta_Fr_l.^2) + sum(delta_Fr_r.^2); */
    i = b_ref_com->size[0] * b_ref_com->size[1];
    b_ref_com->size[0] = 3;
    b_ref_com->size[1] = loop_ub;
    emxEnsureCapacity_real_T(b_ref_com, i);
    for (nx = 0; nx < loop_ub; nx++) {
      b_ref_com->data[3 * nx] = ref_com->data[3 * nx] - b_p->data[3 * nx];
      i = 3 * nx + 1;
      b_ref_com->data[i] = ref_com->data[3 * nx + 1] - b_p->data[i];
      i = 3 * nx + 2;
      b_ref_com->data[i] = ref_com->data[3 * nx + 2] - b_p->data[i];
    }

    emxFree_real_T(&b_p);
    vecnorm(b_ref_com, px);
    i = pdy->size[0] * pdy->size[1];
    pdy->size[0] = 1;
    pdy->size[1] = px->size[1];
    emxEnsureCapacity_real_T(pdy, i);
    nx = px->size[1];
    emxFree_real_T(&b_ref_com);
    for (b_loop_ub = 0; b_loop_ub < nx; b_loop_ub++) {
      d = px->data[b_loop_ub];
      pdy->data[b_loop_ub] = d * d;
    }

    nx = pdy->size[1];
    if (pdy->size[1] == 0) {
      b_py = 0.0;
    } else {
      b_py = pdy->data[0];
      for (b_loop_ub = 2; b_loop_ub <= nx; b_loop_ub++) {
        b_py += pdy->data[b_loop_ub - 1];
      }
    }

    diff(delta_Fr_l, px);
    i = pdy->size[0] * pdy->size[1];
    pdy->size[0] = 1;
    pdy->size[1] = px->size[1];
    emxEnsureCapacity_real_T(pdy, i);
    nx = px->size[1];
    emxFree_real_T(&delta_Fr_l);
    for (b_loop_ub = 0; b_loop_ub < nx; b_loop_ub++) {
      d = px->data[b_loop_ub];
      pdy->data[b_loop_ub] = d * d;
    }

    nx = pdy->size[1];
    if (pdy->size[1] == 0) {
      dt_step = 0.0;
    } else {
      dt_step = pdy->data[0];
      for (b_loop_ub = 2; b_loop_ub <= nx; b_loop_ub++) {
        dt_step += pdy->data[b_loop_ub - 1];
      }
    }

    diff(delta_Fr_r, px);
    i = pdy->size[0] * pdy->size[1];
    pdy->size[0] = 1;
    pdy->size[1] = px->size[1];
    emxEnsureCapacity_real_T(pdy, i);
    nx = px->size[1];
    emxFree_real_T(&delta_Fr_r);
    for (b_loop_ub = 0; b_loop_ub < nx; b_loop_ub++) {
      d = px->data[b_loop_ub];
      pdy->data[b_loop_ub] = d * d;
    }

    emxFree_real_T(&px);
    nx = pdy->size[1];
    if (pdy->size[1] == 0) {
      b_px = 0.0;
    } else {
      b_px = pdy->data[0];
      for (b_loop_ub = 2; b_loop_ub <= nx; b_loop_ub++) {
        b_px += pdy->data[b_loop_ub - 1];
      }
    }

    emxFree_real_T(&pdy);
    varargout_1 = params_w1 * b_py + params_w2 * (dt_step + b_px);

    /*  + params.w3* terminal_cost; % creates issues in gazebo */
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return varargout_1;
}

void optimize_cpp_mpc_no_constraints(const real_T actual_state[6], real_T
  actual_t, const emxArray_real_T *ref_com, const emxArray_real_T *Fr_l0, const
  emxArray_real_T *Fr_r0, real_T Fr_max, int64_T mpc_N, const param *params,
  emxArray_real_T *x, real_T *EXITFLAG, real_T *final_cost)
{
  c_struct_T memspace;
  d_struct_T TrialState;
  e_struct_T FcnEvaluator;
  emxArray_int32_T *indexFixed;
  emxArray_int32_T *indexLB;
  emxArray_int32_T *indexUB;
  emxArray_real_T *Hessian;
  emxArray_real_T *lb;
  emxArray_real_T *ub;
  emxArray_real_T *x0;
  f_struct_T FiniteDifferences;
  g_struct_T QRManager;
  h_struct_T CholManager;
  i_struct_T QPObjective;
  j_struct_T WorkingSet;
  k_struct_T MeritFunction;
  real_T d;
  real_T fval;
  int32_T b_i;
  int32_T i;
  int32_T mConstrMax;
  int32_T mLB;
  int32_T mUB;
  int32_T maxDims;
  int32_T nVar;
  int32_T nVarMax;
  boolean_T hasLB;
  boolean_T hasUB;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&x0, 2, true);

  /*  , delta_Fr_l0, delta_Fr_r0)   */
  /*          if nargin <9 */
  /*              delta_Fr_l0 = 0*ones(1,mpc_N); */
  /*              delta_Fr_r0 = 0*ones(1,mpc_N); */
  /*          end */
  i = x0->size[0] * x0->size[1];
  x0->size[0] = 1;
  nVar = (int32_T)mpc_N + (int32_T)mpc_N;
  x0->size[1] = nVar;
  emxEnsureCapacity_real_T(x0, i);
  nVarMax = (int32_T)mpc_N;
  for (i = 0; i < nVarMax; i++) {
    x0->data[i] = 0.0;
  }

  nVarMax = (int32_T)mpc_N;
  for (i = 0; i < nVarMax; i++) {
    x0->data[i + (int32_T)mpc_N] = 0.0;
  }

  emxInit_real_T(&lb, 2, true);

  /* opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r */
  i = lb->size[0] * lb->size[1];
  lb->size[0] = 1;
  lb->size[1] = nVar;
  emxEnsureCapacity_real_T(lb, i);
  nVarMax = (int32_T)mpc_N;
  for (i = 0; i < nVarMax; i++) {
    lb->data[i] = -Fr_max;
  }

  nVarMax = (int32_T)mpc_N;
  for (i = 0; i < nVarMax; i++) {
    lb->data[i + (int32_T)mpc_N] = -Fr_max;
  }

  emxInit_real_T(&ub, 2, true);
  i = ub->size[0] * ub->size[1];
  ub->size[0] = 1;
  ub->size[1] = nVar;
  emxEnsureCapacity_real_T(ub, i);
  nVarMax = (int32_T)mpc_N;
  for (i = 0; i < nVarMax; i++) {
    ub->data[i] = Fr_max;
  }

  nVarMax = (int32_T)mpc_N;
  for (i = 0; i < nVarMax; i++) {
    ub->data[i + (int32_T)mpc_N] = Fr_max;
  }

  emxInit_real_T(&Hessian, 2, true);

  /*  % does not always satisfy bounds */
  mLB = lb->size[1];
  mUB = ub->size[1];
  nVar = x0->size[1];
  mConstrMax = (lb->size[1] + ub->size[1]) + 1;
  nVarMax = x0->size[1] + 1;
  maxDims = muIntScalarMax_sint32(nVarMax, mConstrMax);
  i = Hessian->size[0] * Hessian->size[1];
  Hessian->size[0] = x0->size[1];
  Hessian->size[1] = x0->size[1];
  emxEnsureCapacity_real_T(Hessian, i);
  nVarMax = x0->size[1] * x0->size[1];
  for (i = 0; i < nVarMax; i++) {
    Hessian->data[i] = 0.0;
  }

  if (x0->size[1] > 0) {
    for (nVarMax = 0; nVarMax < nVar; nVarMax++) {
      Hessian->data[nVarMax + Hessian->size[0] * nVarMax] = 1.0;
    }
  }

  emxInitStruct_struct_T(&TrialState, true);
  emxInitStruct_struct_T1(&FcnEvaluator, true);
  factoryConstruct(x0->size[1] + 1, mConstrMax, x0, &TrialState);
  xcopy(x0->size[1], x0, TrialState.xstarsqp);
  FcnEvaluator.nVar = x0->size[1];
  for (b_i = 0; b_i < 6; b_i++) {
    FcnEvaluator.objfun.tunableEnvironment.f1[b_i] = actual_state[b_i];
  }

  FcnEvaluator.objfun.tunableEnvironment.f2 = actual_t;
  i = FcnEvaluator.objfun.tunableEnvironment.f3->size[0] *
    FcnEvaluator.objfun.tunableEnvironment.f3->size[1];
  FcnEvaluator.objfun.tunableEnvironment.f3->size[0] = 3;
  FcnEvaluator.objfun.tunableEnvironment.f3->size[1] = ref_com->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.objfun.tunableEnvironment.f3, i);
  nVarMax = ref_com->size[0] * ref_com->size[1];
  for (i = 0; i < nVarMax; i++) {
    FcnEvaluator.objfun.tunableEnvironment.f3->data[i] = ref_com->data[i];
  }

  i = FcnEvaluator.objfun.tunableEnvironment.f4->size[0] *
    FcnEvaluator.objfun.tunableEnvironment.f4->size[1];
  FcnEvaluator.objfun.tunableEnvironment.f4->size[0] = 1;
  FcnEvaluator.objfun.tunableEnvironment.f4->size[1] = Fr_l0->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.objfun.tunableEnvironment.f4, i);
  nVarMax = Fr_l0->size[0] * Fr_l0->size[1];
  for (i = 0; i < nVarMax; i++) {
    FcnEvaluator.objfun.tunableEnvironment.f4->data[i] = Fr_l0->data[i];
  }

  i = FcnEvaluator.objfun.tunableEnvironment.f5->size[0] *
    FcnEvaluator.objfun.tunableEnvironment.f5->size[1];
  FcnEvaluator.objfun.tunableEnvironment.f5->size[0] = 1;
  FcnEvaluator.objfun.tunableEnvironment.f5->size[1] = Fr_r0->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.objfun.tunableEnvironment.f5, i);
  nVarMax = Fr_r0->size[0] * Fr_r0->size[1];
  for (i = 0; i < nVarMax; i++) {
    FcnEvaluator.objfun.tunableEnvironment.f5->data[i] = Fr_r0->data[i];
  }

  emxInitStruct_struct_T2(&FiniteDifferences, true);
  emxInitStruct_struct_T3(&QRManager, true);
  FcnEvaluator.objfun.tunableEnvironment.f6 = mpc_N;
  FcnEvaluator.objfun.tunableEnvironment.f7 = *params;
  FcnEvaluator.mCineq = 0;
  FcnEvaluator.mCeq = 0;
  FcnEvaluator.NonFiniteSupport = true;
  FcnEvaluator.SpecifyObjectiveGradient = false;
  FcnEvaluator.SpecifyConstraintGradient = false;
  FcnEvaluator.ScaleProblem = false;
  b_factoryConstruct(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, mpc_N,
                     params, x0->size[1], lb, ub, &FiniteDifferences);
  QRManager.ldq = maxDims;
  i = QRManager.QR->size[0] * QRManager.QR->size[1];
  QRManager.QR->size[0] = maxDims;
  QRManager.QR->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.QR, i);
  i = QRManager.Q->size[0] * QRManager.Q->size[1];
  QRManager.Q->size[0] = maxDims;
  QRManager.Q->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.Q, i);
  nVarMax = maxDims * maxDims;
  for (i = 0; i < nVarMax; i++) {
    QRManager.Q->data[i] = 0.0;
  }

  i = QRManager.jpvt->size[0];
  QRManager.jpvt->size[0] = maxDims;
  emxEnsureCapacity_int32_T(QRManager.jpvt, i);
  for (i = 0; i < maxDims; i++) {
    QRManager.jpvt->data[i] = 0;
  }

  emxInitStruct_struct_T4(&CholManager, true);
  emxInitStruct_struct_T5(&QPObjective, true);
  emxInitStruct_struct_T6(&memspace, true);
  emxInit_int32_T(&indexLB, 1, true);
  emxInit_int32_T(&indexUB, 1, true);
  emxInit_int32_T(&indexFixed, 1, true);
  emxInitStruct_struct_T7(&WorkingSet, true);
  QRManager.mrows = 0;
  QRManager.ncols = 0;
  i = QRManager.tau->size[0];
  QRManager.tau->size[0] = muIntScalarMin_sint32(maxDims, maxDims);
  emxEnsureCapacity_real_T(QRManager.tau, i);
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  i = CholManager.FMat->size[0] * CholManager.FMat->size[1];
  CholManager.FMat->size[0] = maxDims;
  CholManager.FMat->size[1] = maxDims;
  emxEnsureCapacity_real_T(CholManager.FMat, i);
  CholManager.ldm = maxDims;
  CholManager.ndims = 0;
  CholManager.info = 0;
  i = QPObjective.grad->size[0];
  QPObjective.grad->size[0] = x0->size[1] + 1;
  emxEnsureCapacity_real_T(QPObjective.grad, i);
  i = QPObjective.Hx->size[0];
  QPObjective.Hx->size[0] = x0->size[1];
  emxEnsureCapacity_real_T(QPObjective.Hx, i);
  QPObjective.maxVar = x0->size[1] + 1;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.nvar = x0->size[1];
  QPObjective.hasLinear = true;
  QPObjective.objtype = 3;
  i = memspace.workspace_double->size[0] * memspace.workspace_double->size[1];
  memspace.workspace_double->size[0] = maxDims;
  if (2 < x0->size[1] + 1) {
    memspace.workspace_double->size[1] = x0->size[1] + 1;
  } else {
    memspace.workspace_double->size[1] = 2;
  }

  emxEnsureCapacity_real_T(memspace.workspace_double, i);
  i = memspace.workspace_int->size[0];
  memspace.workspace_int->size[0] = maxDims;
  emxEnsureCapacity_int32_T(memspace.workspace_int, i);
  i = memspace.workspace_sort->size[0];
  memspace.workspace_sort->size[0] = maxDims;
  emxEnsureCapacity_int32_T(memspace.workspace_sort, i);
  i = indexLB->size[0];
  indexLB->size[0] = lb->size[1];
  emxEnsureCapacity_int32_T(indexLB, i);
  i = indexUB->size[0];
  indexUB->size[0] = ub->size[1];
  emxEnsureCapacity_int32_T(indexUB, i);
  i = indexFixed->size[0];
  indexFixed->size[0] = muIntScalarMin_sint32(mLB, mUB);
  emxEnsureCapacity_int32_T(indexFixed, i);
  compressBounds(x0->size[1], indexLB, indexUB, indexFixed, lb, ub, &mLB, &mUB,
                 &nVarMax);
  c_factoryConstruct(mLB, indexLB, mUB, indexUB, nVarMax, indexFixed, x0->size[1],
                     x0->size[1] + 1, mConstrMax, &WorkingSet);
  emxFree_int32_T(&indexFixed);
  emxFree_int32_T(&indexUB);
  emxFree_int32_T(&indexLB);
  emxFree_real_T(&x0);
  if (lb->size[1] != 0) {
    for (nVar = 0; nVar < mLB; nVar++) {
      TrialState.xstarsqp->data[WorkingSet.indexLB->data[nVar] - 1] =
        muDoubleScalarMax(TrialState.xstarsqp->data[WorkingSet.indexLB->
                          data[nVar] - 1], lb->data[WorkingSet.indexLB->
                          data[nVar] - 1]);
    }
  }

  if (ub->size[1] != 0) {
    for (nVar = 0; nVar < mUB; nVar++) {
      TrialState.xstarsqp->data[WorkingSet.indexUB->data[nVar] - 1] =
        muDoubleScalarMin(TrialState.xstarsqp->data[WorkingSet.indexUB->
                          data[nVar] - 1], ub->data[WorkingSet.indexUB->
                          data[nVar] - 1]);
    }

    for (nVar = 0; nVar < nVarMax; nVar++) {
      TrialState.xstarsqp->data[WorkingSet.indexFixed->data[nVar] - 1] =
        ub->data[WorkingSet.indexFixed->data[nVar] - 1];
    }
  }

  fval = anon(actual_state, ref_com, Fr_l0, Fr_r0, mpc_N, params->int_method,
              params->int_steps, params->b, params->p_a1, params->p_a2,
              params->g, params->m, params->w1, params->w2, params->mpc_dt,
              TrialState.xstarsqp);
  TrialState.sqpFval = fval;
  computeForwardDifferences(&FiniteDifferences, fval, TrialState.xstarsqp,
    TrialState.grad, lb, ub);
  TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
  hasLB = (lb->size[1] != 0);
  hasUB = (ub->size[1] != 0);
  if (hasLB) {
    for (nVar = 0; nVar < mLB; nVar++) {
      WorkingSet.lb->data[WorkingSet.indexLB->data[nVar] - 1] = -lb->
        data[WorkingSet.indexLB->data[nVar] - 1];
    }
  }

  if (hasUB) {
    for (nVar = 0; nVar < mUB; nVar++) {
      WorkingSet.ub->data[WorkingSet.indexUB->data[nVar] - 1] = ub->
        data[WorkingSet.indexUB->data[nVar] - 1];
    }
  }

  if (hasLB && hasUB) {
    for (nVar = 0; nVar < nVarMax; nVar++) {
      d = ub->data[WorkingSet.indexFixed->data[nVar] - 1];
      WorkingSet.ub->data[WorkingSet.indexFixed->data[nVar] - 1] = d;
      WorkingSet.bwset->data[nVar] = d;
    }
  }

  setProblemType(&WorkingSet, 3);
  nVarMax = WorkingSet.isActiveIdx[2];
  i = WorkingSet.mConstrMax;
  for (nVar = nVarMax; nVar <= i; nVar++) {
    WorkingSet.isActiveConstr->data[nVar - 1] = false;
  }

  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
  nVarMax = WorkingSet.sizes[0];
  for (maxDims = 0; maxDims < nVarMax; maxDims++) {
    WorkingSet.Wid->data[maxDims] = 1;
    WorkingSet.Wlocalidx->data[maxDims] = maxDims + 1;
    WorkingSet.isActiveConstr->data[maxDims] = true;
    i = WorkingSet.indexFixed->data[maxDims];
    for (b_i = 0; b_i <= i - 2; b_i++) {
      WorkingSet.ATwset->data[b_i + WorkingSet.ATwset->size[0] * maxDims] = 0.0;
    }

    WorkingSet.ATwset->data[(WorkingSet.indexFixed->data[maxDims] +
      WorkingSet.ATwset->size[0] * maxDims) - 1] = 1.0;
    i = WorkingSet.indexFixed->data[maxDims] + 1;
    nVar = WorkingSet.nVar;
    for (b_i = i; b_i <= nVar; b_i++) {
      WorkingSet.ATwset->data[(b_i + WorkingSet.ATwset->size[0] * maxDims) - 1] =
        0.0;
    }

    WorkingSet.bwset->data[maxDims] = WorkingSet.ub->data
      [WorkingSet.indexFixed->data[maxDims] - 1];
  }

  MeritFunction.initFval = fval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  MeritFunction.initConstrViolationEq = 0.0;
  MeritFunction.initConstrViolationIneq = 0.0;
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  MeritFunction.hasObjective = true;
  driver(Hessian, lb, ub, &TrialState, &MeritFunction, &FcnEvaluator,
         &FiniteDifferences, &memspace, &WorkingSet, &QRManager, &CholManager,
         &QPObjective);
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = TrialState.xstarsqp->size[1];
  emxEnsureCapacity_real_T(x, i);
  nVarMax = TrialState.xstarsqp->size[0] * TrialState.xstarsqp->size[1];
  emxFree_real_T(&Hessian);
  emxFreeStruct_struct_T7(&WorkingSet);
  emxFreeStruct_struct_T6(&memspace);
  emxFreeStruct_struct_T5(&QPObjective);
  emxFreeStruct_struct_T4(&CholManager);
  emxFreeStruct_struct_T3(&QRManager);
  emxFreeStruct_struct_T2(&FiniteDifferences);
  emxFreeStruct_struct_T1(&FcnEvaluator);
  emxFree_real_T(&ub);
  emxFree_real_T(&lb);
  for (i = 0; i < nVarMax; i++) {
    x->data[i] = TrialState.xstarsqp->data[i];
  }

  /* ,  @(x) constraints_mpc(x, actual_com, ref_com, Fr_l0, Fr_r0 ) , options); */
  *EXITFLAG = TrialState.sqpExitFlag;
  *final_cost = TrialState.sqpFval;
  emxFreeStruct_struct_T(&TrialState);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (optimize_cpp_mpc_no_constraints.c) */
