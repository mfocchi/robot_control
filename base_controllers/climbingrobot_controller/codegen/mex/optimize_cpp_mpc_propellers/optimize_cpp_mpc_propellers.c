/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_mpc_propellers.c
 *
 * Code generation for function 'optimize_cpp_mpc_propellers'
 *
 */

/* Include files */
#include "optimize_cpp_mpc_propellers.h"
#include "compressBounds.h"
#include "computeConstraints_.h"
#include "computeForwardDifferences.h"
#include "diff.h"
#include "driver.h"
#include "factoryConstruct.h"
#include "factoryConstruct1.h"
#include "factoryConstruct2.h"
#include "integrate_dynamics.h"
#include "optimize_cpp_mpc_propellers_data.h"
#include "optimize_cpp_mpc_propellers_emxutil.h"
#include "optimize_cpp_mpc_propellers_internal_types.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "updateWorkingSetForNewQP.h"
#include "vecnorm.h"
#include "xcopy.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo emlrtRTEI = { 11,  /* lineNo */
  9,                                   /* colNo */
  "cost_mpc_propellers",               /* fName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/mpc/cost_mpc_propellers.m"/* pName */
};

static emlrtMCInfo emlrtMCI = { 10,    /* lineNo */
  9,                                   /* colNo */
  "cost_mpc_propellers",               /* fName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/2ropes/optimal_control_2ropes/mpc/cost_mpc_propellers.m"/* pName */
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
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u);
static void disp(const mxArray *b, emlrtMCInfo *location);

/* Function Definitions */
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T iv[2];
  int32_T b_i;
  int32_T i;
  y = NULL;
  iv[0] = u->size[0];
  iv[1] = u->size[1];
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < u->size[1]; b_i++) {
    pData[i] = u->data[3 * b_i];
    i++;
    pData[i] = u->data[3 * b_i + 1];
    i++;
    pData[i] = u->data[3 * b_i + 2];
    i++;
  }

  emlrtAssign(&y, m);
  return y;
}

static void disp(const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 0, NULL, 1, &pArray, "disp", true,
                        location);
}

void anon(const emxArray_real_T *Fr_l0, const emxArray_real_T *Fr_r0, int64_T
          mpc_N, const emxArray_real_T *x, emxArray_real_T *varargout_1)
{
  int64_T y;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;

  /*  size  known */
  y = mpc_N << 1;
  i = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[0] = 1;
  varargout_1->size[1] = (int32_T)y;
  emxEnsureCapacity_real_T(varargout_1, i);
  loop_ub = (int32_T)y;
  for (i = 0; i < loop_ub; i++) {
    varargout_1->data[i] = 0.0;
  }

  /*  init for cpp   */
  if (mpc_N + 1L > (mpc_N << 1)) {
    i = 1;
  } else {
    i = (int32_T)(mpc_N + 1L);
  }

  /*  fr + delta F <=0 */
  y = 1L;
  while (y <= mpc_N) {
    loop_ub = varargout_1->size[1];
    i1 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i1);
    varargout_1->data[loop_ub] = Fr_l0->data[(int32_T)y - 1] + x->data[(int32_T)
      y + -1];
    loop_ub = varargout_1->size[1];
    i1 = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, i1);
    varargout_1->data[loop_ub] = Fr_r0->data[(int32_T)y - 1] + x->data[(i +
      (int32_T)y) - 2];
    y++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  /*  this creates issues!!!! and does not make to converge */
  /*      % -Frmax < fr + delta F  => -(fr + delta F) -Frmax < 0  */
  /*      for i=1:mpc_N  */
  /*          ineq = [ineq -(Fr_l0(i) + delta_Fr_l(i)) -Fr_max  ];    */
  /*          ineq = [ineq -(Fr_r0(i) + delta_Fr_r(i)) -Fr_max  ];  */
  /*      end */
  /*       */
}

real_T b_anon(const real_T actual_state[6], const emxArray_real_T *ref_com,
              const emxArray_real_T *Fr_l0, const emxArray_real_T *Fr_r0,
              int64_T mpc_N, const char_T params_int_method[3], real_T
              params_int_steps, real_T params_b, const real_T params_p_a1[3],
              const real_T params_p_a2[3], real_T params_g, real_T params_m,
              real_T params_w1, real_T params_w2, real_T params_mpc_dt, const
              emxArray_real_T *x)
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
  emxArray_real_T *pdz;
  emxArray_real_T *px;
  emxArray_real_T *py;
  emxArray_real_T *states_rough;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *m;
  const mxArray *y;
  int64_T b_i;
  int64_T i3;
  real_T b_unusedU0[6];
  real_T dv[6];
  real_T k_1[6];
  real_T k_2[6];
  real_T k_3[6];
  real_T unusedU0[6];
  real_T a_tmp;
  real_T b_px;
  real_T b_py;
  real_T b_x;
  real_T d;
  real_T dt_step;
  real_T e_y;
  real_T varargout_1;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i4;
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
    emlrtDisplayR2012b(b_emlrt_marshallOut(ref_com), "ref_com", &emlrtRTEI,
                       emlrtRootTLSGlobal);
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
    i2 = delta_Fr_r->size[0] * delta_Fr_r->size[1];
    delta_Fr_r->size[0] = 1;
    loop_ub = i1 - i;
    delta_Fr_r->size[1] = loop_ub;
    emxEnsureCapacity_real_T(delta_Fr_r, i2);
    for (i1 = 0; i1 < loop_ub; i1++) {
      delta_Fr_r->data[i1] = x->data[i + i1];
    }

    i3 = 3L * mpc_N;
    if (b_i + 1L > i3) {
      i = -1;
      i1 = 0;
    } else {
      i = (int32_T)(b_i + 1L) - 2;
      i1 = (int32_T)i3;
    }

    /*  compute actual state */
    if (1L > mpc_N) {
      loop_ub = 0;
    } else {
      loop_ub = (int32_T)mpc_N;
    }

    emxInit_real_T(&b_p, 2, true);

    /* init values for cpp */
    i2 = b_p->size[0] * b_p->size[1];
    b_p->size[0] = 3;
    b_p->size[1] = (int32_T)mpc_N;
    emxEnsureCapacity_real_T(b_p, i2);
    b_loop_ub = 3 * (int32_T)mpc_N;
    for (i2 = 0; i2 < b_loop_ub; i2++) {
      b_p->data[i2] = 0.0;
    }

    emxInit_real_T(&states_rough, 2, true);
    emxInit_real_T(&px, 2, true);
    emxInit_real_T(&py, 2, true);
    emxInit_real_T(&pdx, 2, true);
    emxInit_real_T(&pdy, 2, true);
    emxInit_real_T(&pdz, 2, true);
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
        i2 = px->size[0] * px->size[1];
        px->size[0] = 1;
        px->size[1] = b_loop_ub;
        emxEnsureCapacity_real_T(px, i2);
        for (i2 = 0; i2 < b_loop_ub; i2++) {
          px->data[i2] = Fr_l0->data[i2] + delta_Fr_l->data[i2];
        }

        i2 = py->size[0] * py->size[1];
        py->size[0] = 1;
        py->size[1] = nx;
        emxEnsureCapacity_real_T(py, i2);
        for (i2 = 0; i2 < nx; i2++) {
          py->data[i2] = Fr_r0->data[i2] + delta_Fr_r->data[i2];
        }

        /* init */
        i2 = states_rough->size[0] * states_rough->size[1];
        states_rough->size[0] = 6;
        states_rough->size[1] = (int32_T)mpc_N;
        emxEnsureCapacity_real_T(states_rough, i2);
        b_loop_ub = 6 * (int32_T)mpc_N;
        for (i2 = 0; i2 < b_loop_ub; i2++) {
          states_rough->data[i2] = 0.0;
        }

        if (params_int_steps == 0.0) {
          /* verify is a column vector */
          i2 = states_rough->size[0] * states_rough->size[1];
          states_rough->size[0] = 6;
          states_rough->size[1] = 1;
          emxEnsureCapacity_real_T(states_rough, i2);
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
              b_x = x->data[i + (int32_T)b_i];
              c_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                     unusedU0, b_px, b_py, b_x, k_1);
              a_tmp = 0.5 * params_mpc_dt;
              for (i2 = 0; i2 < 6; i2++) {
                b_unusedU0[i2] = unusedU0[i2] + a_tmp * k_1[i2];
              }

              c_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                     b_unusedU0, b_px, b_py, b_x, k_2);
              for (i2 = 0; i2 < 6; i2++) {
                b_unusedU0[i2] = unusedU0[i2] + a_tmp * k_2[i2];
              }

              c_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                     b_unusedU0, b_px, b_py, b_x, k_3);
              for (i2 = 0; i2 < 6; i2++) {
                b_unusedU0[i2] = unusedU0[i2] + k_3[i2] * params_mpc_dt;
              }

              c_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                     b_unusedU0, b_px, b_py, b_x, dv);
              for (i2 = 0; i2 < 6; i2++) {
                unusedU0[i2] += 0.16666666666666666 * (((k_1[i2] + 2.0 * k_2[i2])
                  + 2.0 * k_3[i2]) + dv[i2]) * params_mpc_dt;
              }

              i2 = b_states_rough->size[0] * b_states_rough->size[1];
              b_states_rough->size[0] = 6;
              b_states_rough->size[1] = states_rough->size[1] + 1;
              emxEnsureCapacity_real_T(b_states_rough, i2);
              b_loop_ub = states_rough->size[1];
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                for (i4 = 0; i4 < 6; i4++) {
                  nx = i4 + 6 * i2;
                  b_states_rough->data[nx] = states_rough->data[nx];
                }
              }

              for (i2 = 0; i2 < 6; i2++) {
                b_states_rough->data[i2 + 6 * states_rough->size[1]] =
                  unusedU0[i2];
              }

              i2 = states_rough->size[0] * states_rough->size[1];
              states_rough->size[0] = 6;
              states_rough->size[1] = b_states_rough->size[1];
              emxEnsureCapacity_real_T(states_rough, i2);
              b_loop_ub = b_states_rough->size[0] * b_states_rough->size[1];
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                states_rough->data[i2] = b_states_rough->data[i2];
              }

              b_i++;
              if (*emlrtBreakCheckR2012bFlagVar != 0) {
                emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
              }
            }
          } else {
            c_y = NULL;
            m = emlrtCreateCharArray(2, &iv2[0]);
            emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &c_u[0]);
            emlrtAssign(&c_y, m);
            disp(c_y, &c_emlrtMCI);
          }
        } else {
          dt_step = params_mpc_dt / (params_int_steps - 1.0);
          b_i = 1L;
          while (b_i <= mpc_N) {
            if (b_i >= 2L) {
              i2 = pdz->size[0] * pdz->size[1];
              pdz->size[0] = 1;
              pdz->size[1] = (int32_T)params_int_steps;
              emxEnsureCapacity_real_T(pdz, i2);
              b_px = px->data[(int32_T)(b_i - 1L) - 1];
              b_loop_ub = (int32_T)params_int_steps;
              i2 = pdx->size[0] * pdx->size[1];
              pdx->size[0] = 1;
              pdx->size[1] = (int32_T)params_int_steps;
              emxEnsureCapacity_real_T(pdx, i2);
              b_py = py->data[(int32_T)(b_i - 1L) - 1];
              i2 = pdy->size[0] * pdy->size[1];
              pdy->size[0] = 1;
              pdy->size[1] = (int32_T)params_int_steps;
              emxEnsureCapacity_real_T(pdy, i2);
              b_x = x->data[i + (int32_T)(b_i - 1L)];
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                pdz->data[i2] = b_px;
                pdx->data[i2] = b_py;
                pdy->data[i2] = b_x;
              }

              for (i2 = 0; i2 < 6; i2++) {
                dv[i2] = states_rough->data[i2 + 6 * ((int32_T)(b_i - 1L) - 1)];
              }

              /* verify is a column vector */
              if (memcmp(&params_int_method[0], &b[0], 3) == 0) {
                /* https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/ */
                /*  we have  time invariant dynamics so t wont count */
                i2 = (int32_T)(params_int_steps - 1.0);
                for (nx = 0; nx < i2; nx++) {
                  d = pdz->data[nx];
                  b_x = pdx->data[nx];
                  e_y = pdy->data[nx];
                  c_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                         dv, d, b_x, e_y, k_1);
                  a_tmp = 0.5 * dt_step;
                  for (i4 = 0; i4 < 6; i4++) {
                    b_unusedU0[i4] = dv[i4] + a_tmp * k_1[i4];
                  }

                  c_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                         b_unusedU0, d, b_x, e_y, k_2);
                  for (i4 = 0; i4 < 6; i4++) {
                    b_unusedU0[i4] = dv[i4] + a_tmp * k_2[i4];
                  }

                  c_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                         b_unusedU0, d, b_x, e_y, k_3);
                  for (i4 = 0; i4 < 6; i4++) {
                    b_unusedU0[i4] = dv[i4] + k_3[i4] * dt_step;
                  }

                  c_anon(params_b, params_p_a1, params_p_a2, params_g, params_m,
                         b_unusedU0, d, b_x, e_y, unusedU0);
                  for (i4 = 0; i4 < 6; i4++) {
                    dv[i4] += 0.16666666666666666 * (((k_1[i4] + 2.0 * k_2[i4])
                      + 2.0 * k_3[i4]) + unusedU0[i4]) * dt_step;
                  }

                  if (*emlrtBreakCheckR2012bFlagVar != 0) {
                    emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
                  }
                }
              } else {
                d_y = NULL;
                m = emlrtCreateCharArray(2, &iv3[0]);
                emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 15, m, &c_u[0]);
                emlrtAssign(&d_y, m);
                disp(d_y, &c_emlrtMCI);
              }

              for (i2 = 0; i2 < 6; i2++) {
                states_rough->data[i2 + 6 * ((int32_T)b_i - 1)] = dv[i2];
              }

              /*  keep Fr constant            */
            } else {
              for (i2 = 0; i2 < 6; i2++) {
                states_rough->data[i2 + 6 * ((int32_T)b_i - 1)] =
                  actual_state[i2];
              }
            }

            b_i++;
            if (*emlrtBreakCheckR2012bFlagVar != 0) {
              emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
            }
          }
        }

        b_loop_ub = states_rough->size[1];
        i2 = px->size[0] * px->size[1];
        px->size[0] = 1;
        px->size[1] = states_rough->size[1];
        emxEnsureCapacity_real_T(px, i2);
        for (i2 = 0; i2 < b_loop_ub; i2++) {
          px->data[i2] = 0.0;
        }

        b_loop_ub = states_rough->size[1];
        i2 = py->size[0] * py->size[1];
        py->size[0] = 1;
        py->size[1] = states_rough->size[1];
        emxEnsureCapacity_real_T(py, i2);
        for (i2 = 0; i2 < b_loop_ub; i2++) {
          py->data[i2] = 0.0;
        }

        b_loop_ub = states_rough->size[1];
        i2 = pdx->size[0] * pdx->size[1];
        pdx->size[0] = 1;
        pdx->size[1] = states_rough->size[1];
        emxEnsureCapacity_real_T(pdx, i2);
        for (i2 = 0; i2 < b_loop_ub; i2++) {
          pdx->data[i2] = 0.0;
        }

        i2 = states_rough->size[1] - 1;
        for (nx = 0; nx <= i2; nx++) {
          d = states_rough->data[6 * nx];
          b_x = states_rough->data[6 * nx + 1];
          e_y = states_rough->data[6 * nx + 2];
          b_px = params_b * params_b;
          b_py = b_x * b_x;
          a_tmp = (b_px + b_py) - e_y * e_y;
          e_y = muDoubleScalarSqrt(1.0 - a_tmp * a_tmp / (4.0 * b_px * b_py));
          px->data[nx] = b_x * muDoubleScalarSin(d) * e_y;
          py->data[nx] = a_tmp / (2.0 * params_b);
          pdx->data[nx] = -b_x * muDoubleScalarCos(d) * e_y;
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
          }
        }

        i2 = b_p->size[0] * b_p->size[1];
        b_p->size[0] = 3;
        b_p->size[1] = px->size[1];
        emxEnsureCapacity_real_T(b_p, i2);
        b_loop_ub = px->size[1];
        for (i2 = 0; i2 < b_loop_ub; i2++) {
          b_p->data[3 * i2] = px->data[i2];
        }

        b_loop_ub = py->size[1];
        for (i2 = 0; i2 < b_loop_ub; i2++) {
          b_p->data[3 * i2 + 1] = py->data[i2];
        }

        b_loop_ub = pdx->size[1];
        for (i2 = 0; i2 < b_loop_ub; i2++) {
          b_p->data[3 * i2 + 2] = pdx->data[i2];
        }
      }
    }

    if (guard1) {
      b_y = NULL;
      m = emlrtCreateCharArray(2, &iv1[0]);
      emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 66, m, &b_u[0]);
      emlrtAssign(&b_y, m);
      disp(b_y, &b_emlrtMCI);
    }

    emxFree_real_T(&b_states_rough);
    emxFree_real_T(&pdy);
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
    i2 = b_ref_com->size[0] * b_ref_com->size[1];
    b_ref_com->size[0] = 3;
    b_ref_com->size[1] = loop_ub;
    emxEnsureCapacity_real_T(b_ref_com, i2);
    for (nx = 0; nx < loop_ub; nx++) {
      b_ref_com->data[3 * nx] = ref_com->data[3 * nx] - b_p->data[3 * nx];
      i2 = 3 * nx + 1;
      b_ref_com->data[i2] = ref_com->data[3 * nx + 1] - b_p->data[i2];
      i2 = 3 * nx + 2;
      b_ref_com->data[i2] = ref_com->data[3 * nx + 2] - b_p->data[i2];
    }

    emxFree_real_T(&b_p);
    vecnorm(b_ref_com, px);
    i2 = pdz->size[0] * pdz->size[1];
    pdz->size[0] = 1;
    pdz->size[1] = px->size[1];
    emxEnsureCapacity_real_T(pdz, i2);
    nx = px->size[1];
    emxFree_real_T(&b_ref_com);
    for (b_loop_ub = 0; b_loop_ub < nx; b_loop_ub++) {
      d = px->data[b_loop_ub];
      pdz->data[b_loop_ub] = d * d;
    }

    nx = pdz->size[1];
    if (pdz->size[1] == 0) {
      b_py = 0.0;
    } else {
      b_py = pdz->data[0];
      for (b_loop_ub = 2; b_loop_ub <= nx; b_loop_ub++) {
        b_py += pdz->data[b_loop_ub - 1];
      }
    }

    diff(delta_Fr_l, px);
    i2 = pdz->size[0] * pdz->size[1];
    pdz->size[0] = 1;
    pdz->size[1] = px->size[1];
    emxEnsureCapacity_real_T(pdz, i2);
    nx = px->size[1];
    emxFree_real_T(&delta_Fr_l);
    for (b_loop_ub = 0; b_loop_ub < nx; b_loop_ub++) {
      d = px->data[b_loop_ub];
      pdz->data[b_loop_ub] = d * d;
    }

    nx = pdz->size[1];
    if (pdz->size[1] == 0) {
      b_x = 0.0;
    } else {
      b_x = pdz->data[0];
      for (b_loop_ub = 2; b_loop_ub <= nx; b_loop_ub++) {
        b_x += pdz->data[b_loop_ub - 1];
      }
    }

    diff(delta_Fr_r, px);
    i2 = pdz->size[0] * pdz->size[1];
    pdz->size[0] = 1;
    pdz->size[1] = px->size[1];
    emxEnsureCapacity_real_T(pdz, i2);
    nx = px->size[1];
    emxFree_real_T(&delta_Fr_r);
    for (b_loop_ub = 0; b_loop_ub < nx; b_loop_ub++) {
      d = px->data[b_loop_ub];
      pdz->data[b_loop_ub] = d * d;
    }

    nx = pdz->size[1];
    if (pdz->size[1] == 0) {
      e_y = 0.0;
    } else {
      e_y = pdz->data[0];
      for (b_loop_ub = 2; b_loop_ub <= nx; b_loop_ub++) {
        e_y += pdz->data[b_loop_ub - 1];
      }
    }

    i2 = pdx->size[0] * pdx->size[1];
    pdx->size[0] = 1;
    loop_ub = (i1 - i) - 1;
    pdx->size[1] = loop_ub;
    emxEnsureCapacity_real_T(pdx, i2);
    for (i1 = 0; i1 < loop_ub; i1++) {
      pdx->data[i1] = x->data[(i + i1) + 1];
    }

    diff(pdx, px);
    i = pdz->size[0] * pdz->size[1];
    pdz->size[0] = 1;
    pdz->size[1] = px->size[1];
    emxEnsureCapacity_real_T(pdz, i);
    nx = px->size[1];
    emxFree_real_T(&pdx);
    for (b_loop_ub = 0; b_loop_ub < nx; b_loop_ub++) {
      d = px->data[b_loop_ub];
      pdz->data[b_loop_ub] = d * d;
    }

    emxFree_real_T(&px);
    nx = pdz->size[1];
    if (pdz->size[1] == 0) {
      b_px = 0.0;
    } else {
      b_px = pdz->data[0];
      for (b_loop_ub = 2; b_loop_ub <= nx; b_loop_ub++) {
        b_px += pdz->data[b_loop_ub - 1];
      }
    }

    emxFree_real_T(&pdz);
    varargout_1 = params_w1 * b_py + params_w2 * ((b_x + e_y) + b_px);

    /*  + params.w3* terminal_cost; % creates issues in gazebo */
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return varargout_1;
}

void optimize_cpp_mpc_propellers(const real_T actual_state[6], real_T actual_t,
  const emxArray_real_T *ref_com, const emxArray_real_T *Fr_l0, const
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
  emxArray_real_T *fscales_cineq_constraint;
  emxArray_real_T *lb;
  emxArray_real_T *r;
  emxArray_real_T *ub;
  emxArray_real_T *x0;
  f_struct_T FiniteDifferences;
  g_struct_T QRManager;
  h_struct_T CholManager;
  i_struct_T QPObjective;
  j_struct_T WorkingSet;
  k_struct_T MeritFunction;
  real_T fval;
  int32_T Cineq_size_idx_1;
  int32_T b_i;
  int32_T i;
  int32_T idxFillStart;
  int32_T mConstrMax;
  int32_T mLB;
  int32_T mUB;
  int32_T maxDims;
  int32_T nVar;
  int32_T nVarMax;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&x0, 2, true);

  /*  , delta_Fr_l0, delta_Fr_r0)   */
  /*  bootstrap */
  /*          if nargin <9  */
  /*              delta_Fr_l0 = 0*ones(1,mpc_N); */
  /*              delta_Fr_r0 = 0*ones(1,mpc_N); */
  /*          end */
  /*  init to zeros */
  /* opt vars=   Flegx Flegy Flexz Tf  traj_Fr_l traj_Fr_r */
  /*  % does not always satisfy bounds */
  /*  use constraint on force */
  i = x0->size[0] * x0->size[1];
  x0->size[0] = 1;
  nVar = ((int32_T)mpc_N + (int32_T)mpc_N) + (int32_T)mpc_N;
  x0->size[1] = nVar;
  emxEnsureCapacity_real_T(x0, i);
  idxFillStart = (int32_T)mpc_N;
  for (i = 0; i < idxFillStart; i++) {
    x0->data[i] = 0.0;
  }

  idxFillStart = (int32_T)mpc_N;
  for (i = 0; i < idxFillStart; i++) {
    x0->data[i + (int32_T)mpc_N] = 0.0;
  }

  idxFillStart = (int32_T)mpc_N;
  for (i = 0; i < idxFillStart; i++) {
    x0->data[(i + (int32_T)mpc_N) + (int32_T)mpc_N] = 0.0;
  }

  emxInit_real_T(&lb, 2, true);
  i = lb->size[0] * lb->size[1];
  lb->size[0] = 1;
  lb->size[1] = nVar;
  emxEnsureCapacity_real_T(lb, i);
  idxFillStart = (int32_T)mpc_N;
  for (i = 0; i < idxFillStart; i++) {
    lb->data[i] = -Fr_max;
  }

  idxFillStart = (int32_T)mpc_N;
  for (i = 0; i < idxFillStart; i++) {
    lb->data[i + (int32_T)mpc_N] = -Fr_max;
  }

  idxFillStart = (int32_T)mpc_N;
  for (i = 0; i < idxFillStart; i++) {
    lb->data[(i + (int32_T)mpc_N) + (int32_T)mpc_N] = -50.0;
  }

  emxInit_real_T(&ub, 2, true);
  i = ub->size[0] * ub->size[1];
  ub->size[0] = 1;
  ub->size[1] = nVar;
  emxEnsureCapacity_real_T(ub, i);
  idxFillStart = (int32_T)mpc_N;
  for (i = 0; i < idxFillStart; i++) {
    ub->data[i] = Fr_max;
  }

  idxFillStart = (int32_T)mpc_N;
  for (i = 0; i < idxFillStart; i++) {
    ub->data[i + (int32_T)mpc_N] = Fr_max;
  }

  idxFillStart = (int32_T)mpc_N;
  for (i = 0; i < idxFillStart; i++) {
    ub->data[(i + (int32_T)mpc_N) + (int32_T)mpc_N] = 50.0;
  }

  emxInit_real_T(&Hessian, 2, true);
  emxInit_real_T(&r, 2, true);
  anon(Fr_l0, Fr_r0, mpc_N, x0, r);
  Cineq_size_idx_1 = r->size[1];
  mLB = lb->size[1];
  mUB = ub->size[1];
  nVar = x0->size[1];
  mConstrMax = (((r->size[1] + lb->size[1]) + ub->size[1]) + r->size[1]) + 1;
  nVarMax = (x0->size[1] + r->size[1]) + 1;
  maxDims = muIntScalarMax_sint32(nVarMax, mConstrMax);
  i = Hessian->size[0] * Hessian->size[1];
  Hessian->size[0] = x0->size[1];
  Hessian->size[1] = x0->size[1];
  emxEnsureCapacity_real_T(Hessian, i);
  idxFillStart = x0->size[1] * x0->size[1];
  for (i = 0; i < idxFillStart; i++) {
    Hessian->data[i] = 0.0;
  }

  if (x0->size[1] > 0) {
    for (idxFillStart = 0; idxFillStart < nVar; idxFillStart++) {
      Hessian->data[idxFillStart + Hessian->size[0] * idxFillStart] = 1.0;
    }
  }

  emxInitStruct_struct_T(&TrialState, true);
  emxInitStruct_struct_T1(&FcnEvaluator, true);
  factoryConstruct(nVarMax, mConstrMax, r->size[1], x0, r->size[1], &TrialState);
  xcopy(x0->size[1], x0, TrialState.xstarsqp);
  FcnEvaluator.nVar = x0->size[1];
  FcnEvaluator.mCineq = r->size[1];
  for (b_i = 0; b_i < 6; b_i++) {
    FcnEvaluator.objfun.tunableEnvironment.f1[b_i] = actual_state[b_i];
  }

  FcnEvaluator.objfun.tunableEnvironment.f2 = actual_t;
  i = FcnEvaluator.objfun.tunableEnvironment.f3->size[0] *
    FcnEvaluator.objfun.tunableEnvironment.f3->size[1];
  FcnEvaluator.objfun.tunableEnvironment.f3->size[0] = 3;
  FcnEvaluator.objfun.tunableEnvironment.f3->size[1] = ref_com->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.objfun.tunableEnvironment.f3, i);
  idxFillStart = ref_com->size[0] * ref_com->size[1];
  for (i = 0; i < idxFillStart; i++) {
    FcnEvaluator.objfun.tunableEnvironment.f3->data[i] = ref_com->data[i];
  }

  i = FcnEvaluator.objfun.tunableEnvironment.f4->size[0] *
    FcnEvaluator.objfun.tunableEnvironment.f4->size[1];
  FcnEvaluator.objfun.tunableEnvironment.f4->size[0] = 1;
  FcnEvaluator.objfun.tunableEnvironment.f4->size[1] = Fr_l0->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.objfun.tunableEnvironment.f4, i);
  idxFillStart = Fr_l0->size[0] * Fr_l0->size[1];
  for (i = 0; i < idxFillStart; i++) {
    FcnEvaluator.objfun.tunableEnvironment.f4->data[i] = Fr_l0->data[i];
  }

  i = FcnEvaluator.objfun.tunableEnvironment.f5->size[0] *
    FcnEvaluator.objfun.tunableEnvironment.f5->size[1];
  FcnEvaluator.objfun.tunableEnvironment.f5->size[0] = 1;
  FcnEvaluator.objfun.tunableEnvironment.f5->size[1] = Fr_r0->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.objfun.tunableEnvironment.f5, i);
  idxFillStart = Fr_r0->size[0] * Fr_r0->size[1];
  for (i = 0; i < idxFillStart; i++) {
    FcnEvaluator.objfun.tunableEnvironment.f5->data[i] = Fr_r0->data[i];
  }

  FcnEvaluator.objfun.tunableEnvironment.f6 = mpc_N;
  FcnEvaluator.objfun.tunableEnvironment.f7 = *params;
  FcnEvaluator.nonlcon.tunableEnvironment.f1 = Fr_max;
  i = FcnEvaluator.nonlcon.tunableEnvironment.f2->size[0] *
    FcnEvaluator.nonlcon.tunableEnvironment.f2->size[1];
  FcnEvaluator.nonlcon.tunableEnvironment.f2->size[0] = 1;
  FcnEvaluator.nonlcon.tunableEnvironment.f2->size[1] = Fr_l0->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.nonlcon.tunableEnvironment.f2, i);
  idxFillStart = Fr_l0->size[0] * Fr_l0->size[1];
  for (i = 0; i < idxFillStart; i++) {
    FcnEvaluator.nonlcon.tunableEnvironment.f2->data[i] = Fr_l0->data[i];
  }

  i = FcnEvaluator.nonlcon.tunableEnvironment.f3->size[0] *
    FcnEvaluator.nonlcon.tunableEnvironment.f3->size[1];
  FcnEvaluator.nonlcon.tunableEnvironment.f3->size[0] = 1;
  FcnEvaluator.nonlcon.tunableEnvironment.f3->size[1] = Fr_r0->size[1];
  emxEnsureCapacity_real_T(FcnEvaluator.nonlcon.tunableEnvironment.f3, i);
  idxFillStart = Fr_r0->size[0] * Fr_r0->size[1];
  for (i = 0; i < idxFillStart; i++) {
    FcnEvaluator.nonlcon.tunableEnvironment.f3->data[i] = Fr_r0->data[i];
  }

  emxInitStruct_struct_T2(&FiniteDifferences, true);
  emxInitStruct_struct_T3(&QRManager, true);
  FcnEvaluator.nonlcon.tunableEnvironment.f4 = mpc_N;
  FcnEvaluator.mCeq = 0;
  FcnEvaluator.NonFiniteSupport = true;
  FcnEvaluator.SpecifyObjectiveGradient = false;
  FcnEvaluator.SpecifyConstraintGradient = false;
  FcnEvaluator.ScaleProblem = false;
  b_factoryConstruct(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, mpc_N,
                     params, Fr_max, Fr_l0, Fr_r0, mpc_N, x0->size[1], r->size[1],
                     lb, ub, &FiniteDifferences);
  QRManager.ldq = maxDims;
  i = QRManager.QR->size[0] * QRManager.QR->size[1];
  QRManager.QR->size[0] = maxDims;
  QRManager.QR->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.QR, i);
  i = QRManager.Q->size[0] * QRManager.Q->size[1];
  QRManager.Q->size[0] = maxDims;
  QRManager.Q->size[1] = maxDims;
  emxEnsureCapacity_real_T(QRManager.Q, i);
  idxFillStart = maxDims * maxDims;
  for (i = 0; i < idxFillStart; i++) {
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
  emxInit_real_T(&fscales_cineq_constraint, 1, true);
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
  QPObjective.grad->size[0] = nVarMax;
  emxEnsureCapacity_real_T(QPObjective.grad, i);
  i = QPObjective.Hx->size[0];
  QPObjective.Hx->size[0] = nVarMax - 1;
  emxEnsureCapacity_real_T(QPObjective.Hx, i);
  QPObjective.maxVar = nVarMax;
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
  if (2 < nVarMax) {
    memspace.workspace_double->size[1] = nVarMax;
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
  i = fscales_cineq_constraint->size[0];
  fscales_cineq_constraint->size[0] = r->size[1];
  emxEnsureCapacity_real_T(fscales_cineq_constraint, i);
  for (i = 0; i < Cineq_size_idx_1; i++) {
    fscales_cineq_constraint->data[i] = 1.0;
  }

  emxInit_int32_T(&indexLB, 1, true);
  emxInit_int32_T(&indexUB, 1, true);
  emxInit_int32_T(&indexFixed, 1, true);
  emxInitStruct_struct_T7(&WorkingSet, true);
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
                 &nVar);
  c_factoryConstruct(r->size[1], mLB, indexLB, mUB, indexUB, nVar, indexFixed,
                     x0->size[1], nVarMax, mConstrMax, &WorkingSet);
  emxFree_int32_T(&indexFixed);
  emxFree_int32_T(&indexUB);
  emxFree_int32_T(&indexLB);
  emxFree_real_T(&x0);
  if (lb->size[1] != 0) {
    for (maxDims = 0; maxDims < mLB; maxDims++) {
      TrialState.xstarsqp->data[WorkingSet.indexLB->data[maxDims] - 1] =
        muDoubleScalarMax(TrialState.xstarsqp->data[WorkingSet.indexLB->
                          data[maxDims] - 1], lb->data[WorkingSet.indexLB->
                          data[maxDims] - 1]);
    }
  }

  if (ub->size[1] != 0) {
    for (maxDims = 0; maxDims < mUB; maxDims++) {
      TrialState.xstarsqp->data[WorkingSet.indexUB->data[maxDims] - 1] =
        muDoubleScalarMin(TrialState.xstarsqp->data[WorkingSet.indexUB->
                          data[maxDims] - 1], ub->data[WorkingSet.indexUB->
                          data[maxDims] - 1]);
    }

    for (maxDims = 0; maxDims < nVar; maxDims++) {
      TrialState.xstarsqp->data[WorkingSet.indexFixed->data[maxDims] - 1] =
        ub->data[WorkingSet.indexFixed->data[maxDims] - 1];
    }
  }

  fval = b_anon(actual_state, ref_com, Fr_l0, Fr_r0, mpc_N, params->int_method,
                params->int_steps, params->b, params->p_a1, params->p_a2,
                params->g, params->m, params->w1, params->w2, params->mpc_dt,
                TrialState.xstarsqp);
  idxFillStart = 1;
  if (muDoubleScalarIsInf(fval) || muDoubleScalarIsNaN(fval)) {
    if (muDoubleScalarIsNaN(fval)) {
      idxFillStart = -6;
    } else if (fval < 0.0) {
      idxFillStart = -4;
    } else {
      idxFillStart = -5;
    }
  }

  TrialState.sqpFval = fval;
  if (idxFillStart == 1) {
    computeConstraints_(FcnEvaluator.nonlcon.tunableEnvironment.f2,
                        FcnEvaluator.nonlcon.tunableEnvironment.f3, mpc_N,
                        r->size[1], TrialState.xstarsqp, TrialState.cIneq,
                        TrialState.iNonIneq0);
  }

  computeForwardDifferences(&FiniteDifferences, fval, TrialState.cIneq,
    TrialState.iNonIneq0, TrialState.xstarsqp, TrialState.grad, WorkingSet.Aineq,
    TrialState.iNonIneq0, lb, ub);
  TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
  updateWorkingSetForNewQP(&WorkingSet, r->size[1], TrialState.cIneq, mLB, lb,
    mUB, ub, nVar);
  setProblemType(&WorkingSet, 3);
  idxFillStart = WorkingSet.isActiveIdx[2];
  i = WorkingSet.mConstrMax;
  emxFree_real_T(&r);
  for (maxDims = idxFillStart; maxDims <= i; maxDims++) {
    WorkingSet.isActiveConstr->data[maxDims - 1] = false;
  }

  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
  idxFillStart = WorkingSet.sizes[0];
  for (maxDims = 0; maxDims < idxFillStart; maxDims++) {
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
  fval = 0.0;
  for (maxDims = 0; maxDims < Cineq_size_idx_1; maxDims++) {
    if (TrialState.cIneq->data[maxDims] > 0.0) {
      fval += TrialState.cIneq->data[maxDims];
    }
  }

  MeritFunction.initConstrViolationIneq = fval;
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
         &QPObjective, fscales_cineq_constraint);
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = TrialState.xstarsqp->size[1];
  emxEnsureCapacity_real_T(x, i);
  idxFillStart = TrialState.xstarsqp->size[0] * TrialState.xstarsqp->size[1];
  emxFree_real_T(&Hessian);
  emxFreeStruct_struct_T7(&WorkingSet);
  emxFree_real_T(&fscales_cineq_constraint);
  emxFreeStruct_struct_T6(&memspace);
  emxFreeStruct_struct_T5(&QPObjective);
  emxFreeStruct_struct_T4(&CholManager);
  emxFreeStruct_struct_T3(&QRManager);
  emxFreeStruct_struct_T2(&FiniteDifferences);
  emxFreeStruct_struct_T1(&FcnEvaluator);
  emxFree_real_T(&ub);
  emxFree_real_T(&lb);
  for (i = 0; i < idxFillStart; i++) {
    x->data[i] = TrialState.xstarsqp->data[i];
  }

  *EXITFLAG = TrialState.sqpExitFlag;
  *final_cost = TrialState.sqpFval;
  emxFreeStruct_struct_T(&TrialState);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (optimize_cpp_mpc_propellers.c) */
