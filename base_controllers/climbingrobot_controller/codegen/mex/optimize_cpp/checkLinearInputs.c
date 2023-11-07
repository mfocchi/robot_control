/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * checkLinearInputs.c
 *
 * Code generation for function 'checkLinearInputs'
 *
 */

/* Include files */
#include "checkLinearInputs.h"
#include "any.h"
#include "eml_int_forloop_overflow_check.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo l_emlrtRSI = { 1,   /* lineNo */
  "checkLinearInputs",                 /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+validate/checkLinearInputs.p"/* pathName */
};

static emlrtRSInfo m_emlrtRSI = { 1,   /* lineNo */
  "checkX0",                           /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+validate/checkX0.p"/* pathName */
};

static emlrtRSInfo n_emlrtRSI = { 16,  /* lineNo */
  "all",                               /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/eml/lib/matlab/ops/all.m"/* pathName */
};

static emlrtRSInfo u_emlrtRSI = { 1,   /* lineNo */
  "checkBounds",                       /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+validate/checkBounds.p"/* pathName */
};

static emlrtRTEInfo e_emlrtRTEI = { 1, /* lineNo */
  1,                                   /* colNo */
  "checkX0",                           /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+validate/checkX0.p"/* pName */
};

static emlrtRTEInfo f_emlrtRTEI = { 1, /* lineNo */
  1,                                   /* colNo */
  "checkBounds",                       /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+validate/checkBounds.p"/* pName */
};

static emlrtRTEInfo xb_emlrtRTEI = { 15,/* lineNo */
  13,                                  /* colNo */
  "isinf",                             /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/eml/lib/matlab/elmat/isinf.m"/* pName */
};

static emlrtRTEInfo yb_emlrtRTEI = { 15,/* lineNo */
  13,                                  /* colNo */
  "isnan",                             /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/eml/lib/matlab/elmat/isnan.m"/* pName */
};

static emlrtRTEInfo ac_emlrtRTEI = { 13,/* lineNo */
  1,                                   /* colNo */
  "isfinite",                          /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/eml/lib/matlab/elmat/isfinite.m"/* pName */
};

/* Function Definitions */
void checkLinearInputs(const emlrtStack *sp, const emxArray_real_T *x0, const
  emxArray_real_T *lb, const emxArray_real_T *ub)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack st;
  emxArray_boolean_T *b;
  emxArray_boolean_T *r;
  int32_T i;
  int32_T loop_ub;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_boolean_T(sp, &b, 2, &e_emlrtRTEI, true);
  st.site = &l_emlrtRSI;
  i = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = x0->size[1];
  emxEnsureCapacity_boolean_T(&st, b, i, &xb_emlrtRTEI);
  loop_ub = x0->size[0] * x0->size[1];
  for (i = 0; i < loop_ub; i++) {
    b->data[i] = muDoubleScalarIsInf(x0->data[i]);
  }

  emxInit_boolean_T(&st, &r, 2, &yb_emlrtRTEI, true);
  i = r->size[0] * r->size[1];
  r->size[0] = 1;
  r->size[1] = x0->size[1];
  emxEnsureCapacity_boolean_T(&st, r, i, &yb_emlrtRTEI);
  loop_ub = x0->size[0] * x0->size[1];
  for (i = 0; i < loop_ub; i++) {
    r->data[i] = muDoubleScalarIsNaN(x0->data[i]);
  }

  i = b->size[0] * b->size[1];
  loop_ub = b->size[0] * b->size[1];
  b->size[0] = 1;
  emxEnsureCapacity_boolean_T(&st, b, loop_ub, &ac_emlrtRTEI);
  loop_ub = i - 1;
  for (i = 0; i <= loop_ub; i++) {
    b->data[i] = ((!b->data[i]) && (!r->data[i]));
  }

  b_st.site = &m_emlrtRSI;
  c_st.site = &n_emlrtRSI;
  d_st.site = &o_emlrtRSI;
  e_st.site = &p_emlrtRSI;
  f_st.site = &q_emlrtRSI;
  g_st.site = &r_emlrtRSI;
  y = true;
  h_st.site = &s_emlrtRSI;
  if ((1 <= b->size[1]) && (b->size[1] > 2147483646)) {
    i_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&i_st);
  }

  loop_ub = 1;
  exitg1 = false;
  while ((!exitg1) && (loop_ub <= b->size[1])) {
    if (!b->data[loop_ub - 1]) {
      y = false;
      exitg1 = true;
    } else {
      loop_ub++;
    }
  }

  if (!y) {
    emlrtErrorWithMessageIdR2018a(&st, &e_emlrtRTEI,
      "optim_codegen:common:InfNaNComplexDetected",
      "optim_codegen:common:InfNaNComplexDetected", 3, 4, 2, "x0");
  }

  st.site = &l_emlrtRSI;
  if (lb->size[1] != x0->size[1]) {
    emlrtErrorWithMessageIdR2018a(&st, &f_emlrtRTEI,
      "optim_codegen:common:InvalidPartialBounds",
      "optim_codegen:common:InvalidPartialBounds", 5, 4, 2, "lb", 6, (real_T)
      x0->size[1]);
  }

  i = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = lb->size[1];
  emxEnsureCapacity_boolean_T(&st, b, i, &f_emlrtRTEI);
  loop_ub = lb->size[0] * lb->size[1];
  for (i = 0; i < loop_ub; i++) {
    b->data[i] = (lb->data[i] == rtInf);
  }

  guard1 = false;
  b_st.site = &u_emlrtRSI;
  if (any(&b_st, b)) {
    guard1 = true;
  } else {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    r->size[1] = lb->size[1];
    emxEnsureCapacity_boolean_T(&st, r, i, &yb_emlrtRTEI);
    loop_ub = lb->size[0] * lb->size[1];
    for (i = 0; i < loop_ub; i++) {
      r->data[i] = muDoubleScalarIsNaN(lb->data[i]);
    }

    b_st.site = &u_emlrtRSI;
    if (any(&b_st, r)) {
      guard1 = true;
    }
  }

  if (guard1) {
    emlrtErrorWithMessageIdR2018a(&st, &f_emlrtRTEI,
      "optim_codegen:common:InfNaNComplexDetectedLB",
      "optim_codegen:common:InfNaNComplexDetectedLB", 0);
  }

  if (ub->size[1] != x0->size[1]) {
    emlrtErrorWithMessageIdR2018a(&st, &f_emlrtRTEI,
      "optim_codegen:common:InvalidPartialBounds",
      "optim_codegen:common:InvalidPartialBounds", 5, 4, 2, "ub", 6, (real_T)
      x0->size[1]);
  }

  i = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = ub->size[1];
  emxEnsureCapacity_boolean_T(&st, b, i, &f_emlrtRTEI);
  loop_ub = ub->size[0] * ub->size[1];
  for (i = 0; i < loop_ub; i++) {
    b->data[i] = (ub->data[i] == rtMinusInf);
  }

  b_st.site = &u_emlrtRSI;
  if (any(&b_st, b)) {
    y = true;
  } else {
    i = r->size[0] * r->size[1];
    r->size[0] = 1;
    r->size[1] = ub->size[1];
    emxEnsureCapacity_boolean_T(&st, r, i, &yb_emlrtRTEI);
    loop_ub = ub->size[0] * ub->size[1];
    for (i = 0; i < loop_ub; i++) {
      r->data[i] = muDoubleScalarIsNaN(ub->data[i]);
    }

    b_st.site = &u_emlrtRSI;
    if (any(&b_st, r)) {
      y = true;
    } else {
      y = false;
    }
  }

  emxFree_boolean_T(&r);
  emxFree_boolean_T(&b);
  if (y) {
    emlrtErrorWithMessageIdR2018a(&st, &f_emlrtRTEI,
      "optim_codegen:common:InfNaNComplexDetectedUB",
      "optim_codegen:common:InfNaNComplexDetectedUB", 0);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (checkLinearInputs.c) */
