/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computePrimalFeasError.c
 *
 * Code generation for function 'computePrimalFeasError'
 *
 */

/* Include files */
#include "computePrimalFeasError.h"
#include "eml_int_forloop_overflow_check.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo le_emlrtRSI = { 1,  /* lineNo */
  "computePrimalFeasError",            /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+fminconsqp/+stopping/computePrimalFeasError.p"/* pathName */
};

static emlrtBCInfo wc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  1,                                   /* lineNo */
  1,                                   /* colNo */
  "",                                  /* aName */
  "computePrimalFeasError",            /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+fminconsqp/+stopping/computePrimalFeasError.p",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
real_T computePrimalFeasError(const emlrtStack *sp, const emxArray_real_T *x,
  int32_T mLinIneq, int32_T mNonlinIneq, const emxArray_real_T *cIneq, const
  emxArray_int32_T *finiteLB, int32_T mLB, const emxArray_real_T *lb, const
  emxArray_int32_T *finiteUB, int32_T mUB, const emxArray_real_T *ub)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T feasError;
  int32_T idx;
  int32_T mIneq;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  feasError = 0.0;
  mIneq = mNonlinIneq + mLinIneq;
  st.site = &le_emlrtRSI;
  st.site = &le_emlrtRSI;
  if ((1 <= mIneq) && (mIneq > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < mIneq; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > cIneq->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, cIneq->size[0], &wc_emlrtBCI, sp);
    }

    feasError = muDoubleScalarMax(feasError, cIneq->data[idx]);
  }

  st.site = &le_emlrtRSI;
  if ((1 <= mLB) && (mLB > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < mLB; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > finiteLB->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteLB->size[0], &wc_emlrtBCI,
        sp);
    }

    mIneq = finiteLB->data[idx] - 1;
    if ((finiteLB->data[idx] < 1) || (finiteLB->data[idx] > lb->size[1])) {
      emlrtDynamicBoundsCheckR2012b(finiteLB->data[idx], 1, lb->size[1],
        &wc_emlrtBCI, sp);
    }

    if ((finiteLB->data[idx] < 1) || (finiteLB->data[idx] > x->size[1])) {
      emlrtDynamicBoundsCheckR2012b(finiteLB->data[idx], 1, x->size[1],
        &wc_emlrtBCI, sp);
    }

    feasError = muDoubleScalarMax(feasError, lb->data[mIneq] - x->data[mIneq]);
  }

  st.site = &le_emlrtRSI;
  if ((1 <= mUB) && (mUB > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < mUB; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > finiteUB->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, finiteUB->size[0], &wc_emlrtBCI,
        sp);
    }

    mIneq = finiteUB->data[idx] - 1;
    if ((finiteUB->data[idx] < 1) || (finiteUB->data[idx] > x->size[1])) {
      emlrtDynamicBoundsCheckR2012b(finiteUB->data[idx], 1, x->size[1],
        &wc_emlrtBCI, sp);
    }

    if ((finiteUB->data[idx] < 1) || (finiteUB->data[idx] > ub->size[1])) {
      emlrtDynamicBoundsCheckR2012b(finiteUB->data[idx], 1, ub->size[1],
        &wc_emlrtBCI, sp);
    }

    feasError = muDoubleScalarMax(feasError, x->data[mIneq] - ub->data[mIneq]);
  }

  return feasError;
}

/* End of code generation (computePrimalFeasError.c) */
