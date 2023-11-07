/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * isDeltaXTooSmall.c
 *
 * Code generation for function 'isDeltaXTooSmall'
 *
 */

/* Include files */
#include "isDeltaXTooSmall.h"
#include "eml_int_forloop_overflow_check.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo oe_emlrtRSI = { 1,  /* lineNo */
  "isDeltaXTooSmall",                  /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+fminconsqp/+stopping/isDeltaXTooSmall.p"/* pathName */
};

static emlrtBCInfo ad_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  1,                                   /* lineNo */
  1,                                   /* colNo */
  "",                                  /* aName */
  "isDeltaXTooSmall",                  /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+fminconsqp/+stopping/isDeltaXTooSmall.p",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
boolean_T isDeltaXTooSmall(const emlrtStack *sp, const emxArray_real_T *xCurrent,
  const emxArray_real_T *delta_x, int32_T nVar)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  boolean_T exitg1;
  boolean_T tf;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  tf = true;
  st.site = &oe_emlrtRSI;
  if ((1 <= nVar) && (nVar > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= nVar - 1)) {
    if ((idx + 1 < 1) || (idx + 1 > xCurrent->size[1])) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, xCurrent->size[1], &ad_emlrtBCI,
        sp);
    }

    if ((idx + 1 < 1) || (idx + 1 > delta_x->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, delta_x->size[0], &ad_emlrtBCI,
        sp);
    }

    if (1.0E-6 * muDoubleScalarMax(1.0, muDoubleScalarAbs(xCurrent->data[idx])) <=
        muDoubleScalarAbs(delta_x->data[idx])) {
      tf = false;
      exitg1 = true;
    } else {
      idx++;
    }
  }

  return tf;
}

/* End of code generation (isDeltaXTooSmall.c) */
