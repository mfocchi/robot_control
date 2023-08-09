/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeDualFeasError.c
 *
 * Code generation for function 'computeDualFeasError'
 *
 */

/* Include files */
#include "computeDualFeasError.h"
#include "eml_int_forloop_overflow_check.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo me_emlrtRSI = { 1,  /* lineNo */
  "computeDualFeasError",              /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+fminconsqp/+stopping/computeDualFeasError.p"/* pathName */
};

static emlrtBCInfo xc_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  1,                                   /* lineNo */
  1,                                   /* colNo */
  "",                                  /* aName */
  "computeDualFeasError",              /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+fminconsqp/+stopping/computeDualFeasError.p",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void b_computeDualFeasError(const emlrtStack *sp, int32_T nVar, const
  emxArray_real_T *gradLag, boolean_T *gradOK, real_T *val)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T i;
  int32_T idx;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  *gradOK = true;
  *val = 0.0;
  st.site = &me_emlrtRSI;
  if ((1 <= nVar) && (nVar > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= nVar - 1)) {
    i = gradLag->size[0] * gradLag->size[1];
    if ((idx + 1 < 1) || (idx + 1 > i)) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &xc_emlrtBCI, sp);
    }

    *gradOK = ((!muDoubleScalarIsInf(gradLag->data[idx])) &&
               (!muDoubleScalarIsNaN(gradLag->data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      i = gradLag->size[0] * gradLag->size[1];
      if ((idx + 1 < 1) || (idx + 1 > i)) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, i, &xc_emlrtBCI, sp);
      }

      *val = muDoubleScalarMax(*val, muDoubleScalarAbs(gradLag->data[idx]));
      idx++;
    }
  }
}

void computeDualFeasError(const emlrtStack *sp, int32_T nVar, const
  emxArray_real_T *gradLag, boolean_T *gradOK, real_T *val)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T idx;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  *gradOK = true;
  *val = 0.0;
  st.site = &me_emlrtRSI;
  if ((1 <= nVar) && (nVar > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= nVar - 1)) {
    if ((idx + 1 < 1) || (idx + 1 > gradLag->size[0])) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, gradLag->size[0], &xc_emlrtBCI,
        sp);
    }

    *gradOK = ((!muDoubleScalarIsInf(gradLag->data[idx])) &&
               (!muDoubleScalarIsNaN(gradLag->data[idx])));
    if (!*gradOK) {
      exitg1 = true;
    } else {
      if ((idx + 1 < 1) || (idx + 1 > gradLag->size[0])) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, gradLag->size[0], &xc_emlrtBCI,
          sp);
      }

      *val = muDoubleScalarMax(*val, muDoubleScalarAbs(gradLag->data[idx]));
      idx++;
    }
  }
}

/* End of code generation (computeDualFeasError.c) */
