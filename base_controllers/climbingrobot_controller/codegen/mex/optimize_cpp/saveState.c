/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * saveState.c
 *
 * Code generation for function 'saveState'
 *
 */

/* Include files */
#include "saveState.h"
#include "eml_int_forloop_overflow_check.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo of_emlrtRSI = { 1,  /* lineNo */
  "saveState",                         /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+fminconsqp/+TrialState/saveState.p"/* pathName */
};

static emlrtBCInfo sd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  1,                                   /* lineNo */
  1,                                   /* colNo */
  "",                                  /* aName */
  "saveState",                         /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+fminconsqp/+TrialState/saveState.p",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void saveState(const emlrtStack *sp, d_struct_T *obj)
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T b_i;
  int32_T i;
  int32_T nVar;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  obj->sqpFval_old = obj->sqpFval;
  nVar = obj->xstarsqp->size[1];
  st.site = &of_emlrtRSI;
  xcopy(obj->xstarsqp->size[1], obj->xstarsqp, obj->xstarsqp_old);
  st.site = &of_emlrtRSI;
  if ((1 <= obj->xstarsqp->size[1]) && (obj->xstarsqp->size[1] > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (i = 0; i < nVar; i++) {
    b_i = obj->grad->size[0];
    if ((i + 1 < 1) || (i + 1 > b_i)) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_i, &sd_emlrtBCI, sp);
    }

    b_i = obj->grad_old->size[0];
    if ((i + 1 < 1) || (i + 1 > b_i)) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, b_i, &sd_emlrtBCI, sp);
    }

    obj->grad_old->data[i] = obj->grad->data[i];
  }

  st.site = &of_emlrtRSI;
  c_xcopy(obj->mIneq, obj->cIneq, obj->cIneq_old);
}

/* End of code generation (saveState.c) */
