/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * compressBounds.c
 *
 * Code generation for function 'compressBounds'
 *
 */

/* Include files */
#include "compressBounds.h"
#include "eml_int_forloop_overflow_check.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo uc_emlrtRSI = { 1,  /* lineNo */
  "compressBounds",                    /* fcnName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/compressBounds.p"/* pathName */
};

static emlrtBCInfo bd_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  1,                                   /* lineNo */
  1,                                   /* colNo */
  "",                                  /* aName */
  "compressBounds",                    /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/compressBounds.p",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void compressBounds(const emlrtStack *sp, int32_T nVar, emxArray_int32_T
                    *indexLB, emxArray_int32_T *indexUB, emxArray_int32_T
                    *indexFixed, const emxArray_real_T *lb, const
                    emxArray_real_T *ub, int32_T *mLB, int32_T *mUB, int32_T
                    *mFixed)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T d;
  int32_T i;
  int32_T idx;
  boolean_T guard1 = false;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  *mLB = 0;
  *mUB = 0;
  *mFixed = 0;
  st.site = &uc_emlrtRSI;
  if ((1 <= nVar) && (nVar > 2147483646)) {
    b_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < nVar; idx++) {
    if ((idx + 1 < 1) || (idx + 1 > lb->size[1])) {
      emlrtDynamicBoundsCheckR2012b(idx + 1, 1, lb->size[1], &bd_emlrtBCI, sp);
    }

    d = lb->data[idx];
    guard1 = false;
    if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
      if ((idx + 1 < 1) || (idx + 1 > lb->size[1])) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, lb->size[1], &bd_emlrtBCI, sp);
      }

      if ((idx + 1 < 1) || (idx + 1 > ub->size[1])) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, ub->size[1], &bd_emlrtBCI, sp);
      }

      if (muDoubleScalarAbs(d - ub->data[idx]) < 0.001) {
        (*mFixed)++;
        i = indexFixed->size[0];
        if ((*mFixed < 1) || (*mFixed > i)) {
          emlrtDynamicBoundsCheckR2012b(*mFixed, 1, i, &bd_emlrtBCI, sp);
        }

        indexFixed->data[*mFixed - 1] = idx + 1;
        if ((idx + 1 < 1) || (idx + 1 > ub->size[1])) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, ub->size[1], &bd_emlrtBCI,
            sp);
        }

        if ((idx + 1 < 1) || (idx + 1 > lb->size[1])) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, lb->size[1], &bd_emlrtBCI,
            sp);
        }
      } else {
        (*mLB)++;
        i = indexLB->size[0];
        if ((*mLB < 1) || (*mLB > i)) {
          emlrtDynamicBoundsCheckR2012b(*mLB, 1, i, &bd_emlrtBCI, sp);
        }

        indexLB->data[*mLB - 1] = idx + 1;
        if ((idx + 1 < 1) || (idx + 1 > lb->size[1])) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, lb->size[1], &bd_emlrtBCI,
            sp);
        }

        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if ((idx + 1 < 1) || (idx + 1 > ub->size[1])) {
        emlrtDynamicBoundsCheckR2012b(idx + 1, 1, ub->size[1], &bd_emlrtBCI, sp);
      }

      d = ub->data[idx];
      if ((!muDoubleScalarIsInf(d)) && (!muDoubleScalarIsNaN(d))) {
        (*mUB)++;
        i = indexUB->size[0];
        if ((*mUB < 1) || (*mUB > i)) {
          emlrtDynamicBoundsCheckR2012b(*mUB, 1, i, &bd_emlrtBCI, sp);
        }

        indexUB->data[*mUB - 1] = idx + 1;
        if ((idx + 1 < 1) || (idx + 1 > ub->size[1])) {
          emlrtDynamicBoundsCheckR2012b(idx + 1, 1, ub->size[1], &bd_emlrtBCI,
            sp);
        }
      }
    }
  }
}

/* End of code generation (compressBounds.c) */
