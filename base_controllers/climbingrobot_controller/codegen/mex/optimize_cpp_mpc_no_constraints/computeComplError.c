/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeComplError.c
 *
 * Code generation for function 'computeComplError'
 *
 */

/* Include files */
#include "computeComplError.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
real_T computeComplError(const emxArray_real_T *xCurrent, const emxArray_int32_T
  *finiteLB, int32_T mLB, const emxArray_real_T *lb, const emxArray_int32_T
  *finiteUB, int32_T mUB, const emxArray_real_T *ub, const emxArray_real_T
  *lambda, int32_T iL0)
{
  real_T lbDelta;
  real_T lbLambda;
  real_T nlpComplError;
  int32_T idx;
  int32_T ubOffset;
  nlpComplError = 0.0;
  if (mLB + mUB > 0) {
    ubOffset = (iL0 + mLB) - 1;
    for (idx = 0; idx < mLB; idx++) {
      lbDelta = xCurrent->data[finiteLB->data[idx] - 1] - lb->data
        [finiteLB->data[idx] - 1];
      lbLambda = lambda->data[(iL0 + idx) - 1];
      nlpComplError = muDoubleScalarMax(nlpComplError, muDoubleScalarMin
        (muDoubleScalarAbs(lbDelta * lbLambda), muDoubleScalarMin
         (muDoubleScalarAbs(lbDelta), lbLambda)));
    }

    for (idx = 0; idx < mUB; idx++) {
      lbDelta = ub->data[finiteUB->data[idx] - 1] - xCurrent->data
        [finiteUB->data[idx] - 1];
      lbLambda = lambda->data[ubOffset + idx];
      nlpComplError = muDoubleScalarMax(nlpComplError, muDoubleScalarMin
        (muDoubleScalarAbs(lbDelta * lbLambda), muDoubleScalarMin
         (muDoubleScalarAbs(lbDelta), lbLambda)));
    }
  }

  return nlpComplError;
}

/* End of code generation (computeComplError.c) */
