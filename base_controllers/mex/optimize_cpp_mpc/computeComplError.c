/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeComplError.c
 *
 * Code generation for function 'computeComplError'
 *
 */

/* Include files */
#include "computeComplError.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
real_T computeComplError(const emxArray_real_T *xCurrent,
                         const emxArray_int32_T *finiteLB, int32_T mLB,
                         const emxArray_real_T *lb,
                         const emxArray_int32_T *finiteUB, int32_T mUB,
                         const emxArray_real_T *ub,
                         const emxArray_real_T *lambda, int32_T iL0)
{
  const real_T *lambda_data;
  const real_T *lb_data;
  const real_T *ub_data;
  const real_T *xCurrent_data;
  real_T nlpComplError;
  const int32_T *finiteLB_data;
  const int32_T *finiteUB_data;
  int32_T idx;
  lambda_data = lambda->data;
  ub_data = ub->data;
  finiteUB_data = finiteUB->data;
  lb_data = lb->data;
  finiteLB_data = finiteLB->data;
  xCurrent_data = xCurrent->data;
  nlpComplError = 0.0;
  if (mLB + mUB > 0) {
    real_T lbDelta;
    real_T lbLambda;
    int32_T ubOffset;
    ubOffset = (iL0 + mLB) - 1;
    for (idx = 0; idx < mLB; idx++) {
      lbDelta = xCurrent_data[finiteLB_data[idx] - 1] -
                lb_data[finiteLB_data[idx] - 1];
      lbLambda = lambda_data[(iL0 + idx) - 1];
      nlpComplError = muDoubleScalarMax(
          nlpComplError,
          muDoubleScalarMin(
              muDoubleScalarAbs(lbDelta * lbLambda),
              muDoubleScalarMin(muDoubleScalarAbs(lbDelta), lbLambda)));
    }
    for (idx = 0; idx < mUB; idx++) {
      lbDelta = ub_data[finiteUB_data[idx] - 1] -
                xCurrent_data[finiteUB_data[idx] - 1];
      lbLambda = lambda_data[ubOffset + idx];
      nlpComplError = muDoubleScalarMax(
          nlpComplError,
          muDoubleScalarMin(
              muDoubleScalarAbs(lbDelta * lbLambda),
              muDoubleScalarMin(muDoubleScalarAbs(lbDelta), lbLambda)));
    }
  }
  return nlpComplError;
}

/* End of code generation (computeComplError.c) */
