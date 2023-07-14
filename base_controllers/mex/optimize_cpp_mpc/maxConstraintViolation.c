/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * maxConstraintViolation.c
 *
 * Code generation for function 'maxConstraintViolation'
 *
 */

/* Include files */
#include "maxConstraintViolation.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
real_T maxConstraintViolation(const h_struct_T *obj, const emxArray_real_T *x)
{
  const real_T *x_data;
  real_T v;
  int32_T idx;
  int32_T mFixed;
  int32_T mLB;
  int32_T mUB;
  x_data = x->data;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  v = 0.0;
  if (obj->sizes[3] > 0) {
    for (idx = 0; idx < mLB; idx++) {
      int32_T idxLB;
      idxLB = obj->indexLB->data[idx] - 1;
      v = muDoubleScalarMax(v, -x_data[idxLB] - obj->lb->data[idxLB]);
    }
  }
  if (obj->sizes[4] > 0) {
    for (idx = 0; idx < mUB; idx++) {
      mLB = obj->indexUB->data[idx] - 1;
      v = muDoubleScalarMax(v, x_data[mLB] - obj->ub->data[mLB]);
    }
  }
  if (obj->sizes[0] > 0) {
    for (idx = 0; idx < mFixed; idx++) {
      v = muDoubleScalarMax(
          v, muDoubleScalarAbs(x_data[obj->indexFixed->data[idx] - 1] -
                               obj->ub->data[obj->indexFixed->data[idx] - 1]));
    }
  }
  return v;
}

/* End of code generation (maxConstraintViolation.c) */
