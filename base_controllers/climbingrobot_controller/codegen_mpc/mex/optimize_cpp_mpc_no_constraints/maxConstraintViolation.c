/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * maxConstraintViolation.c
 *
 * Code generation for function 'maxConstraintViolation'
 *
 */

/* Include files */
#include "maxConstraintViolation.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
real_T b_maxConstraintViolation(const j_struct_T *obj, const emxArray_real_T *x)
{
  real_T v;
  int32_T idx;
  int32_T idxLB;
  int32_T mFixed;
  int32_T mLB;
  int32_T mUB;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  v = 0.0;
  if (obj->sizes[3] > 0) {
    for (idx = 0; idx < mLB; idx++) {
      idxLB = obj->indexLB->data[idx] - 1;
      v = muDoubleScalarMax(v, -x->data[idxLB] - obj->lb->data[idxLB]);
    }
  }

  if (obj->sizes[4] > 0) {
    for (idx = 0; idx < mUB; idx++) {
      mLB = obj->indexUB->data[idx] - 1;
      v = muDoubleScalarMax(v, x->data[mLB] - obj->ub->data[mLB]);
    }
  }

  if (obj->sizes[0] > 0) {
    for (idx = 0; idx < mFixed; idx++) {
      v = muDoubleScalarMax(v, muDoubleScalarAbs(x->data[obj->indexFixed->
        data[idx] - 1] - obj->ub->data[obj->indexFixed->data[idx] - 1]));
    }
  }

  return v;
}

real_T maxConstraintViolation(const j_struct_T *obj, const emxArray_real_T *x,
  int32_T ix0)
{
  real_T v;
  int32_T idx;
  int32_T mFixed;
  int32_T mLB;
  int32_T mUB;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  v = 0.0;
  if (obj->sizes[3] > 0) {
    for (idx = 0; idx < mLB; idx++) {
      v = muDoubleScalarMax(v, -x->data[(ix0 + obj->indexLB->data[idx]) - 2] -
                            obj->lb->data[obj->indexLB->data[idx] - 1]);
    }
  }

  if (obj->sizes[4] > 0) {
    for (idx = 0; idx < mUB; idx++) {
      v = muDoubleScalarMax(v, x->data[(ix0 + obj->indexUB->data[idx]) - 2] -
                            obj->ub->data[obj->indexUB->data[idx] - 1]);
    }
  }

  if (obj->sizes[0] > 0) {
    for (idx = 0; idx < mFixed; idx++) {
      v = muDoubleScalarMax(v, muDoubleScalarAbs(x->data[(ix0 + obj->
        indexFixed->data[idx]) - 2] - obj->ub->data[obj->indexFixed->data[idx] -
        1]));
    }
  }

  return v;
}

/* End of code generation (maxConstraintViolation.c) */
