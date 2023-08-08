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
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgemv.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
real_T maxConstraintViolation(j_struct_T *obj, const emxArray_real_T *x)
{
  real_T v;
  int32_T idx;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mUB;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  switch (obj->probType) {
   case 2:
    v = 0.0;
    mIneq = obj->sizes[2];
    if ((obj->Aineq->size[0] != 0) && (obj->Aineq->size[1] != 0)) {
      xcopy(obj->sizes[2], obj->bineq, obj->maxConstrWorkspace);
      c_xgemv(obj->nVarOrig, obj->sizes[2], obj->Aineq, obj->ldA, x,
              obj->maxConstrWorkspace);
      for (idx = 0; idx < mIneq; idx++) {
        obj->maxConstrWorkspace->data[idx] -= x->data[obj->nVarOrig + idx];
        v = muDoubleScalarMax(v, obj->maxConstrWorkspace->data[idx]);
      }
    }
    break;

   default:
    v = 0.0;
    mIneq = obj->sizes[2];
    if ((obj->Aineq->size[0] != 0) && (obj->Aineq->size[1] != 0)) {
      xcopy(obj->sizes[2], obj->bineq, obj->maxConstrWorkspace);
      c_xgemv(obj->nVar, obj->sizes[2], obj->Aineq, obj->ldA, x,
              obj->maxConstrWorkspace);
      for (idx = 0; idx < mIneq; idx++) {
        v = muDoubleScalarMax(v, obj->maxConstrWorkspace->data[idx]);
      }
    }
    break;
  }

  if (obj->sizes[3] > 0) {
    for (idx = 0; idx < mLB; idx++) {
      mIneq = obj->indexLB->data[idx] - 1;
      v = muDoubleScalarMax(v, -x->data[mIneq] - obj->lb->data[mIneq]);
    }
  }

  if (obj->sizes[4] > 0) {
    for (idx = 0; idx < mUB; idx++) {
      mIneq = obj->indexUB->data[idx] - 1;
      v = muDoubleScalarMax(v, x->data[mIneq] - obj->ub->data[mIneq]);
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

/* End of code generation (maxConstraintViolation.c) */
