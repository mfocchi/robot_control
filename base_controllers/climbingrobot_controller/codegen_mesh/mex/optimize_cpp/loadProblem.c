/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * loadProblem.c
 *
 * Code generation for function 'loadProblem'
 *
 */

/* Include files */
#include "loadProblem.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void loadProblem(i_struct_T *obj, int32_T mIneq, int32_T mLB, int32_T mUB,
                 int32_T mFixed, int32_T mConstrMax)
{
  int32_T i;
  int32_T k;
  i = mIneq + mLB;
  k = (i + mUB) + mFixed;
  obj->mConstr = k;
  obj->mConstrOrig = k;
  obj->mConstrMax = mConstrMax;
  obj->sizes[0] = mFixed;
  obj->sizes[1] = 0;
  obj->sizes[2] = mIneq;
  obj->sizes[3] = mLB;
  obj->sizes[4] = mUB;
  obj->sizesPhaseOne[0] = mFixed;
  obj->sizesPhaseOne[1] = 0;
  obj->sizesPhaseOne[2] = mIneq;
  obj->sizesPhaseOne[3] = mLB + 1;
  obj->sizesPhaseOne[4] = mUB;
  obj->sizesRegularized[0] = mFixed;
  obj->sizesRegularized[1] = 0;
  obj->sizesRegularized[2] = mIneq;
  obj->sizesRegularized[3] = i;
  obj->sizesRegularized[4] = mUB;
  obj->sizesRegPhaseOne[0] = mFixed;
  obj->sizesRegPhaseOne[1] = 0;
  obj->sizesRegPhaseOne[2] = mIneq;
  obj->sizesRegPhaseOne[3] = i + 1;
  obj->sizesRegPhaseOne[4] = mUB;
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 0;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = mLB;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->sizesNormal[k] = obj->sizes[k];
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 6; k++) {
    obj->isActiveIdx[k] = obj->isActiveIdxRegPhaseOne[k];
    obj->isActiveIdxNormal[k] = obj->isActiveIdxRegPhaseOne[k];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 0;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 1;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 6; k++) {
    obj->isActiveIdxPhaseOne[k] = obj->isActiveIdxRegPhaseOne[k];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 0;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = i;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
  for (k = 0; k < 6; k++) {
    obj->isActiveIdxRegularized[k] = obj->isActiveIdxRegPhaseOne[k];
  }
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 0;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = i + 1;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (k = 0; k < 5; k++) {
    obj->isActiveIdxRegPhaseOne[k + 1] += obj->isActiveIdxRegPhaseOne[k];
  }
}

/* End of code generation (loadProblem.c) */
