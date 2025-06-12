/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeGradLag.c
 *
 * Code generation for function 'computeGradLag'
 *
 */

/* Include files */
#include "computeGradLag.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void computeGradLag(emxArray_real_T *workspace, int32_T nVar, const
                    emxArray_real_T *grad, const emxArray_int32_T *finiteFixed,
                    int32_T mFixed, const emxArray_int32_T *finiteLB, int32_T
                    mLB, const emxArray_int32_T *finiteUB, int32_T mUB, const
                    emxArray_real_T *lambda)
{
  int32_T iL0;
  int32_T idx;
  for (iL0 = 0; iL0 < nVar; iL0++) {
    workspace->data[iL0] = grad->data[iL0];
  }

  for (idx = 0; idx < mFixed; idx++) {
    workspace->data[finiteFixed->data[idx] - 1] += lambda->data[idx];
  }

  iL0 = mFixed;
  for (idx = 0; idx < mLB; idx++) {
    workspace->data[finiteLB->data[idx] - 1] -= lambda->data[iL0];
    iL0++;
  }

  for (idx = 0; idx < mUB; idx++) {
    workspace->data[finiteUB->data[idx] - 1] += lambda->data[iL0];
    iL0++;
  }
}

/* End of code generation (computeGradLag.c) */
