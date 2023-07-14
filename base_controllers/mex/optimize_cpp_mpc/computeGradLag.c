/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * computeGradLag.c
 *
 * Code generation for function 'computeGradLag'
 *
 */

/* Include files */
#include "computeGradLag.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void computeGradLag(emxArray_real_T *workspace, int32_T nVar,
                    const emxArray_real_T *grad,
                    const emxArray_int32_T *finiteFixed, int32_T mFixed,
                    const emxArray_int32_T *finiteLB, int32_T mLB,
                    const emxArray_int32_T *finiteUB, int32_T mUB,
                    const emxArray_real_T *lambda)
{
  const real_T *grad_data;
  const real_T *lambda_data;
  real_T *workspace_data;
  const int32_T *finiteFixed_data;
  const int32_T *finiteLB_data;
  const int32_T *finiteUB_data;
  int32_T iL0;
  int32_T idx;
  lambda_data = lambda->data;
  finiteUB_data = finiteUB->data;
  finiteLB_data = finiteLB->data;
  finiteFixed_data = finiteFixed->data;
  grad_data = grad->data;
  workspace_data = workspace->data;
  for (iL0 = 0; iL0 < nVar; iL0++) {
    workspace_data[iL0] = grad_data[iL0];
  }
  for (idx = 0; idx < mFixed; idx++) {
    workspace_data[finiteFixed_data[idx] - 1] += lambda_data[idx];
  }
  for (idx = 0; idx < mLB; idx++) {
    workspace_data[finiteLB_data[idx] - 1] -= lambda_data[mFixed + idx];
  }
  iL0 = mFixed + mLB;
  for (idx = 0; idx < mUB; idx++) {
    workspace_data[finiteUB_data[idx] - 1] += lambda_data[iL0 + idx];
  }
}

/* End of code generation (computeGradLag.c) */
