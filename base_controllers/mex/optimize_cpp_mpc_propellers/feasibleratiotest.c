/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * feasibleratiotest.c
 *
 * Code generation for function 'feasibleratiotest'
 *
 */

/* Include files */
#include "feasibleratiotest.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgemv.h"
#include "xnrm2.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void feasibleratiotest(const emxArray_real_T *solution_xstar, const
  emxArray_real_T *solution_searchDir, emxArray_real_T *workspace, int32_T
  workingset_nVar, int32_T workingset_ldA, const emxArray_real_T
  *workingset_Aineq, const emxArray_real_T *workingset_bineq, const
  emxArray_real_T *workingset_lb, const emxArray_real_T *workingset_ub, const
  emxArray_int32_T *workingset_indexLB, const emxArray_int32_T
  *workingset_indexUB, const int32_T workingset_sizes[5], const int32_T
  workingset_isActiveIdx[6], const emxArray_boolean_T *workingset_isActiveConstr,
  const int32_T workingset_nWConstr[5], boolean_T isPhaseOne, real_T *alpha,
  boolean_T *newBlocking, int32_T *constrType, int32_T *constrIdx)
{
  real_T alphaTemp;
  real_T denomTol;
  real_T phaseOneCorrectionP;
  real_T phaseOneCorrectionX;
  real_T ratio;
  int32_T i;
  int32_T idx;
  int32_T ldw;
  int32_T totalIneq;
  int32_T totalUB;
  totalIneq = workingset_sizes[2];
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 * xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    xcopy(workingset_sizes[2], workingset_bineq, workspace);
    c_xgemv(workingset_nVar, workingset_sizes[2], workingset_Aineq,
            workingset_ldA, solution_xstar, workspace);
    ldw = workspace->size[0];
    i_xgemv(workingset_nVar, workingset_sizes[2], workingset_Aineq,
            workingset_ldA, solution_searchDir, workspace, workspace->size[0] +
            1);
    for (idx = 0; idx < totalIneq; idx++) {
      i = ldw + idx;
      if ((workspace->data[i] > denomTol) && (!workingset_isActiveConstr->data
           [(workingset_isActiveIdx[2] + idx) - 1])) {
        alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(workspace->data[idx]),
          0.001 - workspace->data[idx]) / workspace->data[i];
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX = (real_T)isPhaseOne * solution_xstar->
      data[workingset_nVar - 1];
    phaseOneCorrectionP = (real_T)isPhaseOne * solution_searchDir->
      data[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (idx = 0; idx <= i - 2; idx++) {
      alphaTemp = -solution_searchDir->data[workingset_indexLB->data[idx] - 1] -
        phaseOneCorrectionP;
      if ((alphaTemp > denomTol) && (!workingset_isActiveConstr->data
           [(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio = (-solution_xstar->data[workingset_indexLB->data[idx] - 1] -
                 workingset_lb->data[workingset_indexLB->data[idx] - 1]) -
          phaseOneCorrectionX;
        alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(ratio), 0.001 - ratio) /
          alphaTemp;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }

    i = workingset_indexLB->data[workingset_sizes[3] - 1] - 1;
    alphaTemp = -solution_searchDir->data[i];
    if ((alphaTemp > denomTol) && (!workingset_isActiveConstr->data
         [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar->data[i] - workingset_lb->data[i];
      alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(ratio), 0.001 - ratio) /
        alphaTemp;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }

  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX = (real_T)isPhaseOne * solution_xstar->
      data[workingset_nVar - 1];
    phaseOneCorrectionP = (real_T)isPhaseOne * solution_searchDir->
      data[workingset_nVar - 1];
    for (idx = 0; idx < totalUB; idx++) {
      alphaTemp = solution_searchDir->data[workingset_indexUB->data[idx] - 1] -
        phaseOneCorrectionP;
      if ((alphaTemp > denomTol) && (!workingset_isActiveConstr->data
           [(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio = (solution_xstar->data[workingset_indexUB->data[idx] - 1] -
                 workingset_ub->data[workingset_indexUB->data[idx] - 1]) -
          phaseOneCorrectionX;
        alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(ratio), 0.001 - ratio) /
          alphaTemp;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (!isPhaseOne) {
    if ((*newBlocking) && (*alpha > 1.0)) {
      *newBlocking = false;
    }

    *alpha = muDoubleScalarMin(*alpha, 1.0);
  }
}

/* End of code generation (feasibleratiotest.c) */
