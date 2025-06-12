/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * relaxed.c
 *
 * Code generation for function 'relaxed'
 *
 */

/* Include files */
#include "relaxed.h"
#include "driver1.h"
#include "ixamax.h"
#include "optimize_cpp_mpc_no_constraints_internal_types.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "updatePenaltyParam.h"
#include "xasum.h"
#include "xdot.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void relaxed(const emxArray_real_T *Hessian, const emxArray_real_T *grad,
             d_struct_T *TrialState, k_struct_T *MeritFunction, c_struct_T
             *memspace, j_struct_T *WorkingSet, g_struct_T *QRManager,
             h_struct_T *CholManager, i_struct_T *QPObjective, b_struct_T
             *qpoptions)
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  b_struct_T b_qpoptions;
  real_T beta;
  real_T rho;
  int32_T idx;
  int32_T idx_max;
  int32_T nVarMax;
  int32_T nVarOrig;
  nVarOrig = WorkingSet->nVar + 1;
  nVarMax = WorkingSet->nVarMax;
  beta = 0.0;
  for (idx = 0; idx <= nVarOrig - 2; idx++) {
    beta += Hessian->data[idx + Hessian->size[0] * idx];
  }

  beta /= (real_T)WorkingSet->nVar;
  if (TrialState->sqpIterations <= 1) {
    if (QPObjective->nvar < 1) {
      idx_max = 0;
    } else {
      n_t = (ptrdiff_t)QPObjective->nvar;
      incx_t = (ptrdiff_t)1;
      n_t = idamax(&n_t, &grad->data[0], &incx_t);
      idx_max = (int32_T)n_t;
    }

    rho = 100.0 * muDoubleScalarMax(1.0, muDoubleScalarAbs(grad->data[idx_max -
      1]));
  } else {
    rho = muDoubleScalarAbs(TrialState->lambdasqp->data[ixamax
      (WorkingSet->mConstr, TrialState->lambdasqp) - 1]);
  }

  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->beta = beta;
  QPObjective->rho = rho;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 4;
  setProblemType(WorkingSet, 2);
  idx_max = qpoptions->MaxIterations;
  qpoptions->MaxIterations = ((qpoptions->MaxIterations + WorkingSet->nVar) -
    nVarOrig) + 1;
  b_qpoptions = *qpoptions;
  b_driver(Hessian, grad, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
  qpoptions->MaxIterations = idx_max;
  if (TrialState->state != -6) {
    idx_max = WorkingSet->nVarMax - nVarOrig;
    updatePenaltyParam(MeritFunction, TrialState->sqpFval,
                       TrialState->sqpIterations, (TrialState->fstar - rho *
      xasum(idx_max, TrialState->xstar, nVarOrig)) - beta / 2.0 * c_xdot(idx_max,
      TrialState->xstar, nVarOrig, TrialState->xstar, nVarOrig),
                       TrialState->xstar, nVarOrig, nVarMax - nVarOrig);
    idx_max = WorkingSet->isActiveIdx[2];
    nVarMax = WorkingSet->nActiveConstr;
    for (idx = idx_max; idx <= nVarMax; idx++) {
      if (WorkingSet->Wid->data[idx - 1] == 3) {
        TrialState->lambda->data[idx - 1] *= (real_T)memspace->
          workspace_int->data[WorkingSet->Wlocalidx->data[idx - 1] - 1];
      }
    }
  }

  QPObjective->nvar = nVarOrig - 1;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 3;
  setProblemType(WorkingSet, 3);
  sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr, WorkingSet->sizes,
               WorkingSet->isActiveIdx, WorkingSet->Wid, WorkingSet->Wlocalidx,
               memspace->workspace_double);
}

/* End of code generation (relaxed.c) */
