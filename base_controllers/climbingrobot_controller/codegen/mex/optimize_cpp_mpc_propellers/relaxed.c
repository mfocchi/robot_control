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
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "ixamax.h"
#include "optimize_cpp_mpc_propellers_internal_types.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "updatePenaltyParam.h"
#include "xasum.h"
#include "xcopy.h"
#include "xdot.h"
#include "xgemv.h"
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
  int32_T b_mIneq;
  int32_T idx;
  int32_T idx_max;
  int32_T mIneq;
  int32_T nActiveLBArtificial;
  int32_T nVarMax;
  int32_T nVarOrig;
  boolean_T tf;
  nVarOrig = WorkingSet->nVar + 1;
  nVarMax = WorkingSet->nVarMax;
  mIneq = WorkingSet->sizes[2];
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
  b_mIneq = WorkingSet->sizes[2];
  idx_max = WorkingSet->sizes[3] - WorkingSet->sizes[2];
  xcopy(WorkingSet->sizes[2], WorkingSet->bineq, memspace->workspace_double);
  c_xgemv(nVarOrig - 1, WorkingSet->sizes[2], WorkingSet->Aineq, WorkingSet->ldA,
          TrialState->xstar, memspace->workspace_double);
  for (idx = 0; idx < b_mIneq; idx++) {
    TrialState->xstar->data[(nVarOrig + idx) - 1] = (real_T)
      (memspace->workspace_double->data[idx] > 0.0) * memspace->
      workspace_double->data[idx];
    if (memspace->workspace_double->data[idx] <= 0.001) {
      addBoundToActiveSetMatrix_(WorkingSet, 4, (idx_max + idx) + 1);
    }
  }

  idx_max = qpoptions->MaxIterations;
  qpoptions->MaxIterations = ((qpoptions->MaxIterations + WorkingSet->nVar) -
    nVarOrig) + 1;
  b_qpoptions = *qpoptions;
  b_driver(Hessian, grad, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
  qpoptions->MaxIterations = idx_max;
  b_mIneq = WorkingSet->sizes[2];
  nActiveLBArtificial = 0;
  for (idx = 0; idx < b_mIneq; idx++) {
    tf = WorkingSet->isActiveConstr->data[(((WorkingSet->isActiveIdx[3] +
      WorkingSet->sizes[3]) - b_mIneq) + idx) - 1];
    memspace->workspace_int->data[idx] = tf;
    nActiveLBArtificial += tf;
  }

  if (TrialState->state != -6) {
    idx_max = WorkingSet->nVarMax - nVarOrig;
    updatePenaltyParam(MeritFunction, TrialState->sqpFval, TrialState->cIneq,
                       mIneq, TrialState->sqpIterations, (TrialState->fstar -
      rho * xasum(idx_max, TrialState->xstar, nVarOrig)) - beta / 2.0 * c_xdot
                       (idx_max, TrialState->xstar, nVarOrig, TrialState->xstar,
                        nVarOrig), TrialState->xstar, nVarOrig, nVarMax -
                       nVarOrig);
    idx_max = WorkingSet->isActiveIdx[2];
    b_mIneq = WorkingSet->nActiveConstr;
    for (idx = idx_max; idx <= b_mIneq; idx++) {
      if (WorkingSet->Wid->data[idx - 1] == 3) {
        TrialState->lambda->data[idx - 1] *= (real_T)memspace->
          workspace_int->data[WorkingSet->Wlocalidx->data[idx - 1] - 1];
      }
    }
  }

  idx_max = WorkingSet->sizes[0];
  b_mIneq = WorkingSet->sizes[3] - WorkingSet->sizes[2];
  idx = WorkingSet->nActiveConstr;
  while ((idx > idx_max) && (nActiveLBArtificial > 0)) {
    if ((WorkingSet->Wid->data[idx - 1] == 4) && (WorkingSet->Wlocalidx->
         data[idx - 1] > b_mIneq)) {
      nVarMax = WorkingSet->nActiveConstr - 1;
      beta = TrialState->lambda->data[nVarMax];
      TrialState->lambda->data[nVarMax] = 0.0;
      TrialState->lambda->data[idx - 1] = beta;
      removeConstr(WorkingSet, idx);
      nActiveLBArtificial--;
    }

    idx--;
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
