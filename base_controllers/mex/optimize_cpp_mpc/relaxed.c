/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
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
#include "optimize_cpp_mpc_internal_types.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void relaxed(const emxArray_real_T *Hessian, const emxArray_real_T *grad,
             g_struct_T *TrialState, struct_T *MeritFunction,
             f_struct_T *memspace, h_struct_T *WorkingSet,
             c_struct_T *QRManager, d_struct_T *CholManager,
             e_struct_T *QPObjective, k_struct_T *qpoptions)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  k_struct_T b_qpoptions;
  const real_T *Hessian_data;
  const real_T *grad_data;
  real_T beta;
  real_T rho;
  int32_T idx;
  int32_T idx_max;
  int32_T nVarOrig;
  grad_data = grad->data;
  Hessian_data = Hessian->data;
  nVarOrig = WorkingSet->nVar;
  beta = 0.0;
  for (idx = 0; idx < nVarOrig; idx++) {
    beta += Hessian_data[idx + Hessian->size[0] * idx];
  }
  beta /= (real_T)WorkingSet->nVar;
  if (TrialState->sqpIterations <= 1) {
    if (QPObjective->nvar < 1) {
      idx_max = 0;
    } else {
      n_t = (ptrdiff_t)QPObjective->nvar;
      incx_t = (ptrdiff_t)1;
      n_t = idamax(&n_t, (real_T *)&grad_data[0], &incx_t);
      idx_max = (int32_T)n_t;
    }
    rho = 100.0 *
          muDoubleScalarMax(1.0, muDoubleScalarAbs(grad_data[idx_max - 1]));
  } else {
    if (WorkingSet->mConstr < 1) {
      idx_max = 0;
    } else {
      n_t = (ptrdiff_t)WorkingSet->mConstr;
      incx_t = (ptrdiff_t)1;
      n_t = idamax(&n_t, &TrialState->lambdasqp->data[0], &incx_t);
      idx_max = (int32_T)n_t;
    }
    rho = muDoubleScalarAbs(TrialState->lambdasqp->data[idx_max - 1]);
  }
  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->beta = beta;
  QPObjective->rho = rho;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 4;
  setProblemType(WorkingSet, 2);
  idx_max = qpoptions->MaxIterations;
  qpoptions->MaxIterations =
      (qpoptions->MaxIterations + WorkingSet->nVar) - nVarOrig;
  b_qpoptions = *qpoptions;
  b_driver(Hessian, grad, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
  qpoptions->MaxIterations = idx_max;
  if (TrialState->state != -6) {
    real_T qpfvalLinearExcess;
    real_T qpfvalQuadExcess;
    int32_T iIneqEnd;
    idx_max = (WorkingSet->nVarMax - nVarOrig) - 1;
    if (idx_max < 1) {
      qpfvalLinearExcess = 0.0;
      qpfvalQuadExcess = 0.0;
    } else {
      n_t = (ptrdiff_t)idx_max;
      incx_t = (ptrdiff_t)1;
      qpfvalLinearExcess =
          dasum(&n_t, &TrialState->xstar->data[nVarOrig], &incx_t);
      n_t = (ptrdiff_t)idx_max;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      qpfvalQuadExcess = ddot(&n_t, &TrialState->xstar->data[nVarOrig], &incx_t,
                              &TrialState->xstar->data[nVarOrig], &incy_t);
    }
    rho = (TrialState->fstar - rho * qpfvalLinearExcess) -
          beta / 2.0 * qpfvalQuadExcess;
    qpfvalLinearExcess = MeritFunction->penaltyParam;
    beta = MeritFunction->linearizedConstrViol;
    if (idx_max < 1) {
      MeritFunction->linearizedConstrViol = 0.0;
    } else {
      n_t = (ptrdiff_t)idx_max;
      incx_t = (ptrdiff_t)1;
      MeritFunction->linearizedConstrViol =
          dasum(&n_t, &TrialState->xstar->data[nVarOrig], &incx_t);
    }
    beta -= MeritFunction->linearizedConstrViol;
    if ((beta > 2.2204460492503131E-16) && (rho > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        qpfvalLinearExcess = 1.0;
      } else {
        qpfvalLinearExcess = 1.5;
      }
      qpfvalLinearExcess = qpfvalLinearExcess * rho / beta;
    }
    if (qpfvalLinearExcess < MeritFunction->penaltyParam) {
      MeritFunction->phi = TrialState->sqpFval;
      if (MeritFunction->initFval - TrialState->sqpFval >
          (real_T)MeritFunction->nPenaltyDecreases * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) >
            TrialState->sqpIterations) {
          MeritFunction->threshold *= 10.0;
        }
        MeritFunction->penaltyParam =
            muDoubleScalarMax(qpfvalLinearExcess, 1.0E-10);
      } else {
        MeritFunction->phi =
            TrialState->sqpFval + MeritFunction->penaltyParam * 0.0;
      }
    } else {
      MeritFunction->penaltyParam =
          muDoubleScalarMax(qpfvalLinearExcess, 1.0E-10);
      MeritFunction->phi =
          TrialState->sqpFval + MeritFunction->penaltyParam * 0.0;
    }
    MeritFunction->phiPrimePlus =
        muDoubleScalarMin(rho - MeritFunction->penaltyParam * 0.0, 0.0);
    idx_max = WorkingSet->isActiveIdx[2];
    iIneqEnd = WorkingSet->nActiveConstr;
    for (idx = idx_max; idx <= iIneqEnd; idx++) {
      if (WorkingSet->Wid->data[idx - 1] == 3) {
        TrialState->lambda->data[idx - 1] *=
            (real_T)memspace->workspace_int
                ->data[WorkingSet->Wlocalidx->data[idx - 1] - 1];
      }
    }
  }
  QPObjective->nvar = nVarOrig;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 3;
  setProblemType(WorkingSet, 3);
  sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr, WorkingSet->sizes,
               WorkingSet->isActiveIdx, WorkingSet->Wid, WorkingSet->Wlocalidx,
               memspace->workspace_double);
}

/* End of code generation (relaxed.c) */
