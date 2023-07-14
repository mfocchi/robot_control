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
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "removeConstr.h"
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
             e_struct_T *QPObjective, l_struct_T *qpoptions)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  l_struct_T b_qpoptions;
  const real_T *Hessian_data;
  const real_T *grad_data;
  real_T beta;
  real_T qpfvalLinearExcess;
  real_T qpfvalQuadExcess;
  real_T rho;
  int32_T idx;
  int32_T idx_max;
  int32_T mIneq;
  int32_T nActiveLBArtificial;
  int32_T nVarOrig;
  char_T TRANSA;
  grad_data = grad->data;
  Hessian_data = Hessian->data;
  nVarOrig = WorkingSet->nVar;
  mIneq = WorkingSet->sizes[2];
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
  idx_max = WorkingSet->sizes[2];
  n_t = (ptrdiff_t)WorkingSet->sizes[2];
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  dcopy(&n_t, &WorkingSet->bineq->data[0], &incx_t,
        &memspace->workspace_double->data[0], &incy_t);
  if (WorkingSet->sizes[2] >= 1) {
    qpfvalLinearExcess = 1.0;
    qpfvalQuadExcess = -1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)nVarOrig;
    n_t = (ptrdiff_t)WorkingSet->sizes[2];
    lda_t = (ptrdiff_t)WorkingSet->ldA;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dgemv(&TRANSA, &m_t, &n_t, &qpfvalLinearExcess, &WorkingSet->Aineq->data[0],
          &lda_t, &TrialState->xstar->data[0], &incx_t, &qpfvalQuadExcess,
          &memspace->workspace_double->data[0], &incy_t);
  }
  for (idx = 0; idx < idx_max; idx++) {
    TrialState->xstar->data[nVarOrig + idx] =
        (real_T)(memspace->workspace_double->data[idx] > 0.0) *
        memspace->workspace_double->data[idx];
  }
  idx_max = qpoptions->MaxIterations;
  qpoptions->MaxIterations =
      (qpoptions->MaxIterations + WorkingSet->nVar) - nVarOrig;
  b_qpoptions = *qpoptions;
  b_driver(Hessian, grad, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
  qpoptions->MaxIterations = idx_max;
  idx_max = WorkingSet->sizes[2];
  nActiveLBArtificial = 0;
  for (idx = 0; idx < idx_max; idx++) {
    boolean_T tf;
    tf = WorkingSet->isActiveConstr
             ->data[(((WorkingSet->isActiveIdx[3] + WorkingSet->sizes[3]) -
                      idx_max) +
                     idx) -
                    1];
    memspace->workspace_int->data[idx] = tf;
    nActiveLBArtificial += tf;
  }
  if (TrialState->state != -6) {
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
    qpfvalQuadExcess = (TrialState->fstar - rho * qpfvalLinearExcess) -
                       beta / 2.0 * qpfvalQuadExcess;
    beta = MeritFunction->penaltyParam;
    rho = 0.0;
    for (idx = 0; idx < mIneq; idx++) {
      qpfvalLinearExcess = TrialState->cIneq->data[idx];
      if (qpfvalLinearExcess > 0.0) {
        rho += qpfvalLinearExcess;
      }
    }
    qpfvalLinearExcess = MeritFunction->linearizedConstrViol;
    if (idx_max < 1) {
      MeritFunction->linearizedConstrViol = 0.0;
    } else {
      n_t = (ptrdiff_t)idx_max;
      incx_t = (ptrdiff_t)1;
      MeritFunction->linearizedConstrViol =
          dasum(&n_t, &TrialState->xstar->data[nVarOrig], &incx_t);
    }
    qpfvalLinearExcess =
        (rho + qpfvalLinearExcess) - MeritFunction->linearizedConstrViol;
    if ((qpfvalLinearExcess > 2.2204460492503131E-16) &&
        (qpfvalQuadExcess > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        beta = 1.0;
      } else {
        beta = 1.5;
      }
      beta = beta * qpfvalQuadExcess / qpfvalLinearExcess;
    }
    if (beta < MeritFunction->penaltyParam) {
      MeritFunction->phi = TrialState->sqpFval + beta * rho;
      if ((MeritFunction->initFval +
           beta * MeritFunction->initConstrViolationIneq) -
              MeritFunction->phi >
          (real_T)MeritFunction->nPenaltyDecreases * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) >
            TrialState->sqpIterations) {
          MeritFunction->threshold *= 10.0;
        }
        MeritFunction->penaltyParam = muDoubleScalarMax(beta, 1.0E-10);
      } else {
        MeritFunction->phi =
            TrialState->sqpFval + MeritFunction->penaltyParam * rho;
      }
    } else {
      MeritFunction->penaltyParam = muDoubleScalarMax(beta, 1.0E-10);
      MeritFunction->phi =
          TrialState->sqpFval + MeritFunction->penaltyParam * rho;
    }
    MeritFunction->phiPrimePlus = muDoubleScalarMin(
        qpfvalQuadExcess - MeritFunction->penaltyParam * rho, 0.0);
    idx_max = WorkingSet->isActiveIdx[2];
    mIneq = WorkingSet->nActiveConstr;
    for (idx = idx_max; idx <= mIneq; idx++) {
      if (WorkingSet->Wid->data[idx - 1] == 3) {
        TrialState->lambda->data[idx - 1] *=
            (real_T)memspace->workspace_int
                ->data[WorkingSet->Wlocalidx->data[idx - 1] - 1];
      }
    }
  }
  idx_max = WorkingSet->sizes[0];
  mIneq = WorkingSet->sizes[3] - WorkingSet->sizes[2];
  idx = WorkingSet->nActiveConstr;
  while ((idx > idx_max) && (nActiveLBArtificial > 0)) {
    if ((WorkingSet->Wid->data[idx - 1] == 4) &&
        (WorkingSet->Wlocalidx->data[idx - 1] > mIneq)) {
      int32_T tmp_tmp;
      tmp_tmp = WorkingSet->nActiveConstr - 1;
      qpfvalLinearExcess = TrialState->lambda->data[tmp_tmp];
      TrialState->lambda->data[tmp_tmp] = 0.0;
      TrialState->lambda->data[idx - 1] = qpfvalLinearExcess;
      removeConstr(WorkingSet, idx);
      nActiveLBArtificial--;
    }
    idx--;
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
