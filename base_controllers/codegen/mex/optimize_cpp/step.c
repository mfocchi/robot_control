/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * step.c
 *
 * Code generation for function 'step'
 *
 */

/* Include files */
#include "step.h"
#include "addAineqConstr.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "relaxed.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
boolean_T step(int32_T *STEP_TYPE, emxArray_real_T *Hessian,
               const emxArray_real_T *lb, const emxArray_real_T *ub,
               g_struct_T *TrialState, struct_T *MeritFunction,
               f_struct_T *memspace, h_struct_T *WorkingSet,
               c_struct_T *QRManager, d_struct_T *CholManager,
               e_struct_T *QPObjective, l_struct_T *qpoptions)
{
  ptrdiff_t b_incx_t;
  ptrdiff_t b_incy_t;
  ptrdiff_t c_incx_t;
  ptrdiff_t c_incy_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emxArray_real_T *r;
  emxArray_real_T *r3;
  l_struct_T b_qpoptions;
  const real_T *lb_data;
  const real_T *ub_data;
  real_T alpha1;
  real_T beta1;
  real_T lenSOC;
  real_T oldDirIdx;
  real_T *Hessian_data;
  real_T *r1;
  int32_T i;
  int32_T idx;
  int32_T idxIneqOffset;
  int32_T idx_Aineq;
  int32_T idx_global;
  int32_T idx_lower;
  int32_T mIneq;
  int32_T nVar_tmp_tmp;
  char_T TRANSA;
  boolean_T checkBoundViolation;
  boolean_T stepSuccess;
  ub_data = ub->data;
  lb_data = lb->data;
  Hessian_data = Hessian->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  stepSuccess = true;
  checkBoundViolation = true;
  nVar_tmp_tmp = WorkingSet->nVar;
  if (*STEP_TYPE != 3) {
    n_t = (ptrdiff_t)WorkingSet->nVar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &TrialState->xstarsqp->data[0], &incx_t,
          &TrialState->xstar->data[0], &incy_t);
  } else if (WorkingSet->nVar >= 1) {
    n_t = (ptrdiff_t)WorkingSet->nVar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &TrialState->xstar->data[0], &incx_t,
          &TrialState->searchDir->data[0], &incy_t);
  }
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  alpha1 = 1.0;
  beta1 = 1.0;
  TRANSA = 'T';
  b_incx_t = (ptrdiff_t)1;
  b_incy_t = (ptrdiff_t)1;
  c_incx_t = (ptrdiff_t)1;
  c_incy_t = (ptrdiff_t)1;
  emxInit_real_T(&r, 1);
  int32_T exitg1;
  boolean_T guard1;
  do {
    int32_T idx_upper;
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
    case 1: {
      b_qpoptions = *qpoptions;
      b_driver(Hessian, TrialState->grad, TrialState, memspace, WorkingSet,
               QRManager, CholManager, QPObjective, &b_qpoptions,
               qpoptions->MaxIterations);
      if (TrialState->state > 0) {
        real_T constrViolationIneq;
        real_T penaltyParamTrial;
        mIneq = WorkingSet->sizes[2];
        penaltyParamTrial = MeritFunction->penaltyParam;
        constrViolationIneq = 0.0;
        for (idx = 0; idx < mIneq; idx++) {
          lenSOC = TrialState->cIneq->data[idx];
          if (lenSOC > 0.0) {
            constrViolationIneq += lenSOC;
          }
        }
        oldDirIdx = MeritFunction->linearizedConstrViol;
        MeritFunction->linearizedConstrViol = 0.0;
        oldDirIdx += constrViolationIneq;
        if ((oldDirIdx > 2.2204460492503131E-16) && (TrialState->fstar > 0.0)) {
          if (TrialState->sqpFval == 0.0) {
            lenSOC = 1.0;
          } else {
            lenSOC = 1.5;
          }
          penaltyParamTrial = lenSOC * TrialState->fstar / oldDirIdx;
        }
        if (penaltyParamTrial < MeritFunction->penaltyParam) {
          MeritFunction->phi =
              TrialState->sqpFval + penaltyParamTrial * constrViolationIneq;
          if ((MeritFunction->initFval +
               penaltyParamTrial * MeritFunction->initConstrViolationIneq) -
                  MeritFunction->phi >
              (real_T)MeritFunction->nPenaltyDecreases *
                  MeritFunction->threshold) {
            MeritFunction->nPenaltyDecreases++;
            if ((MeritFunction->nPenaltyDecreases << 1) >
                TrialState->sqpIterations) {
              MeritFunction->threshold *= 10.0;
            }
            MeritFunction->penaltyParam =
                muDoubleScalarMax(penaltyParamTrial, 1.0E-10);
          } else {
            MeritFunction->phi =
                TrialState->sqpFval +
                MeritFunction->penaltyParam * constrViolationIneq;
          }
        } else {
          MeritFunction->penaltyParam =
              muDoubleScalarMax(penaltyParamTrial, 1.0E-10);
          MeritFunction->phi =
              TrialState->sqpFval +
              MeritFunction->penaltyParam * constrViolationIneq;
        }
        MeritFunction->phiPrimePlus =
            muDoubleScalarMin(TrialState->fstar - MeritFunction->penaltyParam *
                                                      constrViolationIneq,
                              0.0);
      }
      sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                   WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                   WorkingSet->Wlocalidx, memspace->workspace_double);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        if (nVar_tmp_tmp >= 1) {
          n_t = (ptrdiff_t)nVar_tmp_tmp;
          m_t = (ptrdiff_t)1;
          lda_t = (ptrdiff_t)1;
          dcopy(&n_t, &TrialState->xstar->data[0], &m_t,
                &TrialState->delta_x->data[0], &lda_t);
        }
        guard1 = true;
      }
    } break;
    case 2:
      idx_lower = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
      idx_upper = idx_lower + 1;
      idx_Aineq = WorkingSet->nActiveConstr;
      for (idx_global = idx_upper; idx_global <= idx_Aineq; idx_global++) {
        WorkingSet->isActiveConstr
            ->data[(WorkingSet->isActiveIdx
                        [WorkingSet->Wid->data[idx_global - 1] - 1] +
                    WorkingSet->Wlocalidx->data[idx_global - 1]) -
                   2] = false;
      }
      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = idx_lower;
      i = r->size[0];
      r->size[0] = TrialState->xstar->size[0];
      emxEnsureCapacity_real_T(r, i);
      r1 = r->data;
      idx_lower = TrialState->xstar->size[0];
      for (i = 0; i < idx_lower; i++) {
        r1[i] = TrialState->xstar->data[i];
      }
      idxIneqOffset = WorkingSet->sizes[3];
      idx_Aineq = WorkingSet->sizes[4];
      for (idx = 0; idx < idxIneqOffset; idx++) {
        lenSOC = WorkingSet->lb->data[WorkingSet->indexLB->data[idx] - 1];
        if (-r1[WorkingSet->indexLB->data[idx] - 1] > lenSOC) {
          if (muDoubleScalarIsInf(
                  ub_data[WorkingSet->indexLB->data[idx] - 1])) {
            r1[WorkingSet->indexLB->data[idx] - 1] =
                -lenSOC + muDoubleScalarAbs(lenSOC);
          } else {
            r1[WorkingSet->indexLB->data[idx] - 1] =
                (WorkingSet->ub->data[WorkingSet->indexLB->data[idx] - 1] -
                 lenSOC) /
                2.0;
          }
        }
      }
      for (idx = 0; idx < idx_Aineq; idx++) {
        lenSOC = WorkingSet->ub->data[WorkingSet->indexUB->data[idx] - 1];
        if (r1[WorkingSet->indexUB->data[idx] - 1] > lenSOC) {
          if (muDoubleScalarIsInf(
                  lb_data[WorkingSet->indexUB->data[idx] - 1])) {
            r1[WorkingSet->indexUB->data[idx] - 1] =
                lenSOC - muDoubleScalarAbs(lenSOC);
          } else {
            r1[WorkingSet->indexUB->data[idx] - 1] =
                (lenSOC -
                 WorkingSet->lb->data[WorkingSet->indexUB->data[idx] - 1]) /
                2.0;
          }
        }
      }
      i = TrialState->xstar->size[0];
      TrialState->xstar->size[0] = r->size[0];
      emxEnsureCapacity_real_T(TrialState->xstar, i);
      idx_lower = r->size[0];
      for (i = 0; i < idx_lower; i++) {
        TrialState->xstar->data[i] = r1[i];
      }
      relaxed(Hessian, TrialState->grad, TrialState, MeritFunction, memspace,
              WorkingSet, QRManager, CholManager, QPObjective, qpoptions);
      if (nVar_tmp_tmp >= 1) {
        n_t = (ptrdiff_t)nVar_tmp_tmp;
        m_t = (ptrdiff_t)1;
        lda_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->xstar->data[0], &m_t,
              &TrialState->delta_x->data[0], &lda_t);
      }
      guard1 = true;
      break;
    default: {
      __m128d r2;
      __m128d r4;
      int32_T mConstrMax;
      int32_T nWIneq_old;
      int32_T nWLower_old;
      int32_T nWUpper_old;
      nWIneq_old = WorkingSet->nWConstr[2];
      nWLower_old = WorkingSet->nWConstr[3];
      nWUpper_old = WorkingSet->nWConstr[4];
      i = WorkingSet->nVar;
      mConstrMax = WorkingSet->mConstrMax;
      n_t = (ptrdiff_t)WorkingSet->nVar;
      dcopy(&n_t, &TrialState->xstarsqp_old->data[0], &incx_t,
            &TrialState->xstarsqp->data[0], &incy_t);
      for (idx_Aineq = 0; idx_Aineq < i; idx_Aineq++) {
        TrialState->socDirection->data[idx_Aineq] =
            TrialState->xstar->data[idx_Aineq];
      }
      if (WorkingSet->mConstrMax >= 1) {
        n_t = (ptrdiff_t)WorkingSet->mConstrMax;
        m_t = (ptrdiff_t)1;
        lda_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->lambda->data[0], &m_t,
              &TrialState->lambdaStopTest->data[0], &lda_t);
      }
      mIneq = WorkingSet->sizes[2];
      idxIneqOffset = WorkingSet->isActiveIdx[2];
      idx_Aineq = (mIneq / 2) << 1;
      idx_lower = idx_Aineq - 2;
      for (idx = 0; idx <= idx_lower; idx += 2) {
        r2 = _mm_loadu_pd(&TrialState->cIneq->data[idx]);
        _mm_storeu_pd(&WorkingSet->bineq->data[idx],
                      _mm_mul_pd(r2, _mm_set1_pd(-1.0)));
      }
      for (idx = idx_Aineq; idx < mIneq; idx++) {
        WorkingSet->bineq->data[idx] = -TrialState->cIneq->data[idx];
      }
      m_t = (ptrdiff_t)WorkingSet->nVar;
      n_t = (ptrdiff_t)WorkingSet->sizes[2];
      lda_t = (ptrdiff_t)WorkingSet->ldA;
      dgemv(&TRANSA, &m_t, &n_t, &alpha1, &WorkingSet->Aineq->data[0], &lda_t,
            &TrialState->searchDir->data[0], &b_incx_t, &beta1,
            &WorkingSet->bineq->data[0], &b_incy_t);
      idx_Aineq = 1;
      idx_lower = WorkingSet->sizes[2] + 1;
      idx_upper = (WorkingSet->sizes[2] + WorkingSet->sizes[3]) + 1;
      idx_global = WorkingSet->nActiveConstr;
      for (idx = idxIneqOffset; idx <= idx_global; idx++) {
        switch (WorkingSet->Wid->data[idx - 1]) {
        case 3:
          mIneq = idx_Aineq;
          idx_Aineq++;
          WorkingSet->bwset->data[idx - 1] =
              WorkingSet->bineq->data[WorkingSet->Wlocalidx->data[idx - 1] - 1];
          break;
        case 4:
          mIneq = idx_lower;
          idx_lower++;
          break;
        default:
          mIneq = idx_upper;
          idx_upper++;
          break;
        }
        TrialState->workingset_old->data[mIneq - 1] =
            WorkingSet->Wlocalidx->data[idx - 1];
      }
      n_t = (ptrdiff_t)WorkingSet->nVar;
      dcopy(&n_t, &TrialState->xstarsqp->data[0], &c_incx_t,
            &TrialState->xstar->data[0], &c_incy_t);
      idx_global = r->size[0];
      r->size[0] = TrialState->grad->size[0];
      emxEnsureCapacity_real_T(r, idx_global);
      r1 = r->data;
      idx_lower = TrialState->grad->size[0];
      for (idx_global = 0; idx_global < idx_lower; idx_global++) {
        r1[idx_global] = TrialState->grad->data[idx_global];
      }
      b_qpoptions = *qpoptions;
      b_driver(Hessian, r, TrialState, memspace, WorkingSet, QRManager,
               CholManager, QPObjective, &b_qpoptions,
               qpoptions->MaxIterations);
      idx_Aineq = (i / 2) << 1;
      idx_lower = idx_Aineq - 2;
      for (idx = 0; idx <= idx_lower; idx += 2) {
        r2 = _mm_loadu_pd(&TrialState->socDirection->data[idx]);
        r4 = _mm_loadu_pd(&TrialState->xstar->data[idx]);
        _mm_storeu_pd(&TrialState->socDirection->data[idx], _mm_sub_pd(r4, r2));
        _mm_storeu_pd(&TrialState->xstar->data[idx], r2);
      }
      for (idx = idx_Aineq; idx < i; idx++) {
        lenSOC = TrialState->socDirection->data[idx];
        oldDirIdx = lenSOC;
        lenSOC = TrialState->xstar->data[idx] - lenSOC;
        TrialState->socDirection->data[idx] = lenSOC;
        TrialState->xstar->data[idx] = oldDirIdx;
      }
      if (i < 1) {
        lenSOC = 0.0;
        oldDirIdx = 0.0;
      } else {
        n_t = (ptrdiff_t)i;
        m_t = (ptrdiff_t)1;
        lenSOC = dnrm2(&n_t, &TrialState->socDirection->data[0], &m_t);
        n_t = (ptrdiff_t)i;
        m_t = (ptrdiff_t)1;
        oldDirIdx = dnrm2(&n_t, &TrialState->xstar->data[0], &m_t);
      }
      stepSuccess = (lenSOC <= 2.0 * oldDirIdx);
      mIneq = WorkingSet->sizes[2];
      idxIneqOffset = WorkingSet->sizes[3];
      idx_Aineq = (mIneq / 2) << 1;
      idx_lower = idx_Aineq - 2;
      for (idx = 0; idx <= idx_lower; idx += 2) {
        r2 = _mm_loadu_pd(&TrialState->cIneq->data[idx]);
        _mm_storeu_pd(&WorkingSet->bineq->data[idx],
                      _mm_mul_pd(r2, _mm_set1_pd(-1.0)));
      }
      for (idx = idx_Aineq; idx < mIneq; idx++) {
        WorkingSet->bineq->data[idx] = -TrialState->cIneq->data[idx];
      }
      if (!stepSuccess) {
        idx_upper = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
        i = WorkingSet->nActiveConstr;
        for (idx_global = idx_upper; idx_global <= i; idx_global++) {
          WorkingSet->isActiveConstr
              ->data[(WorkingSet->isActiveIdx
                          [WorkingSet->Wid->data[idx_global - 1] - 1] +
                      WorkingSet->Wlocalidx->data[idx_global - 1]) -
                     2] = false;
        }
        WorkingSet->nWConstr[2] = 0;
        WorkingSet->nWConstr[3] = 0;
        WorkingSet->nWConstr[4] = 0;
        WorkingSet->nActiveConstr =
            WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
        for (idx = 0; idx < nWIneq_old; idx++) {
          addAineqConstr(WorkingSet, TrialState->workingset_old->data[idx]);
        }
        for (idx = 0; idx < nWLower_old; idx++) {
          addBoundToActiveSetMatrix_(
              WorkingSet, 4, TrialState->workingset_old->data[idx + mIneq]);
        }
        for (idx = 0; idx < nWUpper_old; idx++) {
          addBoundToActiveSetMatrix_(
              WorkingSet, 5,
              TrialState->workingset_old->data[(idx + mIneq) + idxIneqOffset]);
        }
        if (mConstrMax >= 1) {
          n_t = (ptrdiff_t)mConstrMax;
          m_t = (ptrdiff_t)1;
          lda_t = (ptrdiff_t)1;
          dcopy(&n_t, &TrialState->lambdaStopTest->data[0], &m_t,
                &TrialState->lambda->data[0], &lda_t);
        }
      } else {
        sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                     WorkingSet->sizes, WorkingSet->isActiveIdx,
                     WorkingSet->Wid, WorkingSet->Wlocalidx,
                     memspace->workspace_double);
      }
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState->state != -6)) {
        idx_Aineq = (nVar_tmp_tmp / 2) << 1;
        idx_lower = idx_Aineq - 2;
        for (idx = 0; idx <= idx_lower; idx += 2) {
          r2 = _mm_loadu_pd(&TrialState->xstar->data[idx]);
          r4 = _mm_loadu_pd(&TrialState->socDirection->data[idx]);
          _mm_storeu_pd(&TrialState->delta_x->data[idx], _mm_add_pd(r2, r4));
        }
        for (idx = idx_Aineq; idx < nVar_tmp_tmp; idx++) {
          TrialState->delta_x->data[idx] = TrialState->xstar->data[idx] +
                                           TrialState->socDirection->data[idx];
        }
      }
      guard1 = true;
    } break;
    }
    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        idx_upper = Hessian->size[0] - 1;
        oldDirIdx = 0.0;
        lenSOC = 1.0;
        for (idx = 0; idx <= idx_upper; idx++) {
          oldDirIdx = muDoubleScalarMax(
              oldDirIdx, muDoubleScalarAbs(TrialState->grad->data[idx]));
          lenSOC = muDoubleScalarMax(
              lenSOC, muDoubleScalarAbs(TrialState->xstar->data[idx]));
        }
        lenSOC = muDoubleScalarMax(2.2204460492503131E-16, oldDirIdx / lenSOC);
        for (mIneq = 0; mIneq <= idx_upper; mIneq++) {
          idx_lower = (idx_upper + 1) * mIneq;
          for (idxIneqOffset = 0; idxIneqOffset < mIneq; idxIneqOffset++) {
            Hessian_data[idx_lower + idxIneqOffset] = 0.0;
          }
          Hessian_data[mIneq + Hessian->size[0] * mIneq] = lenSOC;
          idx_Aineq = idx_lower + mIneq;
          idx_lower = idx_upper - mIneq;
          for (idxIneqOffset = 0; idxIneqOffset < idx_lower; idxIneqOffset++) {
            Hessian_data[(idx_Aineq + idxIneqOffset) + 1] = 0.0;
          }
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    idxIneqOffset = WorkingSet->sizes[3];
    idx_Aineq = WorkingSet->sizes[4];
    i = r->size[0];
    r->size[0] = TrialState->delta_x->size[0];
    emxEnsureCapacity_real_T(r, i);
    r1 = r->data;
    idx_lower = TrialState->delta_x->size[0];
    for (i = 0; i < idx_lower; i++) {
      r1[i] = TrialState->delta_x->data[i];
    }
    emxInit_real_T(&r3, 1);
    i = r3->size[0];
    r3->size[0] = TrialState->xstar->size[0];
    emxEnsureCapacity_real_T(r3, i);
    Hessian_data = r3->data;
    idx_lower = TrialState->xstar->size[0];
    for (i = 0; i < idx_lower; i++) {
      Hessian_data[i] = TrialState->xstar->data[i];
    }
    for (idx = 0; idx < idxIneqOffset; idx++) {
      lenSOC = r1[WorkingSet->indexLB->data[idx] - 1];
      oldDirIdx =
          (TrialState->xstarsqp->data[WorkingSet->indexLB->data[idx] - 1] +
           lenSOC) -
          lb_data[WorkingSet->indexLB->data[idx] - 1];
      if (oldDirIdx < 0.0) {
        r1[WorkingSet->indexLB->data[idx] - 1] = lenSOC - oldDirIdx;
        Hessian_data[WorkingSet->indexLB->data[idx] - 1] -= oldDirIdx;
      }
    }
    for (idx = 0; idx < idx_Aineq; idx++) {
      lenSOC = r1[WorkingSet->indexUB->data[idx] - 1];
      oldDirIdx =
          (ub_data[WorkingSet->indexUB->data[idx] - 1] -
           TrialState->xstarsqp->data[WorkingSet->indexUB->data[idx] - 1]) -
          lenSOC;
      if (oldDirIdx < 0.0) {
        r1[WorkingSet->indexUB->data[idx] - 1] = lenSOC + oldDirIdx;
        Hessian_data[WorkingSet->indexUB->data[idx] - 1] += oldDirIdx;
      }
    }
    i = TrialState->delta_x->size[0];
    TrialState->delta_x->size[0] = r->size[0];
    emxEnsureCapacity_real_T(TrialState->delta_x, i);
    idx_lower = r->size[0];
    for (i = 0; i < idx_lower; i++) {
      TrialState->delta_x->data[i] = r1[i];
    }
    i = TrialState->xstar->size[0];
    TrialState->xstar->size[0] = r3->size[0];
    emxEnsureCapacity_real_T(TrialState->xstar, i);
    idx_lower = r3->size[0];
    for (i = 0; i < idx_lower; i++) {
      TrialState->xstar->data[i] = Hessian_data[i];
    }
    emxFree_real_T(&r3);
  }
  emxFree_real_T(&r);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return stepSuccess;
}

/* End of code generation (step.c) */
