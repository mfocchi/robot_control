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
#include "driver1.h"
#include "optimize_cpp_mpc_data.h"
#include "optimize_cpp_mpc_emxutil.h"
#include "optimize_cpp_mpc_internal_types.h"
#include "optimize_cpp_mpc_types.h"
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
               e_struct_T *QPObjective, k_struct_T *qpoptions)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emxArray_real_T *r;
  emxArray_real_T *r2;
  k_struct_T b_qpoptions;
  const real_T *lb_data;
  const real_T *ub_data;
  real_T lenSOC;
  real_T oldDirIdx;
  real_T *Hessian_data;
  real_T *r1;
  int32_T iH0;
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T idx_col;
  int32_T k;
  int32_T mConstrMax;
  int32_T nVar_tmp_tmp;
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
    if (WorkingSet->nVar >= 1) {
      n_t = (ptrdiff_t)WorkingSet->nVar;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &TrialState->xstarsqp->data[0], &incx_t,
            &TrialState->xstar->data[0], &incy_t);
    }
  } else if (WorkingSet->nVar >= 1) {
    n_t = (ptrdiff_t)WorkingSet->nVar;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &TrialState->xstar->data[0], &incx_t,
          &TrialState->searchDir->data[0], &incy_t);
  }
  emxInit_real_T(&r, 1);
  int32_T exitg1;
  boolean_T guard1;
  do {
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
    case 1:
      b_qpoptions = *qpoptions;
      b_driver(Hessian, TrialState->grad, TrialState, memspace, WorkingSet,
               QRManager, CholManager, QPObjective, &b_qpoptions,
               qpoptions->MaxIterations);
      if (TrialState->state > 0) {
        oldDirIdx = MeritFunction->penaltyParam;
        lenSOC = MeritFunction->linearizedConstrViol;
        MeritFunction->linearizedConstrViol = 0.0;
        if ((lenSOC > 2.2204460492503131E-16) && (TrialState->fstar > 0.0)) {
          if (TrialState->sqpFval == 0.0) {
            oldDirIdx = 1.0;
          } else {
            oldDirIdx = 1.5;
          }
          oldDirIdx = oldDirIdx * TrialState->fstar / lenSOC;
        }
        if (oldDirIdx < MeritFunction->penaltyParam) {
          MeritFunction->phi = TrialState->sqpFval;
          if (MeritFunction->initFval - TrialState->sqpFval >
              (real_T)MeritFunction->nPenaltyDecreases *
                  MeritFunction->threshold) {
            MeritFunction->nPenaltyDecreases++;
            if ((MeritFunction->nPenaltyDecreases << 1) >
                TrialState->sqpIterations) {
              MeritFunction->threshold *= 10.0;
            }
            MeritFunction->penaltyParam = muDoubleScalarMax(oldDirIdx, 1.0E-10);
          } else {
            MeritFunction->phi =
                TrialState->sqpFval + MeritFunction->penaltyParam * 0.0;
          }
        } else {
          MeritFunction->penaltyParam = muDoubleScalarMax(oldDirIdx, 1.0E-10);
          MeritFunction->phi =
              TrialState->sqpFval + MeritFunction->penaltyParam * 0.0;
        }
        MeritFunction->phiPrimePlus = muDoubleScalarMin(
            TrialState->fstar - MeritFunction->penaltyParam * 0.0, 0.0);
      }
      sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                   WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                   WorkingSet->Wlocalidx, memspace->workspace_double);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        if (nVar_tmp_tmp >= 1) {
          n_t = (ptrdiff_t)nVar_tmp_tmp;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dcopy(&n_t, &TrialState->xstar->data[0], &incx_t,
                &TrialState->delta_x->data[0], &incy_t);
        }
        guard1 = true;
      }
      break;
    case 2:
      iH0 = WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
      idxStartIneq = iH0 + 1;
      idxEndIneq = WorkingSet->nActiveConstr;
      for (mConstrMax = idxStartIneq; mConstrMax <= idxEndIneq; mConstrMax++) {
        WorkingSet->isActiveConstr
            ->data[(WorkingSet->isActiveIdx
                        [WorkingSet->Wid->data[mConstrMax - 1] - 1] +
                    WorkingSet->Wlocalidx->data[mConstrMax - 1]) -
                   2] = false;
      }
      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = iH0;
      idx_col = r->size[0];
      r->size[0] = TrialState->xstar->size[0];
      emxEnsureCapacity_real_T(r, idx_col);
      r1 = r->data;
      iH0 = TrialState->xstar->size[0];
      for (idx_col = 0; idx_col < iH0; idx_col++) {
        r1[idx_col] = TrialState->xstar->data[idx_col];
      }
      idxStartIneq = WorkingSet->sizes[3] - 1;
      idxEndIneq = WorkingSet->sizes[4] - 1;
      if (lb->size[1] != 0) {
        if (ub->size[1] == 0) {
          for (iH0 = 0; iH0 <= idxStartIneq; iH0++) {
            lenSOC = WorkingSet->lb->data[WorkingSet->indexLB->data[iH0] - 1];
            if (-r1[WorkingSet->indexLB->data[iH0] - 1] > lenSOC) {
              r1[WorkingSet->indexLB->data[iH0] - 1] =
                  -lenSOC + muDoubleScalarAbs(lenSOC);
            }
          }
        } else {
          for (iH0 = 0; iH0 <= idxStartIneq; iH0++) {
            lenSOC = WorkingSet->lb->data[WorkingSet->indexLB->data[iH0] - 1];
            if (-r1[WorkingSet->indexLB->data[iH0] - 1] > lenSOC) {
              if (muDoubleScalarIsInf(
                      ub_data[WorkingSet->indexLB->data[iH0] - 1])) {
                r1[WorkingSet->indexLB->data[iH0] - 1] =
                    -lenSOC + muDoubleScalarAbs(lenSOC);
              } else {
                r1[WorkingSet->indexLB->data[iH0] - 1] =
                    (WorkingSet->ub->data[WorkingSet->indexLB->data[iH0] - 1] -
                     lenSOC) /
                    2.0;
              }
            }
          }
        }
      }
      if (ub->size[1] != 0) {
        if (lb->size[1] == 0) {
          for (iH0 = 0; iH0 <= idxEndIneq; iH0++) {
            lenSOC = WorkingSet->ub->data[WorkingSet->indexUB->data[iH0] - 1];
            if (r1[WorkingSet->indexUB->data[iH0] - 1] > lenSOC) {
              r1[WorkingSet->indexUB->data[iH0] - 1] =
                  lenSOC - muDoubleScalarAbs(lenSOC);
            }
          }
        } else {
          for (iH0 = 0; iH0 <= idxEndIneq; iH0++) {
            lenSOC = WorkingSet->ub->data[WorkingSet->indexUB->data[iH0] - 1];
            if (r1[WorkingSet->indexUB->data[iH0] - 1] > lenSOC) {
              if (muDoubleScalarIsInf(
                      lb_data[WorkingSet->indexUB->data[iH0] - 1])) {
                r1[WorkingSet->indexUB->data[iH0] - 1] =
                    lenSOC - muDoubleScalarAbs(lenSOC);
              } else {
                r1[WorkingSet->indexUB->data[iH0] - 1] =
                    (lenSOC -
                     WorkingSet->lb->data[WorkingSet->indexUB->data[iH0] - 1]) /
                    2.0;
              }
            }
          }
        }
      }
      idx_col = TrialState->xstar->size[0];
      TrialState->xstar->size[0] = r->size[0];
      emxEnsureCapacity_real_T(TrialState->xstar, idx_col);
      iH0 = r->size[0];
      for (idx_col = 0; idx_col < iH0; idx_col++) {
        TrialState->xstar->data[idx_col] = r1[idx_col];
      }
      relaxed(Hessian, TrialState->grad, TrialState, MeritFunction, memspace,
              WorkingSet, QRManager, CholManager, QPObjective, qpoptions);
      if (nVar_tmp_tmp >= 1) {
        n_t = (ptrdiff_t)nVar_tmp_tmp;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->xstar->data[0], &incx_t,
              &TrialState->delta_x->data[0], &incy_t);
      }
      guard1 = true;
      break;
    default: {
      __m128d r3;
      __m128d r4;
      idx_col = WorkingSet->nVar;
      mConstrMax = WorkingSet->mConstrMax;
      if (WorkingSet->nVar >= 1) {
        n_t = (ptrdiff_t)WorkingSet->nVar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->xstarsqp_old->data[0], &incx_t,
              &TrialState->xstarsqp->data[0], &incy_t);
      }
      for (idxStartIneq = 0; idxStartIneq < idx_col; idxStartIneq++) {
        TrialState->socDirection->data[idxStartIneq] =
            TrialState->xstar->data[idxStartIneq];
      }
      if (WorkingSet->mConstrMax >= 1) {
        n_t = (ptrdiff_t)WorkingSet->mConstrMax;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->lambda->data[0], &incx_t,
              &TrialState->lambdaStopTest->data[0], &incy_t);
      }
      if (WorkingSet->nVar >= 1) {
        n_t = (ptrdiff_t)WorkingSet->nVar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->xstarsqp->data[0], &incx_t,
              &TrialState->xstar->data[0], &incy_t);
      }
      idxStartIneq = r->size[0];
      r->size[0] = TrialState->grad->size[0];
      emxEnsureCapacity_real_T(r, idxStartIneq);
      r1 = r->data;
      iH0 = TrialState->grad->size[0];
      for (idxStartIneq = 0; idxStartIneq < iH0; idxStartIneq++) {
        r1[idxStartIneq] = TrialState->grad->data[idxStartIneq];
      }
      b_qpoptions = *qpoptions;
      b_driver(Hessian, r, TrialState, memspace, WorkingSet, QRManager,
               CholManager, QPObjective, &b_qpoptions,
               qpoptions->MaxIterations);
      idxStartIneq = (idx_col / 2) << 1;
      idxEndIneq = idxStartIneq - 2;
      for (iH0 = 0; iH0 <= idxEndIneq; iH0 += 2) {
        r3 = _mm_loadu_pd(&TrialState->socDirection->data[iH0]);
        r4 = _mm_loadu_pd(&TrialState->xstar->data[iH0]);
        _mm_storeu_pd(&TrialState->socDirection->data[iH0], _mm_sub_pd(r4, r3));
        _mm_storeu_pd(&TrialState->xstar->data[iH0], r3);
      }
      for (iH0 = idxStartIneq; iH0 < idx_col; iH0++) {
        lenSOC = TrialState->socDirection->data[iH0];
        oldDirIdx = lenSOC;
        lenSOC = TrialState->xstar->data[iH0] - lenSOC;
        TrialState->socDirection->data[iH0] = lenSOC;
        TrialState->xstar->data[iH0] = oldDirIdx;
      }
      if (idx_col < 1) {
        lenSOC = 0.0;
        oldDirIdx = 0.0;
      } else {
        n_t = (ptrdiff_t)idx_col;
        incx_t = (ptrdiff_t)1;
        lenSOC = dnrm2(&n_t, &TrialState->socDirection->data[0], &incx_t);
        n_t = (ptrdiff_t)idx_col;
        incx_t = (ptrdiff_t)1;
        oldDirIdx = dnrm2(&n_t, &TrialState->xstar->data[0], &incx_t);
      }
      stepSuccess = (lenSOC <= 2.0 * oldDirIdx);
      if (!stepSuccess) {
        if (mConstrMax >= 1) {
          n_t = (ptrdiff_t)mConstrMax;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dcopy(&n_t, &TrialState->lambdaStopTest->data[0], &incx_t,
                &TrialState->lambda->data[0], &incy_t);
        }
      } else {
        sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                     WorkingSet->sizes, WorkingSet->isActiveIdx,
                     WorkingSet->Wid, WorkingSet->Wlocalidx,
                     memspace->workspace_double);
      }
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState->state != -6)) {
        idxStartIneq = (nVar_tmp_tmp / 2) << 1;
        idxEndIneq = idxStartIneq - 2;
        for (iH0 = 0; iH0 <= idxEndIneq; iH0 += 2) {
          r3 = _mm_loadu_pd(&TrialState->xstar->data[iH0]);
          r4 = _mm_loadu_pd(&TrialState->socDirection->data[iH0]);
          _mm_storeu_pd(&TrialState->delta_x->data[iH0], _mm_add_pd(r3, r4));
        }
        for (iH0 = idxStartIneq; iH0 < nVar_tmp_tmp; iH0++) {
          TrialState->delta_x->data[iH0] = TrialState->xstar->data[iH0] +
                                           TrialState->socDirection->data[iH0];
        }
      }
      guard1 = true;
    } break;
    }
    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        mConstrMax = Hessian->size[0] - 1;
        oldDirIdx = 0.0;
        lenSOC = 1.0;
        for (iH0 = 0; iH0 <= mConstrMax; iH0++) {
          oldDirIdx = muDoubleScalarMax(
              oldDirIdx, muDoubleScalarAbs(TrialState->grad->data[iH0]));
          lenSOC = muDoubleScalarMax(
              lenSOC, muDoubleScalarAbs(TrialState->xstar->data[iH0]));
        }
        lenSOC = muDoubleScalarMax(2.2204460492503131E-16, oldDirIdx / lenSOC);
        for (idx_col = 0; idx_col <= mConstrMax; idx_col++) {
          iH0 = (mConstrMax + 1) * idx_col;
          for (k = 0; k < idx_col; k++) {
            Hessian_data[iH0 + k] = 0.0;
          }
          Hessian_data[idx_col + Hessian->size[0] * idx_col] = lenSOC;
          idxStartIneq = iH0 + idx_col;
          idxEndIneq = mConstrMax - idx_col;
          for (k = 0; k < idxEndIneq; k++) {
            Hessian_data[(idxStartIneq + k) + 1] = 0.0;
          }
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    idxStartIneq = WorkingSet->sizes[3];
    idxEndIneq = WorkingSet->sizes[4];
    idx_col = r->size[0];
    r->size[0] = TrialState->delta_x->size[0];
    emxEnsureCapacity_real_T(r, idx_col);
    r1 = r->data;
    iH0 = TrialState->delta_x->size[0];
    for (idx_col = 0; idx_col < iH0; idx_col++) {
      r1[idx_col] = TrialState->delta_x->data[idx_col];
    }
    emxInit_real_T(&r2, 1);
    idx_col = r2->size[0];
    r2->size[0] = TrialState->xstar->size[0];
    emxEnsureCapacity_real_T(r2, idx_col);
    Hessian_data = r2->data;
    iH0 = TrialState->xstar->size[0];
    for (idx_col = 0; idx_col < iH0; idx_col++) {
      Hessian_data[idx_col] = TrialState->xstar->data[idx_col];
    }
    if (lb->size[1] != 0) {
      for (iH0 = 0; iH0 < idxStartIneq; iH0++) {
        oldDirIdx = r1[WorkingSet->indexLB->data[iH0] - 1];
        lenSOC =
            (TrialState->xstarsqp->data[WorkingSet->indexLB->data[iH0] - 1] +
             oldDirIdx) -
            lb_data[WorkingSet->indexLB->data[iH0] - 1];
        if (lenSOC < 0.0) {
          r1[WorkingSet->indexLB->data[iH0] - 1] = oldDirIdx - lenSOC;
          Hessian_data[WorkingSet->indexLB->data[iH0] - 1] -= lenSOC;
        }
      }
    }
    if (ub->size[1] != 0) {
      for (iH0 = 0; iH0 < idxEndIneq; iH0++) {
        oldDirIdx = r1[WorkingSet->indexUB->data[iH0] - 1];
        lenSOC =
            (ub_data[WorkingSet->indexUB->data[iH0] - 1] -
             TrialState->xstarsqp->data[WorkingSet->indexUB->data[iH0] - 1]) -
            oldDirIdx;
        if (lenSOC < 0.0) {
          r1[WorkingSet->indexUB->data[iH0] - 1] = oldDirIdx + lenSOC;
          Hessian_data[WorkingSet->indexUB->data[iH0] - 1] += lenSOC;
        }
      }
    }
    idx_col = TrialState->delta_x->size[0];
    TrialState->delta_x->size[0] = r->size[0];
    emxEnsureCapacity_real_T(TrialState->delta_x, idx_col);
    iH0 = r->size[0];
    for (idx_col = 0; idx_col < iH0; idx_col++) {
      TrialState->delta_x->data[idx_col] = r1[idx_col];
    }
    idx_col = TrialState->xstar->size[0];
    TrialState->xstar->size[0] = r2->size[0];
    emxEnsureCapacity_real_T(TrialState->xstar, idx_col);
    iH0 = r2->size[0];
    for (idx_col = 0; idx_col < iH0; idx_col++) {
      TrialState->xstar->data[idx_col] = Hessian_data[idx_col];
    }
    emxFree_real_T(&r2);
  }
  emxFree_real_T(&r);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return stepSuccess;
}

/* End of code generation (step.c) */
