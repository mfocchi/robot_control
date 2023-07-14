/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * test_exit.c
 *
 * Code generation for function 'test_exit'
 *
 */

/* Include files */
#include "test_exit.h"
#include "computeComplError.h"
#include "computeDualFeasError.h"
#include "computeGradLag.h"
#include "computeQ_.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "updateWorkingSetForNewQP.h"
#include "xgeqp3.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void b_test_exit(b_struct_T *Flags, f_struct_T *memspace,
                 struct_T *MeritFunction,
                 const emxArray_real_T *fscales_cineq_constraint,
                 h_struct_T *WorkingSet, g_struct_T *TrialState,
                 c_struct_T *QRManager, const emxArray_real_T *lb,
                 const emxArray_real_T *ub)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emxArray_real_T *r;
  const real_T *lb_data;
  const real_T *ub_data;
  real_T nlpComplErrorLSQ;
  real_T optimRelativeFactor;
  real_T tol;
  real_T *r1;
  int32_T idx;
  int32_T idx_max;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mLambda;
  int32_T mUB;
  int32_T nVar;
  char_T TRANSA;
  char_T TRANSA1;
  char_T UPLO1;
  boolean_T isFeasible;
  ub_data = ub->data;
  lb_data = lb->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mIneq = WorkingSet->sizes[2];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mLambda =
      ((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes[3]) +
      WorkingSet->sizes[4];
  if (mLambda >= 1) {
    n_t = (ptrdiff_t)mLambda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &TrialState->lambdasqp->data[0], &incx_t,
          &TrialState->lambdaStopTest->data[0], &incy_t);
  }
  computeGradLag(TrialState->gradLag, WorkingSet->ldA, WorkingSet->nVar,
                 TrialState->grad, WorkingSet->sizes[2], WorkingSet->Aineq,
                 WorkingSet->indexFixed, WorkingSet->sizes[0],
                 WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
                 WorkingSet->sizes[4], TrialState->lambdaStopTest);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    n_t = (ptrdiff_t)WorkingSet->nVar;
    incx_t = (ptrdiff_t)1;
    n_t = idamax(&n_t, &TrialState->grad->data[0], &incx_t);
    idx_max = (int32_T)n_t;
  }
  optimRelativeFactor = muDoubleScalarMax(
      1.0, muDoubleScalarAbs(TrialState->grad->data[idx_max - 1]));
  if (muDoubleScalarIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  tol = 0.0;
  idx_max = WorkingSet->sizes[2];
  for (idx = 0; idx < idx_max; idx++) {
    tol = muDoubleScalarMax(tol, TrialState->cIneq->data[idx]);
  }
  for (idx = 0; idx < mLB; idx++) {
    idx_max = WorkingSet->indexLB->data[idx] - 1;
    tol = muDoubleScalarMax(tol, lb_data[idx_max] -
                                     TrialState->xstarsqp->data[idx_max]);
  }
  for (idx = 0; idx < mUB; idx++) {
    idx_max = WorkingSet->indexUB->data[idx] - 1;
    tol = muDoubleScalarMax(tol, TrialState->xstarsqp->data[idx_max] -
                                     ub_data[idx_max]);
  }
  MeritFunction->nlpPrimalFeasError = tol;
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = muDoubleScalarMax(1.0, tol);
  }
  isFeasible = (tol <= 0.001 * MeritFunction->feasRelativeFactor);
  Flags->gradOK = computeDualFeasError(WorkingSet->nVar, TrialState->gradLag,
                                       &MeritFunction->nlpDualFeasError);
  emxInit_real_T(&r, 1);
  if (!Flags->gradOK) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = computeComplError(
        fscales_cineq_constraint, TrialState->xstarsqp, WorkingSet->sizes[2],
        TrialState->cIneq, WorkingSet->indexLB, WorkingSet->sizes[3], lb,
        WorkingSet->indexUB, WorkingSet->sizes[4], ub,
        TrialState->lambdaStopTest, WorkingSet->sizes[0] + 1);
    tol = muDoubleScalarMax(MeritFunction->nlpDualFeasError,
                            MeritFunction->nlpComplError);
    MeritFunction->firstOrderOpt = tol;
    if (TrialState->sqpIterations > 1) {
      real_T d;
      real_T nlpComplErrorTmp;
      computeGradLag(memspace->workspace_double, WorkingSet->ldA,
                     WorkingSet->nVar, TrialState->grad, WorkingSet->sizes[2],
                     WorkingSet->Aineq, WorkingSet->indexFixed,
                     WorkingSet->sizes[0], WorkingSet->indexLB,
                     WorkingSet->sizes[3], WorkingSet->indexUB,
                     WorkingSet->sizes[4], TrialState->lambdaStopTestPrev);
      computeDualFeasError(WorkingSet->nVar, memspace->workspace_double,
                           &nlpComplErrorLSQ);
      nlpComplErrorTmp = computeComplError(
          fscales_cineq_constraint, TrialState->xstarsqp, WorkingSet->sizes[2],
          TrialState->cIneq, WorkingSet->indexLB, WorkingSet->sizes[3], lb,
          WorkingSet->indexUB, WorkingSet->sizes[4], ub,
          TrialState->lambdaStopTestPrev, WorkingSet->sizes[0] + 1);
      d = muDoubleScalarMax(nlpComplErrorLSQ, nlpComplErrorTmp);
      if (d < tol) {
        MeritFunction->nlpDualFeasError = nlpComplErrorLSQ;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = d;
        if (mLambda >= 1) {
          n_t = (ptrdiff_t)mLambda;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dcopy(&n_t, &TrialState->lambdaStopTestPrev->data[0], &incx_t,
                &TrialState->lambdaStopTest->data[0], &incy_t);
        }
      } else if (mLambda >= 1) {
        n_t = (ptrdiff_t)mLambda;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->lambdaStopTest->data[0], &incx_t,
              &TrialState->lambdaStopTestPrev->data[0], &incy_t);
      }
    } else if (mLambda >= 1) {
      n_t = (ptrdiff_t)mLambda;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &TrialState->lambdaStopTest->data[0], &incx_t,
            &TrialState->lambdaStopTestPrev->data[0], &incy_t);
    }
    if (isFeasible &&
        (MeritFunction->nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (MeritFunction->nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        boolean_T guard1;
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          boolean_T dxTooSmall;
          boolean_T exitg1;
          dxTooSmall = true;
          idx = 0;
          exitg1 = false;
          while ((!exitg1) && (idx <= nVar - 1)) {
            if (1.0E-6 * muDoubleScalarMax(
                             1.0, muDoubleScalarAbs(
                                      TrialState->xstarsqp->data[idx])) <=
                muDoubleScalarAbs(TrialState->delta_x->data[idx])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              idx++;
            }
          }
          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType != 2) {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              }
            } else {
              int32_T nActiveConstr;
              nActiveConstr = WorkingSet->nActiveConstr;
              if (WorkingSet->nActiveConstr > 0) {
                int32_T iQR_diag;
                updateWorkingSetForNewQP(
                    TrialState->xstarsqp, WorkingSet, WorkingSet->sizes[2],
                    TrialState->mNonlinIneq, TrialState->cIneq,
                    WorkingSet->sizes[3], lb, WorkingSet->sizes[4], ub,
                    WorkingSet->sizes[0]);
                idx = r->size[0];
                r->size[0] = TrialState->lambda->size[0];
                emxEnsureCapacity_real_T(r, idx);
                r1 = r->data;
                idx_max = TrialState->lambda->size[0];
                for (idx = 0; idx < idx_max; idx++) {
                  r1[idx] = TrialState->lambda->data[idx];
                }
                for (idx_max = 0; idx_max < nActiveConstr; idx_max++) {
                  r1[idx_max] = 0.0;
                }
                n_t = (ptrdiff_t)nVar;
                incx_t = (ptrdiff_t)1;
                incy_t = (ptrdiff_t)1;
                for (idx = 0; idx < nActiveConstr; idx++) {
                  dcopy(&n_t, &WorkingSet->ATwset->data[WorkingSet->ldA * idx],
                        &incx_t, &QRManager->QR->data[QRManager->ldq * idx],
                        &incy_t);
                }
                QRManager->usedPivoting = true;
                QRManager->mrows = nVar;
                QRManager->ncols = nActiveConstr;
                idx = muIntScalarMin_sint32(nVar, nActiveConstr);
                QRManager->minRowCol = idx;
                xgeqp3(QRManager->QR, nVar, nActiveConstr, QRManager->jpvt,
                       QRManager->tau);
                computeQ_(QRManager, nVar);
                if (nVar >= 1) {
                  tol = 1.0;
                  nlpComplErrorLSQ = 0.0;
                  TRANSA = 'T';
                  m_t = (ptrdiff_t)nVar;
                  n_t = (ptrdiff_t)nVar;
                  lda_t = (ptrdiff_t)QRManager->ldq;
                  incx_t = (ptrdiff_t)1;
                  incy_t = (ptrdiff_t)1;
                  dgemv(&TRANSA, &m_t, &n_t, &tol, &QRManager->Q->data[0],
                        &lda_t, &TrialState->grad->data[0], &incx_t,
                        &nlpComplErrorLSQ, &memspace->workspace_double->data[0],
                        &incy_t);
                }
                tol = muDoubleScalarAbs(QRManager->QR->data[0]) *
                      muDoubleScalarMin(
                          1.4901161193847656E-8,
                          (real_T)muIntScalarMax_sint32(nVar, nActiveConstr) *
                              2.2204460492503131E-16);
                idx_max = 0;
                iQR_diag = 0;
                while (
                    (idx_max < idx) &&
                    (muDoubleScalarAbs(QRManager->QR->data[iQR_diag]) > tol)) {
                  idx_max++;
                  iQR_diag = (iQR_diag + QRManager->ldq) + 1;
                }
                if (idx_max >= 1) {
                  TRANSA = 'N';
                  TRANSA1 = 'N';
                  UPLO1 = 'U';
                  n_t = (ptrdiff_t)idx_max;
                  lda_t = (ptrdiff_t)QRManager->ldq;
                  incx_t = (ptrdiff_t)1;
                  dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t,
                        &QRManager->QR->data[0], &lda_t,
                        &memspace->workspace_double->data[0], &incx_t);
                }
                idx_max = muIntScalarMin_sint32(nActiveConstr, idx);
                for (idx = 0; idx < idx_max; idx++) {
                  r1[QRManager->jpvt->data[idx] - 1] =
                      memspace->workspace_double->data[idx];
                }
                idx = TrialState->lambda->size[0];
                TrialState->lambda->size[0] = r->size[0];
                emxEnsureCapacity_real_T(TrialState->lambda, idx);
                idx_max = r->size[0];
                for (idx = 0; idx < idx_max; idx++) {
                  TrialState->lambda->data[idx] = r1[idx];
                }
                idx_max = mFixed + 1;
                iQR_diag = mFixed - 1;
                for (idx = idx_max; idx <= iQR_diag; idx += 2) {
                  __m128d r2;
                  r2 = _mm_loadu_pd(&TrialState->lambda->data[idx - 1]);
                  _mm_storeu_pd(&TrialState->lambda->data[idx - 1],
                                _mm_mul_pd(r2, _mm_set1_pd(-1.0)));
                }
                for (idx = idx_max; idx <= mFixed; idx++) {
                  TrialState->lambda->data[idx - 1] =
                      -TrialState->lambda->data[idx - 1];
                }
                sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                             WorkingSet->sizes, WorkingSet->isActiveIdx,
                             WorkingSet->Wid, WorkingSet->Wlocalidx,
                             memspace->workspace_double);
                computeGradLag(memspace->workspace_double, WorkingSet->ldA,
                               nVar, TrialState->grad, mIneq, WorkingSet->Aineq,
                               WorkingSet->indexFixed, mFixed,
                               WorkingSet->indexLB, mLB, WorkingSet->indexUB,
                               mUB, TrialState->lambda);
                computeDualFeasError(nVar, memspace->workspace_double, &tol);
                nlpComplErrorLSQ = computeComplError(
                    fscales_cineq_constraint, TrialState->xstarsqp, mIneq,
                    TrialState->cIneq, WorkingSet->indexLB, mLB, lb,
                    WorkingSet->indexUB, mUB, ub, TrialState->lambda,
                    mFixed + 1);
                if ((tol <= 1.0E-6 * optimRelativeFactor) &&
                    (nlpComplErrorLSQ <= 1.0E-6 * optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = tol;
                  MeritFunction->nlpComplError = nlpComplErrorLSQ;
                  MeritFunction->firstOrderOpt =
                      muDoubleScalarMax(tol, nlpComplErrorLSQ);
                  if (mLambda >= 1) {
                    n_t = (ptrdiff_t)mLambda;
                    incx_t = (ptrdiff_t)1;
                    incy_t = (ptrdiff_t)1;
                    dcopy(&n_t, &TrialState->lambda->data[0], &incx_t,
                          &TrialState->lambdaStopTest->data[0], &incy_t);
                  }
                  Flags->done = true;
                  TrialState->sqpExitFlag = 1;
                } else {
                  Flags->done = true;
                  TrialState->sqpExitFlag = 2;
                }
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          if (TrialState->sqpIterations >= 400) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 10000) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
  emxFree_real_T(&r);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

boolean_T test_exit(struct_T *MeritFunction,
                    const emxArray_real_T *fscales_cineq_constraint,
                    const h_struct_T *WorkingSet, g_struct_T *TrialState,
                    const emxArray_real_T *lb, const emxArray_real_T *ub,
                    boolean_T *Flags_fevalOK, boolean_T *Flags_done,
                    boolean_T *Flags_stepAccepted,
                    boolean_T *Flags_failedLineSearch, int32_T *Flags_stepType)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emxArray_real_T *gradLag;
  const real_T *lb_data;
  const real_T *ub_data;
  real_T feasError;
  real_T optimRelativeFactor;
  int32_T idx;
  int32_T idx_max;
  int32_T mLB;
  int32_T mLambda;
  int32_T mUB;
  boolean_T Flags_gradOK;
  boolean_T exitg1;
  boolean_T isFeasible;
  ub_data = ub->data;
  lb_data = lb->data;
  *Flags_fevalOK = true;
  *Flags_done = false;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mLambda =
      ((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes[3]) +
      WorkingSet->sizes[4];
  if (mLambda >= 1) {
    n_t = (ptrdiff_t)mLambda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &TrialState->lambdasqp->data[0], &incx_t,
          &TrialState->lambdaStopTest->data[0], &incy_t);
  }
  computeGradLag(TrialState->gradLag, WorkingSet->ldA, WorkingSet->nVar,
                 TrialState->grad, WorkingSet->sizes[2], WorkingSet->Aineq,
                 WorkingSet->indexFixed, WorkingSet->sizes[0],
                 WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
                 WorkingSet->sizes[4], TrialState->lambdaStopTest);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    n_t = (ptrdiff_t)WorkingSet->nVar;
    incx_t = (ptrdiff_t)1;
    n_t = idamax(&n_t, &TrialState->grad->data[0], &incx_t);
    idx_max = (int32_T)n_t;
  }
  optimRelativeFactor = muDoubleScalarMax(
      1.0, muDoubleScalarAbs(TrialState->grad->data[idx_max - 1]));
  if (muDoubleScalarIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  feasError = 0.0;
  idx_max = WorkingSet->sizes[2];
  for (idx = 0; idx < idx_max; idx++) {
    feasError = muDoubleScalarMax(feasError, TrialState->cIneq->data[idx]);
  }
  for (idx = 0; idx < mLB; idx++) {
    idx_max = WorkingSet->indexLB->data[idx] - 1;
    feasError = muDoubleScalarMax(
        feasError, lb_data[idx_max] - TrialState->xstarsqp->data[idx_max]);
  }
  for (idx = 0; idx < mUB; idx++) {
    idx_max = WorkingSet->indexUB->data[idx] - 1;
    feasError = muDoubleScalarMax(
        feasError, TrialState->xstarsqp->data[idx_max] - ub_data[idx_max]);
  }
  MeritFunction->nlpPrimalFeasError = feasError;
  MeritFunction->feasRelativeFactor = muDoubleScalarMax(1.0, feasError);
  isFeasible = (feasError <= 0.001 * MeritFunction->feasRelativeFactor);
  idx_max = WorkingSet->nVar;
  gradLag = TrialState->gradLag;
  ub_data = gradLag->data;
  Flags_gradOK = true;
  feasError = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= idx_max - 1)) {
    Flags_gradOK = ((!muDoubleScalarIsInf(ub_data[idx])) &&
                    (!muDoubleScalarIsNaN(ub_data[idx])));
    if (!Flags_gradOK) {
      exitg1 = true;
    } else {
      feasError = muDoubleScalarMax(feasError, muDoubleScalarAbs(ub_data[idx]));
      idx++;
    }
  }
  MeritFunction->nlpDualFeasError = feasError;
  if (!Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = computeComplError(
        fscales_cineq_constraint, TrialState->xstarsqp, WorkingSet->sizes[2],
        TrialState->cIneq, WorkingSet->indexLB, WorkingSet->sizes[3], lb,
        WorkingSet->indexUB, WorkingSet->sizes[4], ub,
        TrialState->lambdaStopTest, WorkingSet->sizes[0] + 1);
    MeritFunction->firstOrderOpt = muDoubleScalarMax(
        MeritFunction->nlpDualFeasError, MeritFunction->nlpComplError);
    if (mLambda >= 1) {
      n_t = (ptrdiff_t)mLambda;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &TrialState->lambdaStopTest->data[0], &incx_t,
            &TrialState->lambdaStopTestPrev->data[0], &incy_t);
    }
    if (isFeasible &&
        (MeritFunction->nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (MeritFunction->nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    } else if (TrialState->FunctionEvaluations >= 10000) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 0;
    }
  }
  return Flags_gradOK;
}

/* End of code generation (test_exit.c) */
