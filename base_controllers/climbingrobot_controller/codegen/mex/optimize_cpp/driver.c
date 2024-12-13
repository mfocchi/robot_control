/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * driver.c
 *
 * Code generation for function 'driver'
 *
 */

/* Include files */
#include "driver.h"
#include "BFGSUpdate.h"
#include "computeFiniteDifferences.h"
#include "evalObjAndConstr.h"
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "saveState.h"
#include "step.h"
#include "test_exit.h"
#include "updateWorkingSetForNewQP.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void driver(emxArray_real_T *Hessian, const emxArray_real_T *lb,
            const emxArray_real_T *ub, g_struct_T *TrialState,
            struct_T *MeritFunction,
            const i_coder_internal_stickyStruct *FcnEvaluator,
            k_struct_T *FiniteDifferences, f_struct_T *memspace,
            h_struct_T *WorkingSet, c_struct_T *QRManager,
            d_struct_T *CholManager, e_struct_T *QPObjective,
            const emxArray_real_T *fscales_cineq_constraint)
{
  static const char_T qpoptions_SolverName[7] = {'f', 'm', 'i', 'n',
                                                 'c', 'o', 'n'};
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  b_struct_T Flags;
  l_struct_T b_expl_temp;
  l_struct_T expl_temp;
  real_T beta1;
  real_T constrViolationIneq;
  int32_T b_i;
  int32_T b_mIneq;
  int32_T i;
  int32_T idx;
  int32_T ldJ_tmp;
  int32_T mConstr;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mLinIneq;
  int32_T mUB;
  int32_T nVar_tmp_tmp;
  int32_T qpoptions_MaxIterations;
  int32_T y;
  char_T TRANSA;
  nVar_tmp_tmp = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mIneq = WorkingSet->sizes[2];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr =
      ((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes[3]) +
      WorkingSet->sizes[4];
  mLinIneq = WorkingSet->sizes[2] - TrialState->mNonlinIneq;
  y = ((WorkingSet->sizes[2] + WorkingSet->sizes[3]) + WorkingSet->sizes[4]) +
      (WorkingSet->sizes[0] << 1);
  qpoptions_MaxIterations = 10 * muIntScalarMax_sint32(WorkingSet->nVar, y);
  TrialState->steplength = 1.0;
  Flags.gradOK =
      test_exit(MeritFunction, fscales_cineq_constraint, WorkingSet, TrialState,
                lb, ub, &Flags.fevalOK, &Flags.done, &Flags.stepAccepted,
                &Flags.failedLineSearch, &Flags.stepType);
  ldJ_tmp = WorkingSet->ldA;
  y = WorkingSet->ldA * (TrialState->iNonIneq0 - 1);
  i = mIneq - TrialState->iNonIneq0;
  for (b_mIneq = 0; b_mIneq <= i; b_mIneq++) {
    n_t = (ptrdiff_t)nVar_tmp_tmp;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    b_i = b_mIneq * ldJ_tmp;
    dcopy(&n_t, &WorkingSet->Aineq->data[y + b_i], &incx_t,
          &TrialState->JacCineqTrans_old->data[b_i], &incy_t);
  }
  saveState(TrialState);
  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }
  while (!Flags.done) {
    __m128d r;
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        updateWorkingSetForNewQP(TrialState->xstarsqp, WorkingSet, mIneq,
                                 TrialState->mNonlinIneq, TrialState->cIneq,
                                 mLB, lb, mUB, ub, mFixed);
      }
      expl_temp.ObjectiveLimit = rtMinusInf;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (i = 0; i < 7; i++) {
        expl_temp.SolverName[i] = qpoptions_SolverName[i];
      }
      b_expl_temp = expl_temp;
      Flags.stepAccepted = step(&Flags.stepType, Hessian, lb, ub, TrialState,
                                MeritFunction, memspace, WorkingSet, QRManager,
                                CholManager, QPObjective, &b_expl_temp);
      if (Flags.stepAccepted) {
        for (b_i = 0; b_i < nVar_tmp_tmp; b_i++) {
          TrialState->xstarsqp->data[b_i] += TrialState->delta_x->data[b_i];
        }
        TrialState->sqpFval = evalObjAndConstr(
            FcnEvaluator->next.next.next.next.next.value,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace.p0,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace.pf,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                .Fleg_max,
            FcnEvaluator->next.next.next.next.next.next.next.value.workspace.mu,
            &FcnEvaluator->next.next.next.next.next.next.next.value.workspace
                 .params,
            &FcnEvaluator->next.next.next.next.next.next.next.next.value
                 .workspace,
            TrialState->xstarsqp, TrialState->cIneq, TrialState->iNonIneq0, &y);
        Flags.fevalOK = (y == 1);
        TrialState->FunctionEvaluations++;
        if (mLinIneq > 0) {
          constrViolationIneq = 1.0;
          beta1 = -1.0;
          TRANSA = 'T';
          m_t = (ptrdiff_t)nVar_tmp_tmp;
          n_t = (ptrdiff_t)mLinIneq;
          lda_t = (ptrdiff_t)WorkingSet->ldA;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dgemv(&TRANSA, &m_t, &n_t, &constrViolationIneq,
                &WorkingSet->Aineq->data[0], &lda_t,
                &TrialState->xstarsqp->data[0], &incx_t, &beta1,
                &TrialState->cIneq->data[0], &incy_t);
        }
        if (Flags.fevalOK) {
          constrViolationIneq = 0.0;
          for (idx = 0; idx < mIneq; idx++) {
            beta1 = TrialState->cIneq->data[idx];
            if (beta1 > 0.0) {
              constrViolationIneq += beta1;
            }
          }
          MeritFunction->phiFullStep =
              TrialState->sqpFval +
              MeritFunction->penaltyParam * constrViolationIneq;
        } else {
          MeritFunction->phiFullStep = rtInf;
        }
      }
      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction->phi < MeritFunction->phiFullStep) &&
          (TrialState->sqpFval < TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        real_T alpha;
        int32_T b_mLinIneq;
        int32_T exitflagLnSrch;
        boolean_T evalWellDefined;
        boolean_T socTaken;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          socTaken = true;
        } else {
          socTaken = false;
        }
        evalWellDefined = Flags.fevalOK;
        i = WorkingSet->nVar;
        b_mIneq = TrialState->mIneq;
        b_mLinIneq = TrialState->mIneq - TrialState->mNonlinIneq;
        alpha = 1.0;
        exitflagLnSrch = 1;
        constrViolationIneq = MeritFunction->phiFullStep;
        if (WorkingSet->nVar >= 1) {
          n_t = (ptrdiff_t)WorkingSet->nVar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dcopy(&n_t, &TrialState->delta_x->data[0], &incx_t,
                &TrialState->searchDir->data[0], &incy_t);
        }
        int32_T exitg1;
        do {
          exitg1 = 0;
          if (TrialState->FunctionEvaluations < 10000) {
            if (evalWellDefined &&
                (constrViolationIneq <=
                 MeritFunction->phi +
                     alpha * 0.0001 * MeritFunction->phiPrimePlus)) {
              exitg1 = 1;
            } else {
              boolean_T exitg2;
              boolean_T tooSmallX;
              alpha *= 0.7;
              y = (i / 2) << 1;
              b_i = y - 2;
              for (idx = 0; idx <= b_i; idx += 2) {
                r = _mm_loadu_pd(&TrialState->xstar->data[idx]);
                _mm_storeu_pd(&TrialState->delta_x->data[idx],
                              _mm_mul_pd(_mm_set1_pd(alpha), r));
              }
              for (idx = y; idx < i; idx++) {
                TrialState->delta_x->data[idx] =
                    alpha * TrialState->xstar->data[idx];
              }
              if (socTaken) {
                constrViolationIneq = alpha * alpha;
                if (i >= 1) {
                  n_t = (ptrdiff_t)i;
                  incx_t = (ptrdiff_t)1;
                  incy_t = (ptrdiff_t)1;
                  daxpy(&n_t, &constrViolationIneq,
                        &TrialState->socDirection->data[0], &incx_t,
                        &TrialState->delta_x->data[0], &incy_t);
                }
              }
              tooSmallX = true;
              idx = 0;
              exitg2 = false;
              while ((!exitg2) && (idx <= i - 1)) {
                if (1.0E-6 * muDoubleScalarMax(
                                 1.0, muDoubleScalarAbs(
                                          TrialState->xstarsqp->data[idx])) <=
                    muDoubleScalarAbs(TrialState->delta_x->data[idx])) {
                  tooSmallX = false;
                  exitg2 = true;
                } else {
                  idx++;
                }
              }
              if (tooSmallX) {
                exitflagLnSrch = -2;
                exitg1 = 1;
              } else {
                for (idx = 0; idx < i; idx++) {
                  TrialState->xstarsqp->data[idx] =
                      TrialState->xstarsqp_old->data[idx] +
                      TrialState->delta_x->data[idx];
                }
                TrialState->sqpFval = evalObjAndConstr(
                    FcnEvaluator->next.next.next.next.next.value,
                    FcnEvaluator->next.next.next.next.next.next.next.value
                        .workspace.p0,
                    FcnEvaluator->next.next.next.next.next.next.next.value
                        .workspace.pf,
                    FcnEvaluator->next.next.next.next.next.next.next.value
                        .workspace.Fleg_max,
                    FcnEvaluator->next.next.next.next.next.next.next.value
                        .workspace.mu,
                    &FcnEvaluator->next.next.next.next.next.next.next.value
                         .workspace.params,
                    &FcnEvaluator->next.next.next.next.next.next.next.next.value
                         .workspace,
                    TrialState->xstarsqp, TrialState->cIneq,
                    TrialState->iNonIneq0, &y);
                if (b_mLinIneq > 0) {
                  constrViolationIneq = 1.0;
                  beta1 = -1.0;
                  TRANSA = 'T';
                  m_t = (ptrdiff_t)i;
                  n_t = (ptrdiff_t)b_mLinIneq;
                  lda_t = (ptrdiff_t)WorkingSet->ldA;
                  incx_t = (ptrdiff_t)1;
                  incy_t = (ptrdiff_t)1;
                  dgemv(&TRANSA, &m_t, &n_t, &constrViolationIneq,
                        &WorkingSet->Aineq->data[0], &lda_t,
                        &TrialState->xstarsqp->data[0], &incx_t, &beta1,
                        &TrialState->cIneq->data[0], &incy_t);
                }
                TrialState->FunctionEvaluations++;
                evalWellDefined = (y == 1);
                if (evalWellDefined) {
                  constrViolationIneq = 0.0;
                  for (idx = 0; idx < b_mIneq; idx++) {
                    beta1 = TrialState->cIneq->data[idx];
                    if (beta1 > 0.0) {
                      constrViolationIneq += beta1;
                    }
                  }
                  constrViolationIneq =
                      TrialState->sqpFval +
                      MeritFunction->penaltyParam * constrViolationIneq;
                } else {
                  constrViolationIneq = rtInf;
                }
              }
            }
          } else {
            exitflagLnSrch = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        Flags.fevalOK = evalWellDefined;
        TrialState->steplength = alpha;
        if (exitflagLnSrch > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }
    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      for (idx = 0; idx < nVar_tmp_tmp; idx++) {
        TrialState->xstarsqp->data[idx] = TrialState->xstarsqp_old->data[idx] +
                                          TrialState->delta_x->data[idx];
      }
      y = (mConstr / 2) << 1;
      b_i = y - 2;
      for (idx = 0; idx <= b_i; idx += 2) {
        __m128d r1;
        r = _mm_loadu_pd(&TrialState->lambda->data[idx]);
        r1 = _mm_loadu_pd(&TrialState->lambdasqp->data[idx]);
        _mm_storeu_pd(
            &TrialState->lambdasqp->data[idx],
            _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(TrialState->steplength),
                                      _mm_sub_pd(r, r1))));
      }
      for (idx = y; idx < mConstr; idx++) {
        beta1 = TrialState->lambdasqp->data[idx];
        beta1 +=
            TrialState->steplength * (TrialState->lambda->data[idx] - beta1);
        TrialState->lambdasqp->data[idx] = beta1;
      }
      saveState(TrialState);
      Flags.gradOK = computeFiniteDifferences(
          FiniteDifferences, TrialState->sqpFval, TrialState->cIneq,
          TrialState->iNonIneq0, TrialState->xstarsqp, TrialState->grad,
          WorkingSet->Aineq, TrialState->iNonIneq0, WorkingSet->ldA, lb, ub);
      TrialState->FunctionEvaluations += FiniteDifferences->numEvals;
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      n_t = (ptrdiff_t)TrialState->xstarsqp->size[1];
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &TrialState->xstarsqp_old->data[0], &incx_t,
            &TrialState->xstarsqp->data[0], &incy_t);
      if (TrialState->mIneq >= 1) {
        n_t = (ptrdiff_t)TrialState->mIneq;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->cIneq_old->data[0], &incx_t,
              &TrialState->cIneq->data[0], &incy_t);
      }
    }
    b_test_exit(&Flags, memspace, MeritFunction, fscales_cineq_constraint,
                WorkingSet, TrialState, QRManager, lb, ub);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      y = (mFixed + TrialState->iNonIneq0) - 1;
      for (b_i = 0; b_i < nVar_tmp_tmp; b_i++) {
        TrialState->delta_gradLag->data[b_i] = TrialState->grad->data[b_i];
      }
      if (nVar_tmp_tmp >= 1) {
        constrViolationIneq = -1.0;
        n_t = (ptrdiff_t)nVar_tmp_tmp;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        daxpy(&n_t, &constrViolationIneq, &TrialState->grad_old->data[0],
              &incx_t, &TrialState->delta_gradLag->data[0], &incy_t);
      }
      constrViolationIneq = 1.0;
      beta1 = 1.0;
      TRANSA = 'N';
      m_t = (ptrdiff_t)nVar_tmp_tmp;
      n_t = (ptrdiff_t)TrialState->mNonlinIneq;
      lda_t = (ptrdiff_t)WorkingSet->ldA;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dgemv(&TRANSA, &m_t, &n_t, &constrViolationIneq,
            &WorkingSet->Aineq
                 ->data[WorkingSet->ldA * (TrialState->iNonIneq0 - 1)],
            &lda_t, &TrialState->lambdasqp->data[y], &incx_t, &beta1,
            &TrialState->delta_gradLag->data[0], &incy_t);
      constrViolationIneq = -1.0;
      beta1 = 1.0;
      TRANSA = 'N';
      m_t = (ptrdiff_t)nVar_tmp_tmp;
      n_t = (ptrdiff_t)TrialState->mNonlinIneq;
      lda_t = (ptrdiff_t)WorkingSet->ldA;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dgemv(&TRANSA, &m_t, &n_t, &constrViolationIneq,
            &TrialState->JacCineqTrans_old->data[0], &lda_t,
            &TrialState->lambdasqp->data[y], &incx_t, &beta1,
            &TrialState->delta_gradLag->data[0], &incy_t);
      i = WorkingSet->ldA * (TrialState->iNonIneq0 - 1);
      b_i = mIneq - TrialState->iNonIneq0;
      for (b_mIneq = 0; b_mIneq <= b_i; b_mIneq++) {
        n_t = (ptrdiff_t)nVar_tmp_tmp;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        y = b_mIneq * ldJ_tmp;
        dcopy(&n_t, &WorkingSet->Aineq->data[i + y], &incx_t,
              &TrialState->JacCineqTrans_old->data[y], &incy_t);
      }
      BFGSUpdate(nVar_tmp_tmp, Hessian, TrialState->delta_x,
                 TrialState->delta_gradLag, memspace->workspace_double);
      TrialState->sqpIterations++;
    }
  }
}

/* End of code generation (driver.c) */
