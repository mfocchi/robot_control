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
#include "optimize_cpp_mpc_internal_types.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "step.h"
#include "test_exit.h"
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
            j_struct_T *FiniteDifferences, f_struct_T *memspace,
            h_struct_T *WorkingSet, c_struct_T *QRManager,
            d_struct_T *CholManager, e_struct_T *QPObjective)
{
  static const char_T qpoptions_SolverName[7] = {'f', 'm', 'i', 'n',
                                                 'c', 'o', 'n'};
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  b_struct_T Flags;
  k_struct_T b_expl_temp;
  k_struct_T expl_temp;
  const real_T *lb_data;
  const real_T *ub_data;
  real_T phi_alpha;
  int32_T b_i;
  int32_T i;
  int32_T idx;
  int32_T mConstr;
  int32_T mFixed;
  int32_T mLB;
  int32_T mUB;
  int32_T nVar;
  int32_T nVar_tmp_tmp;
  int32_T qpoptions_MaxIterations;
  ub_data = ub->data;
  lb_data = lb->data;
  nVar_tmp_tmp = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr =
      (WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4];
  nVar = (WorkingSet->sizes[3] + WorkingSet->sizes[4]) +
         (WorkingSet->sizes[0] << 1);
  qpoptions_MaxIterations = 10 * muIntScalarMax_sint32(WorkingSet->nVar, nVar);
  TrialState->steplength = 1.0;
  Flags.gradOK = test_exit(MeritFunction, WorkingSet, TrialState, lb, ub,
                           &Flags.fevalOK, &Flags.done, &Flags.stepAccepted,
                           &Flags.failedLineSearch, &Flags.stepType);
  TrialState->sqpFval_old = TrialState->sqpFval;
  nVar = TrialState->xstarsqp->size[1];
  if (TrialState->xstarsqp->size[1] >= 1) {
    n_t = (ptrdiff_t)TrialState->xstarsqp->size[1];
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    dcopy(&n_t, &TrialState->xstarsqp->data[0], &incx_t,
          &TrialState->xstarsqp_old->data[0], &incy_t);
  }
  for (i = 0; i < nVar; i++) {
    TrialState->grad_old->data[i] = TrialState->grad->data[i];
  }
  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }
  while (!Flags.done) {
    __m128d r;
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      boolean_T hasLB;
      boolean_T hasUB;
      if (Flags.stepType != 3) {
        hasLB = (lb->size[1] != 0);
        hasUB = (ub->size[1] != 0);
        if (hasLB) {
          for (idx = 0; idx < mLB; idx++) {
            WorkingSet->lb->data[WorkingSet->indexLB->data[idx] - 1] =
                -lb_data[WorkingSet->indexLB->data[idx] - 1] +
                TrialState->xstarsqp->data[WorkingSet->indexLB->data[idx] - 1];
          }
        }
        if (hasUB) {
          for (idx = 0; idx < mUB; idx++) {
            WorkingSet->ub->data[WorkingSet->indexUB->data[idx] - 1] =
                ub_data[WorkingSet->indexUB->data[idx] - 1] -
                TrialState->xstarsqp->data[WorkingSet->indexUB->data[idx] - 1];
          }
        }
        if (hasLB && hasUB) {
          for (idx = 0; idx < mFixed; idx++) {
            phi_alpha = ub_data[WorkingSet->indexFixed->data[idx] - 1] -
                        TrialState->xstarsqp
                            ->data[WorkingSet->indexFixed->data[idx] - 1];
            WorkingSet->ub->data[WorkingSet->indexFixed->data[idx] - 1] =
                phi_alpha;
            WorkingSet->bwset->data[idx] = phi_alpha;
          }
        }
        if (WorkingSet->nActiveConstr > mFixed) {
          b_i = mFixed + 1;
          nVar = muIntScalarMax_sint32(b_i, 1);
          b_i = WorkingSet->nActiveConstr;
          for (idx = nVar; idx <= b_i; idx++) {
            switch (WorkingSet->Wid->data[idx - 1]) {
            case 4:
              WorkingSet->bwset->data[idx - 1] =
                  WorkingSet->lb
                      ->data[WorkingSet->indexLB->data
                                 [WorkingSet->Wlocalidx->data[idx - 1] - 1] -
                             1];
              break;
            case 5:
              WorkingSet->bwset->data[idx - 1] =
                  WorkingSet->ub
                      ->data[WorkingSet->indexUB->data
                                 [WorkingSet->Wlocalidx->data[idx - 1] - 1] -
                             1];
              break;
            default:
              /* A check that is always false is detected at compile-time.
               * Eliminating code that follows. */
              break;
            }
          }
        }
      }
      expl_temp.ObjectiveLimit = rtMinusInf;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (b_i = 0; b_i < 7; b_i++) {
        expl_temp.SolverName[b_i] = qpoptions_SolverName[b_i];
      }
      b_expl_temp = expl_temp;
      Flags.stepAccepted = step(&Flags.stepType, Hessian, lb, ub, TrialState,
                                MeritFunction, memspace, WorkingSet, QRManager,
                                CholManager, QPObjective, &b_expl_temp);
      if (Flags.stepAccepted) {
        for (i = 0; i < nVar_tmp_tmp; i++) {
          TrialState->xstarsqp->data[i] += TrialState->delta_x->data[i];
        }
        TrialState->sqpFval =
            evalObjAndConstr(&FcnEvaluator->next.next.next.next.next.next.next
                                  .next.value.workspace,
                             TrialState->xstarsqp, &nVar);
        Flags.fevalOK = (nVar == 1);
        TrialState->FunctionEvaluations++;
        if (Flags.fevalOK) {
          MeritFunction->phiFullStep =
              TrialState->sqpFval + MeritFunction->penaltyParam * 0.0;
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
        int32_T exitflagLnSrch;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          hasLB = true;
        } else {
          hasLB = false;
        }
        hasUB = Flags.fevalOK;
        b_i = WorkingSet->nVar;
        alpha = 1.0;
        exitflagLnSrch = 1;
        phi_alpha = MeritFunction->phiFullStep;
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
          if (TrialState->FunctionEvaluations < 5000) {
            if (hasUB && (phi_alpha <=
                          MeritFunction->phi +
                              alpha * 0.0001 * MeritFunction->phiPrimePlus)) {
              exitg1 = 1;
            } else {
              boolean_T exitg2;
              boolean_T tooSmallX;
              alpha *= 0.7;
              nVar = (b_i / 2) << 1;
              i = nVar - 2;
              for (idx = 0; idx <= i; idx += 2) {
                r = _mm_loadu_pd(&TrialState->xstar->data[idx]);
                _mm_storeu_pd(&TrialState->delta_x->data[idx],
                              _mm_mul_pd(_mm_set1_pd(alpha), r));
              }
              for (idx = nVar; idx < b_i; idx++) {
                TrialState->delta_x->data[idx] =
                    alpha * TrialState->xstar->data[idx];
              }
              if (hasLB) {
                phi_alpha = alpha * alpha;
                if (b_i >= 1) {
                  n_t = (ptrdiff_t)b_i;
                  incx_t = (ptrdiff_t)1;
                  incy_t = (ptrdiff_t)1;
                  daxpy(&n_t, &phi_alpha, &TrialState->socDirection->data[0],
                        &incx_t, &TrialState->delta_x->data[0], &incy_t);
                }
              }
              tooSmallX = true;
              idx = 0;
              exitg2 = false;
              while ((!exitg2) && (idx <= b_i - 1)) {
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
                for (idx = 0; idx < b_i; idx++) {
                  TrialState->xstarsqp->data[idx] =
                      TrialState->xstarsqp_old->data[idx] +
                      TrialState->delta_x->data[idx];
                }
                TrialState->sqpFval =
                    evalObjAndConstr(&FcnEvaluator->next.next.next.next.next
                                          .next.next.next.value.workspace,
                                     TrialState->xstarsqp, &nVar);
                TrialState->FunctionEvaluations++;
                hasUB = (nVar == 1);
                if (hasUB) {
                  phi_alpha =
                      TrialState->sqpFval + MeritFunction->penaltyParam * 0.0;
                } else {
                  phi_alpha = rtInf;
                }
              }
            }
          } else {
            exitflagLnSrch = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        Flags.fevalOK = hasUB;
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
      nVar = (mConstr / 2) << 1;
      i = nVar - 2;
      for (idx = 0; idx <= i; idx += 2) {
        __m128d r1;
        r = _mm_loadu_pd(&TrialState->lambda->data[idx]);
        r1 = _mm_loadu_pd(&TrialState->lambdasqp->data[idx]);
        _mm_storeu_pd(
            &TrialState->lambdasqp->data[idx],
            _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(TrialState->steplength),
                                      _mm_sub_pd(r, r1))));
      }
      for (idx = nVar; idx < mConstr; idx++) {
        phi_alpha = TrialState->lambdasqp->data[idx];
        phi_alpha += TrialState->steplength *
                     (TrialState->lambda->data[idx] - phi_alpha);
        TrialState->lambdasqp->data[idx] = phi_alpha;
      }
      TrialState->sqpFval_old = TrialState->sqpFval;
      nVar = TrialState->xstarsqp->size[1];
      if (TrialState->xstarsqp->size[1] >= 1) {
        n_t = (ptrdiff_t)TrialState->xstarsqp->size[1];
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->xstarsqp->data[0], &incx_t,
              &TrialState->xstarsqp_old->data[0], &incy_t);
      }
      for (i = 0; i < nVar; i++) {
        TrialState->grad_old->data[i] = TrialState->grad->data[i];
      }
      Flags.gradOK = computeFiniteDifferences(
          FiniteDifferences, TrialState->sqpFval, TrialState->xstarsqp,
          TrialState->grad, lb, ub);
      TrialState->FunctionEvaluations += FiniteDifferences->numEvals;
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      if (TrialState->xstarsqp->size[1] >= 1) {
        n_t = (ptrdiff_t)TrialState->xstarsqp->size[1];
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &TrialState->xstarsqp_old->data[0], &incx_t,
              &TrialState->xstarsqp->data[0], &incy_t);
      }
    }
    b_test_exit(&Flags, memspace, MeritFunction, WorkingSet, TrialState,
                QRManager, lb, ub);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      for (i = 0; i < nVar_tmp_tmp; i++) {
        TrialState->delta_gradLag->data[i] = TrialState->grad->data[i];
      }
      if (nVar_tmp_tmp >= 1) {
        phi_alpha = -1.0;
        n_t = (ptrdiff_t)nVar_tmp_tmp;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        daxpy(&n_t, &phi_alpha, &TrialState->grad_old->data[0], &incx_t,
              &TrialState->delta_gradLag->data[0], &incy_t);
      }
      BFGSUpdate(nVar_tmp_tmp, Hessian, TrialState->delta_x,
                 TrialState->delta_gradLag, memspace->workspace_double);
      TrialState->sqpIterations++;
    }
  }
}

/* End of code generation (driver.c) */
