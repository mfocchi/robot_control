/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
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
#include "computeDeltaLag.h"
#include "computeForwardDifferences.h"
#include "computeGradLag.h"
#include "evalObjAndConstr.h"
#include "ixamax.h"
#include "optimize_cpp_mpc_no_constraints_data.h"
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_internal_types.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "step.h"
#include "test_exit.h"
#include "xaxpy.h"
#include "xcopy.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void driver(emxArray_real_T *Hessian, const emxArray_real_T *lb, const
            emxArray_real_T *ub, d_struct_T *TrialState, k_struct_T
            *MeritFunction, const e_struct_T *FcnEvaluator, f_struct_T
            *FiniteDifferences, c_struct_T *memspace, j_struct_T *WorkingSet,
            g_struct_T *QRManager, h_struct_T *CholManager, i_struct_T
            *QPObjective)
{
  static const char_T qpoptions_SolverName[7] = { 'f', 'm', 'i', 'n', 'c', 'o',
    'n' };

  b_struct_T b_expl_temp;
  b_struct_T expl_temp;
  emxArray_real_T *b_TrialState;
  emxArray_real_T *gradLag;
  struct_T Flags;
  real_T feasError;
  real_T optimRelativeFactor;
  int32_T b_mLB;
  int32_T b_mUB;
  int32_T exitg2;
  int32_T i;
  int32_T idx;
  int32_T idxFiniteLB;
  int32_T mConstr;
  int32_T mFixed;
  int32_T mLB;
  int32_T mUB;
  int32_T nVar;
  int32_T qpoptions_MaxIterations;
  boolean_T exitg1;
  boolean_T hasUB;
  boolean_T isFeasible;
  boolean_T tooSmallX;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0] + 1;
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr = (WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4];
  b_mLB = (WorkingSet->sizes[3] + WorkingSet->sizes[4]) + (WorkingSet->sizes[0] <<
    1);
  qpoptions_MaxIterations = 10 * muIntScalarMax_sint32(WorkingSet->nVar, b_mLB);
  TrialState->steplength = 1.0;
  Flags.fevalOK = true;
  Flags.stepAccepted = false;
  Flags.failedLineSearch = false;
  Flags.stepType = 1;
  b_mLB = WorkingSet->sizes[3];
  b_mUB = WorkingSet->sizes[4];
  computeGradLag(TrialState->gradLag, WorkingSet->nVar, TrialState->grad,
                 WorkingSet->indexFixed, WorkingSet->sizes[0],
                 WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
                 WorkingSet->sizes[4], TrialState->lambdasqp);
  optimRelativeFactor = muDoubleScalarMax(1.0, muDoubleScalarAbs
    (TrialState->grad->data[ixamax(WorkingSet->nVar, TrialState->grad) - 1]));
  if (muDoubleScalarIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }

  feasError = 0.0;
  for (idx = 0; idx < b_mLB; idx++) {
    idxFiniteLB = WorkingSet->indexLB->data[idx] - 1;
    feasError = muDoubleScalarMax(feasError, lb->data[idxFiniteLB] -
      TrialState->xstarsqp->data[idxFiniteLB]);
  }

  for (idx = 0; idx < b_mUB; idx++) {
    b_mLB = WorkingSet->indexUB->data[idx] - 1;
    feasError = muDoubleScalarMax(feasError, TrialState->xstarsqp->data[b_mLB] -
      ub->data[b_mLB]);
  }

  MeritFunction->nlpPrimalFeasError = feasError;
  MeritFunction->feasRelativeFactor = muDoubleScalarMax(1.0, feasError);
  isFeasible = (feasError <= 0.001 * MeritFunction->feasRelativeFactor);
  b_mUB = WorkingSet->nVar;
  gradLag = TrialState->gradLag;
  hasUB = true;
  feasError = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= b_mUB - 1)) {
    hasUB = ((!muDoubleScalarIsInf(gradLag->data[idx])) && (!muDoubleScalarIsNaN
              (gradLag->data[idx])));
    if (!hasUB) {
      exitg1 = true;
    } else {
      feasError = muDoubleScalarMax(feasError, muDoubleScalarAbs(gradLag->
        data[idx]));
      idx++;
    }
  }

  Flags.gradOK = hasUB;
  MeritFunction->nlpDualFeasError = feasError;
  if (!hasUB) {
    Flags.done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = muDoubleScalarMax
      (MeritFunction->nlpDualFeasError, 0.0);
    xcopy((WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4],
          TrialState->lambdasqp, TrialState->lambdasqp_old);
    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 *
                       optimRelativeFactor)) {
      Flags.done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags.done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags.done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        if (TrialState->FunctionEvaluations >= 5000) {
          Flags.done = true;
          TrialState->sqpExitFlag = 0;
        }
      }
    }
  }

  TrialState->sqpFval_old = TrialState->sqpFval;
  b_mUB = TrialState->xstarsqp->size[1];
  xcopy(TrialState->xstarsqp->size[1], TrialState->xstarsqp,
        TrialState->xstarsqp_old);
  for (b_mLB = 0; b_mLB < b_mUB; b_mLB++) {
    TrialState->grad_old->data[b_mLB] = TrialState->grad->data[b_mLB];
  }

  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }

  emxInit_real_T(&b_TrialState, 2, true);
  while (!Flags.done) {
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      isFeasible = (lb->size[1] != 0);
      hasUB = (ub->size[1] != 0);
      if (isFeasible) {
        for (idx = 0; idx < mLB; idx++) {
          WorkingSet->lb->data[WorkingSet->indexLB->data[idx] - 1] = -lb->
            data[WorkingSet->indexLB->data[idx] - 1] + TrialState->
            xstarsqp->data[WorkingSet->indexLB->data[idx] - 1];
        }
      }

      if (hasUB) {
        for (idx = 0; idx < mUB; idx++) {
          WorkingSet->ub->data[WorkingSet->indexUB->data[idx] - 1] = ub->
            data[WorkingSet->indexUB->data[idx] - 1] - TrialState->
            xstarsqp->data[WorkingSet->indexUB->data[idx] - 1];
        }
      }

      if (isFeasible && hasUB) {
        for (idx = 0; idx <= mFixed - 2; idx++) {
          b_mLB = WorkingSet->indexFixed->data[idx] - 1;
          WorkingSet->ub->data[WorkingSet->indexFixed->data[idx] - 1] = ub->
            data[WorkingSet->indexFixed->data[idx] - 1] - TrialState->
            xstarsqp->data[WorkingSet->indexFixed->data[idx] - 1];
          WorkingSet->bwset->data[idx] = ub->data[b_mLB] - TrialState->
            xstarsqp->data[b_mLB];
        }
      }

      if (WorkingSet->nActiveConstr > mFixed - 1) {
        if (1 < mFixed) {
          b_mLB = mFixed;
        } else {
          b_mLB = 1;
        }

        i = WorkingSet->nActiveConstr;
        for (idx = b_mLB; idx <= i; idx++) {
          switch (WorkingSet->Wid->data[idx - 1]) {
           case 4:
            WorkingSet->bwset->data[idx - 1] = WorkingSet->lb->data
              [WorkingSet->indexLB->data[WorkingSet->Wlocalidx->data[idx - 1] -
              1] - 1];
            break;

           case 5:
            WorkingSet->bwset->data[idx - 1] = WorkingSet->ub->data
              [WorkingSet->indexUB->data[WorkingSet->Wlocalidx->data[idx - 1] -
              1] - 1];
            break;

           default:
            /* A check that is always false is detected at compile-time. Eliminating code that follows. */
            break;
          }
        }
      }

      expl_temp.IterDisplayQP = false;
      expl_temp.RemainFeasible = false;
      expl_temp.ProbRelTolFactor = 1.0;
      expl_temp.ConstrRelTolFactor = 1.0;
      expl_temp.PricingTolerance = 0.0;
      expl_temp.ObjectiveLimit = rtMinusInf;
      expl_temp.ConstraintTolerance = 0.001;
      expl_temp.OptimalityTolerance = 2.2204460492503131E-14;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (i = 0; i < 7; i++) {
        expl_temp.SolverName[i] = qpoptions_SolverName[i];
      }

      b_expl_temp = expl_temp;
      Flags.stepAccepted = step(&Flags.stepType, Hessian, lb, ub, TrialState,
        MeritFunction, memspace, WorkingSet, QRManager, CholManager, QPObjective,
        &b_expl_temp);
      if (Flags.stepAccepted) {
        for (b_mLB = 0; b_mLB < nVar; b_mLB++) {
          TrialState->xstarsqp->data[b_mLB] += TrialState->delta_x->data[b_mLB];
        }

        i = b_TrialState->size[0] * b_TrialState->size[1];
        b_TrialState->size[0] = 1;
        b_TrialState->size[1] = TrialState->xstarsqp->size[1];
        emxEnsureCapacity_real_T(b_TrialState, i);
        b_mLB = TrialState->xstarsqp->size[0] * TrialState->xstarsqp->size[1] -
          1;
        for (i = 0; i <= b_mLB; i++) {
          b_TrialState->data[i] = TrialState->xstarsqp->data[i];
        }

        evalObjAndConstr(FcnEvaluator->objfun.tunableEnvironment.f1,
                         FcnEvaluator->objfun.tunableEnvironment.f3,
                         FcnEvaluator->objfun.tunableEnvironment.f4,
                         FcnEvaluator->objfun.tunableEnvironment.f5,
                         FcnEvaluator->objfun.tunableEnvironment.f6,
                         FcnEvaluator->objfun.tunableEnvironment.f7.int_method,
                         FcnEvaluator->objfun.tunableEnvironment.f7.int_steps,
                         FcnEvaluator->objfun.tunableEnvironment.f7.b,
                         FcnEvaluator->objfun.tunableEnvironment.f7.p_a1,
                         FcnEvaluator->objfun.tunableEnvironment.f7.p_a2,
                         FcnEvaluator->objfun.tunableEnvironment.f7.g,
                         FcnEvaluator->objfun.tunableEnvironment.f7.m,
                         FcnEvaluator->objfun.tunableEnvironment.f7.w1,
                         FcnEvaluator->objfun.tunableEnvironment.f7.w2,
                         FcnEvaluator->objfun.tunableEnvironment.f7.mpc_dt,
                         b_TrialState, &TrialState->sqpFval, &b_mLB);
        Flags.fevalOK = (b_mLB == 1);
        TrialState->FunctionEvaluations++;
        if (Flags.fevalOK) {
          MeritFunction->phiFullStep = TrialState->sqpFval +
            MeritFunction->penaltyParam * 0.0;
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
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          isFeasible = true;
        } else {
          isFeasible = false;
        }

        hasUB = Flags.fevalOK;
        b_mUB = WorkingSet->nVar - 1;
        feasError = 1.0;
        idxFiniteLB = 1;
        optimRelativeFactor = MeritFunction->phiFullStep;
        xcopy(WorkingSet->nVar, TrialState->delta_x, TrialState->searchDir);
        do {
          exitg2 = 0;
          if (TrialState->FunctionEvaluations < 5000) {
            if (hasUB && (optimRelativeFactor <= MeritFunction->phi + feasError *
                          0.0001 * MeritFunction->phiPrimePlus)) {
              exitg2 = 1;
            } else {
              feasError *= 0.7;
              for (idx = 0; idx <= b_mUB; idx++) {
                TrialState->delta_x->data[idx] = feasError * TrialState->
                  xstar->data[idx];
              }

              if (isFeasible) {
                xaxpy(b_mUB + 1, feasError * feasError, TrialState->socDirection,
                      TrialState->delta_x);
              }

              tooSmallX = true;
              idx = 0;
              exitg1 = false;
              while ((!exitg1) && (idx <= b_mUB)) {
                if (1.0E-6 * muDoubleScalarMax(1.0, muDoubleScalarAbs
                     (TrialState->xstarsqp->data[idx])) <= muDoubleScalarAbs
                    (TrialState->delta_x->data[idx])) {
                  tooSmallX = false;
                  exitg1 = true;
                } else {
                  idx++;
                }
              }

              if (tooSmallX) {
                idxFiniteLB = -2;
                exitg2 = 1;
              } else {
                for (idx = 0; idx <= b_mUB; idx++) {
                  TrialState->xstarsqp->data[idx] = TrialState->
                    xstarsqp_old->data[idx] + TrialState->delta_x->data[idx];
                }

                i = b_TrialState->size[0] * b_TrialState->size[1];
                b_TrialState->size[0] = 1;
                b_TrialState->size[1] = TrialState->xstarsqp->size[1];
                emxEnsureCapacity_real_T(b_TrialState, i);
                b_mLB = TrialState->xstarsqp->size[0] * TrialState->
                  xstarsqp->size[1] - 1;
                for (i = 0; i <= b_mLB; i++) {
                  b_TrialState->data[i] = TrialState->xstarsqp->data[i];
                }

                evalObjAndConstr(FcnEvaluator->objfun.tunableEnvironment.f1,
                                 FcnEvaluator->objfun.tunableEnvironment.f3,
                                 FcnEvaluator->objfun.tunableEnvironment.f4,
                                 FcnEvaluator->objfun.tunableEnvironment.f5,
                                 FcnEvaluator->objfun.tunableEnvironment.f6,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.int_method,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.int_steps,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.b,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.p_a1,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.p_a2,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.g,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.m,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.w1,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.w2,
                                 FcnEvaluator->objfun.tunableEnvironment.f7.mpc_dt,
                                 b_TrialState, &TrialState->sqpFval, &b_mLB);
                TrialState->FunctionEvaluations++;
                hasUB = (b_mLB == 1);
                if (hasUB) {
                  optimRelativeFactor = TrialState->sqpFval +
                    MeritFunction->penaltyParam * 0.0;
                } else {
                  optimRelativeFactor = rtInf;
                }
              }
            }
          } else {
            idxFiniteLB = 0;
            exitg2 = 1;
          }
        } while (exitg2 == 0);

        Flags.fevalOK = hasUB;
        TrialState->steplength = feasError;
        if (idxFiniteLB > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }

    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      for (idx = 0; idx < nVar; idx++) {
        TrialState->xstarsqp->data[idx] = TrialState->xstarsqp_old->data[idx] +
          TrialState->delta_x->data[idx];
      }

      for (idx = 0; idx < mConstr; idx++) {
        TrialState->lambdasqp->data[idx] += TrialState->steplength *
          (TrialState->lambda->data[idx] - TrialState->lambdasqp->data[idx]);
      }

      TrialState->sqpFval_old = TrialState->sqpFval;
      b_mUB = TrialState->xstarsqp->size[1];
      xcopy(TrialState->xstarsqp->size[1], TrialState->xstarsqp,
            TrialState->xstarsqp_old);
      for (b_mLB = 0; b_mLB < b_mUB; b_mLB++) {
        TrialState->grad_old->data[b_mLB] = TrialState->grad->data[b_mLB];
      }

      Flags.gradOK = computeForwardDifferences(FiniteDifferences,
        TrialState->sqpFval, TrialState->xstarsqp, TrialState->grad, lb, ub);
      TrialState->FunctionEvaluations += FiniteDifferences->numEvals;
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      xcopy(TrialState->xstarsqp->size[1], TrialState->xstarsqp_old,
            TrialState->xstarsqp);
    }

    test_exit(&Flags, memspace, MeritFunction, WorkingSet, TrialState, QRManager,
              lb, ub);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      computeDeltaLag(nVar, TrialState->delta_gradLag, TrialState->grad,
                      TrialState->grad_old);
      BFGSUpdate(nVar, Hessian, TrialState->delta_x, TrialState->delta_gradLag,
                 memspace->workspace_double);
      TrialState->sqpIterations++;
    }
  }

  emxFree_real_T(&b_TrialState);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (driver.c) */
