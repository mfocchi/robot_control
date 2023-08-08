/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
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
#include "computeGradLag.h"
#include "computeLambdaLSQ.h"
#include "ixamax.h"
#include "optimize_cpp_mpc_propellers_internal_types.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "updateWorkingSetForNewQP.h"
#include "xcopy.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void test_exit(struct_T *Flags, c_struct_T *memspace, k_struct_T *MeritFunction,
               const emxArray_real_T *fscales_cineq_constraint, j_struct_T
               *WorkingSet, d_struct_T *TrialState, g_struct_T *QRManager, const
               emxArray_real_T *lb, const emxArray_real_T *ub)
{
  emxArray_real_T *gradLag;
  real_T d;
  real_T feasError;
  real_T nlpComplErrorTmp;
  real_T optimRelativeFactor;
  int32_T b_mIneq;
  int32_T idx;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mLambda;
  int32_T mUB;
  int32_T nVar;
  boolean_T dxTooSmall;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T isFeasible;
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mIneq = WorkingSet->sizes[2];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mLambda = ((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes[3])
    + WorkingSet->sizes[4];
  computeGradLag(TrialState->gradLag, WorkingSet->ldA, WorkingSet->nVar,
                 TrialState->grad, WorkingSet->sizes[2], WorkingSet->Aineq,
                 WorkingSet->indexFixed, WorkingSet->sizes[0],
                 WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
                 WorkingSet->sizes[4], TrialState->lambdasqp);
  optimRelativeFactor = muDoubleScalarMax(1.0, muDoubleScalarAbs
    (TrialState->grad->data[ixamax(WorkingSet->nVar, TrialState->grad) - 1]));
  if (muDoubleScalarIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }

  feasError = 0.0;
  b_mIneq = WorkingSet->sizes[2];
  for (idx = 0; idx < b_mIneq; idx++) {
    feasError = muDoubleScalarMax(feasError, TrialState->cIneq->data[idx]);
  }

  for (idx = 0; idx < mLB; idx++) {
    b_mIneq = WorkingSet->indexLB->data[idx] - 1;
    feasError = muDoubleScalarMax(feasError, lb->data[b_mIneq] -
      TrialState->xstarsqp->data[b_mIneq]);
  }

  for (idx = 0; idx < mUB; idx++) {
    b_mIneq = WorkingSet->indexUB->data[idx] - 1;
    feasError = muDoubleScalarMax(feasError, TrialState->xstarsqp->data[b_mIneq]
      - ub->data[b_mIneq]);
  }

  MeritFunction->nlpPrimalFeasError = feasError;
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = muDoubleScalarMax(1.0, feasError);
  }

  isFeasible = (feasError <= 0.001 * MeritFunction->feasRelativeFactor);
  b_mIneq = WorkingSet->nVar;
  gradLag = TrialState->gradLag;
  dxTooSmall = true;
  feasError = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= b_mIneq - 1)) {
    dxTooSmall = ((!muDoubleScalarIsInf(gradLag->data[idx])) &&
                  (!muDoubleScalarIsNaN(gradLag->data[idx])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      feasError = muDoubleScalarMax(feasError, muDoubleScalarAbs(gradLag->
        data[idx]));
      idx++;
    }
  }

  Flags->gradOK = dxTooSmall;
  MeritFunction->nlpDualFeasError = feasError;
  if (!Flags->gradOK) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = computeComplError(fscales_cineq_constraint,
      TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq,
      WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
      WorkingSet->sizes[4], ub, TrialState->lambdasqp, WorkingSet->sizes[0] + 1);
    MeritFunction->firstOrderOpt = muDoubleScalarMax
      (MeritFunction->nlpDualFeasError, MeritFunction->nlpComplError);
    if (TrialState->sqpIterations > 1) {
      computeGradLag(memspace->workspace_double, WorkingSet->ldA,
                     WorkingSet->nVar, TrialState->grad, WorkingSet->sizes[2],
                     WorkingSet->Aineq, WorkingSet->indexFixed,
                     WorkingSet->sizes[0], WorkingSet->indexLB,
                     WorkingSet->sizes[3], WorkingSet->indexUB,
                     WorkingSet->sizes[4], TrialState->lambdasqp_old);
      b_mIneq = WorkingSet->nVar;
      gradLag = memspace->workspace_double;
      feasError = 0.0;
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= b_mIneq - 1)) {
        dxTooSmall = ((!muDoubleScalarIsInf(gradLag->data[idx])) &&
                      (!muDoubleScalarIsNaN(gradLag->data[idx])));
        if (!dxTooSmall) {
          exitg1 = true;
        } else {
          feasError = muDoubleScalarMax(feasError, muDoubleScalarAbs
            (gradLag->data[idx]));
          idx++;
        }
      }

      nlpComplErrorTmp = computeComplError(fscales_cineq_constraint,
        TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq,
        WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
        WorkingSet->sizes[4], ub, TrialState->lambdasqp_old, WorkingSet->sizes[0]
        + 1);
      d = muDoubleScalarMax(feasError, nlpComplErrorTmp);
      if (d < muDoubleScalarMax(MeritFunction->nlpDualFeasError,
           MeritFunction->nlpComplError)) {
        MeritFunction->nlpDualFeasError = feasError;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = d;
        xcopy(mLambda, TrialState->lambdasqp_old, TrialState->lambdasqp);
      } else {
        xcopy(mLambda, TrialState->lambdasqp, TrialState->lambdasqp_old);
      }
    } else {
      xcopy(mLambda, TrialState->lambdasqp, TrialState->lambdasqp_old);
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 *
                       optimRelativeFactor) && (MeritFunction->nlpComplError <=
         1.0E-6 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          idx = 0;
          exitg1 = false;
          while ((!exitg1) && (idx <= nVar - 1)) {
            if (1.0E-6 * muDoubleScalarMax(1.0, muDoubleScalarAbs
                 (TrialState->xstarsqp->data[idx])) <= muDoubleScalarAbs
                (TrialState->delta_x->data[idx])) {
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
              b_mIneq = WorkingSet->nActiveConstr;
              if (WorkingSet->nActiveConstr > 0) {
                if (TrialState->mNonlinIneq > 0) {
                  b_updateWorkingSetForNewQP(TrialState->xstarsqp, WorkingSet,
                    WorkingSet->sizes[2], TrialState->mNonlinIneq,
                    TrialState->cIneq, WorkingSet->sizes[3], lb,
                    WorkingSet->sizes[4], ub, WorkingSet->sizes[0]);
                }

                computeLambdaLSQ(nVar, b_mIneq, QRManager, WorkingSet->ATwset,
                                 TrialState->grad, TrialState->lambda,
                                 memspace->workspace_double);
                b_mIneq = mFixed + 1;
                for (idx = b_mIneq; idx <= mFixed; idx++) {
                  TrialState->lambda->data[idx - 1] = -TrialState->lambda->
                    data[idx - 1];
                }

                sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                             WorkingSet->sizes, WorkingSet->isActiveIdx,
                             WorkingSet->Wid, WorkingSet->Wlocalidx,
                             memspace->workspace_double);
                computeGradLag(memspace->workspace_double, WorkingSet->ldA, nVar,
                               TrialState->grad, mIneq, WorkingSet->Aineq,
                               WorkingSet->indexFixed, mFixed,
                               WorkingSet->indexLB, mLB, WorkingSet->indexUB,
                               mUB, TrialState->lambda);
                gradLag = memspace->workspace_double;
                feasError = 0.0;
                idx = 0;
                exitg1 = false;
                while ((!exitg1) && (idx <= nVar - 1)) {
                  dxTooSmall = ((!muDoubleScalarIsInf(gradLag->data[idx])) &&
                                (!muDoubleScalarIsNaN(gradLag->data[idx])));
                  if (!dxTooSmall) {
                    exitg1 = true;
                  } else {
                    feasError = muDoubleScalarMax(feasError, muDoubleScalarAbs
                      (gradLag->data[idx]));
                    idx++;
                  }
                }

                nlpComplErrorTmp = computeComplError(fscales_cineq_constraint,
                  TrialState->xstarsqp, mIneq, TrialState->cIneq,
                  WorkingSet->indexLB, mLB, lb, WorkingSet->indexUB, mUB, ub,
                  TrialState->lambda, mFixed + 1);
                if ((feasError <= 1.0E-6 * optimRelativeFactor) &&
                    (nlpComplErrorTmp <= 1.0E-6 * optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = feasError;
                  MeritFunction->nlpComplError = nlpComplErrorTmp;
                  MeritFunction->firstOrderOpt = muDoubleScalarMax(feasError,
                    nlpComplErrorTmp);
                  xcopy(mLambda, TrialState->lambda, TrialState->lambdasqp);
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
          } else {
            if (TrialState->FunctionEvaluations >= 5000) {
              Flags->done = true;
              TrialState->sqpExitFlag = 0;
            }
          }
        }
      }
    }
  }
}

/* End of code generation (test_exit.c) */
