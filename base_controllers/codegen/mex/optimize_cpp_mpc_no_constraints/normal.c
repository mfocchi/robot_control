/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * normal.c
 *
 * Code generation for function 'normal'
 *
 */

/* Include files */
#include "normal.h"
#include "driver1.h"
#include "optimize_cpp_mpc_no_constraints_internal_types.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void normal(const emxArray_real_T *Hessian, const emxArray_real_T *grad,
            d_struct_T *TrialState, k_struct_T *MeritFunction, c_struct_T
            *memspace, j_struct_T *WorkingSet, g_struct_T *QRManager, h_struct_T
            *CholManager, i_struct_T *QPObjective, const b_struct_T *qpoptions)
{
  b_struct_T b_qpoptions;
  real_T linearizedConstrViolPrev;
  real_T penaltyParamTrial;
  b_qpoptions = *qpoptions;
  b_driver(Hessian, grad, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
  if (TrialState->state > 0) {
    penaltyParamTrial = MeritFunction->penaltyParam;
    linearizedConstrViolPrev = MeritFunction->linearizedConstrViol;
    MeritFunction->linearizedConstrViol = 0.0;
    if ((linearizedConstrViolPrev > 2.2204460492503131E-16) &&
        (TrialState->fstar > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        penaltyParamTrial = 1.0;
      } else {
        penaltyParamTrial = 1.5;
      }

      penaltyParamTrial = penaltyParamTrial * TrialState->fstar /
        linearizedConstrViolPrev;
    }

    if (penaltyParamTrial < MeritFunction->penaltyParam) {
      MeritFunction->phi = TrialState->sqpFval;
      if (MeritFunction->initFval - TrialState->sqpFval > (real_T)
          MeritFunction->nPenaltyDecreases * MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) > TrialState->sqpIterations)
        {
          MeritFunction->threshold *= 10.0;
        }

        MeritFunction->penaltyParam = muDoubleScalarMax(penaltyParamTrial,
          1.0E-10);
      } else {
        MeritFunction->phi = TrialState->sqpFval + MeritFunction->penaltyParam *
          0.0;
      }
    } else {
      MeritFunction->penaltyParam = muDoubleScalarMax(penaltyParamTrial, 1.0E-10);
      MeritFunction->phi = TrialState->sqpFval + MeritFunction->penaltyParam *
        0.0;
    }

    MeritFunction->phiPrimePlus = muDoubleScalarMin(TrialState->fstar -
      MeritFunction->penaltyParam * 0.0, 0.0);
  }

  sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr, WorkingSet->sizes,
               WorkingSet->isActiveIdx, WorkingSet->Wid, WorkingSet->Wlocalidx,
               memspace->workspace_double);
}

/* End of code generation (normal.c) */
