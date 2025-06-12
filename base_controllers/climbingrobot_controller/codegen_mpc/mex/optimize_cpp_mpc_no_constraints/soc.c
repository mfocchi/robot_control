/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * soc.c
 *
 * Code generation for function 'soc'
 *
 */

/* Include files */
#include "soc.h"
#include "driver1.h"
#include "optimize_cpp_mpc_no_constraints_internal_types.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xcopy.h"
#include "xnrm2.h"
#include <string.h>

/* Function Definitions */
boolean_T soc(const emxArray_real_T *Hessian, const emxArray_real_T *grad,
              d_struct_T *TrialState, c_struct_T *memspace, j_struct_T
              *WorkingSet, g_struct_T *QRManager, h_struct_T *CholManager,
              i_struct_T *QPObjective, const b_struct_T *qpoptions)
{
  b_struct_T b_qpoptions;
  real_T oldDirIdx;
  int32_T i;
  int32_T mConstrMax;
  int32_T nVar;
  boolean_T success;
  nVar = WorkingSet->nVar;
  mConstrMax = WorkingSet->mConstrMax;
  xcopy(WorkingSet->nVar, TrialState->xstarsqp_old, TrialState->xstarsqp);
  for (i = 0; i < nVar; i++) {
    TrialState->socDirection->data[i] = TrialState->xstar->data[i];
  }

  xcopy(WorkingSet->mConstrMax, TrialState->lambda, TrialState->lambda_old);
  xcopy(WorkingSet->nVar, TrialState->xstarsqp, TrialState->xstar);
  b_qpoptions = *qpoptions;
  b_driver(Hessian, grad, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
  for (i = 0; i < nVar; i++) {
    oldDirIdx = TrialState->socDirection->data[i];
    TrialState->socDirection->data[i] = TrialState->xstar->data[i] -
      TrialState->socDirection->data[i];
    TrialState->xstar->data[i] = oldDirIdx;
  }

  success = (xnrm2(nVar, TrialState->socDirection) <= 2.0 * xnrm2(nVar,
              TrialState->xstar));
  if (!success) {
    xcopy(mConstrMax, TrialState->lambda_old, TrialState->lambda);
  } else {
    sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                 WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                 WorkingSet->Wlocalidx, memspace->workspace_double);
  }

  return success;
}

/* End of code generation (soc.c) */
