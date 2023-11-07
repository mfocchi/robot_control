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
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "optimize_cpp_mpc_propellers_internal_types.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xcopy.h"
#include "xgemv.h"
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
  int32_T idx;
  int32_T idxIneqOffset;
  int32_T idx_Aineq;
  int32_T idx_IneqLocal_tmp;
  int32_T idx_Partition;
  int32_T idx_lower;
  int32_T idx_upper;
  int32_T mConstrMax;
  int32_T mIneq;
  int32_T nVar;
  int32_T nWIneq_old;
  int32_T nWLower_old;
  int32_T nWUpper_old;
  boolean_T success;
  nWIneq_old = WorkingSet->nWConstr[2];
  nWLower_old = WorkingSet->nWConstr[3];
  nWUpper_old = WorkingSet->nWConstr[4];
  nVar = WorkingSet->nVar;
  mConstrMax = WorkingSet->mConstrMax;
  xcopy(WorkingSet->nVar, TrialState->xstarsqp_old, TrialState->xstarsqp);
  for (mIneq = 0; mIneq < nVar; mIneq++) {
    TrialState->socDirection->data[mIneq] = TrialState->xstar->data[mIneq];
  }

  xcopy(WorkingSet->mConstrMax, TrialState->lambda, TrialState->lambda_old);
  mIneq = WorkingSet->sizes[2];
  idxIneqOffset = WorkingSet->isActiveIdx[2];
  if (WorkingSet->sizes[2] > 0) {
    for (idx = 0; idx < mIneq; idx++) {
      WorkingSet->bineq->data[idx] = -TrialState->cIneq->data[idx];
    }

    j_xgemv(WorkingSet->nVar, WorkingSet->sizes[2], WorkingSet->Aineq,
            WorkingSet->ldA, TrialState->searchDir, WorkingSet->bineq);
    idx_Aineq = 1;
    idx_lower = WorkingSet->sizes[2] + 1;
    idx_upper = (WorkingSet->sizes[2] + WorkingSet->sizes[3]) + 1;
    mIneq = WorkingSet->nActiveConstr;
    for (idx = idxIneqOffset; idx <= mIneq; idx++) {
      idx_IneqLocal_tmp = WorkingSet->Wlocalidx->data[idx - 1];
      switch (WorkingSet->Wid->data[idx - 1]) {
       case 3:
        idx_Partition = idx_Aineq;
        idx_Aineq++;
        WorkingSet->bwset->data[idx - 1] = WorkingSet->bineq->
          data[idx_IneqLocal_tmp - 1];
        break;

       case 4:
        idx_Partition = idx_lower;
        idx_lower++;
        break;

       default:
        idx_Partition = idx_upper;
        idx_upper++;
        break;
      }

      TrialState->workingset_old->data[idx_Partition - 1] = idx_IneqLocal_tmp;
    }
  }

  xcopy(WorkingSet->nVar, TrialState->xstarsqp, TrialState->xstar);
  b_qpoptions = *qpoptions;
  b_driver(Hessian, grad, TrialState, memspace, WorkingSet, QRManager,
           CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
  for (idx = 0; idx < nVar; idx++) {
    oldDirIdx = TrialState->socDirection->data[idx];
    TrialState->socDirection->data[idx] = TrialState->xstar->data[idx] -
      TrialState->socDirection->data[idx];
    TrialState->xstar->data[idx] = oldDirIdx;
  }

  success = (xnrm2(nVar, TrialState->socDirection) <= 2.0 * xnrm2(nVar,
              TrialState->xstar));
  mIneq = WorkingSet->sizes[2];
  idx_upper = WorkingSet->sizes[3];
  if (WorkingSet->sizes[2] > 0) {
    for (idx = 0; idx < mIneq; idx++) {
      WorkingSet->bineq->data[idx] = -TrialState->cIneq->data[idx];
    }

    if (!success) {
      idx_Aineq = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
      idx_lower = WorkingSet->nActiveConstr;
      for (idx_IneqLocal_tmp = idx_Aineq; idx_IneqLocal_tmp <= idx_lower;
           idx_IneqLocal_tmp++) {
        WorkingSet->isActiveConstr->data[(WorkingSet->isActiveIdx
          [WorkingSet->Wid->data[idx_IneqLocal_tmp - 1] - 1] +
          WorkingSet->Wlocalidx->data[idx_IneqLocal_tmp - 1]) - 2] = false;
      }

      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr = WorkingSet->nWConstr[0] + WorkingSet->
        nWConstr[1];
      for (idx = 0; idx < nWIneq_old; idx++) {
        idx_Aineq = TrialState->workingset_old->data[idx] - 1;
        WorkingSet->nWConstr[2]++;
        WorkingSet->isActiveConstr->data[(WorkingSet->isActiveIdx[2] + idx_Aineq)
          - 1] = true;
        WorkingSet->nActiveConstr++;
        WorkingSet->Wid->data[WorkingSet->nActiveConstr - 1] = 3;
        WorkingSet->Wlocalidx->data[WorkingSet->nActiveConstr - 1] =
          TrialState->workingset_old->data[idx];
        b_xcopy(WorkingSet->nVar, WorkingSet->Aineq, WorkingSet->ldA * idx_Aineq
                + 1, WorkingSet->ATwset, WorkingSet->ldA *
                (WorkingSet->nActiveConstr - 1) + 1);
        WorkingSet->bwset->data[WorkingSet->nActiveConstr - 1] =
          WorkingSet->bineq->data[TrialState->workingset_old->data[idx] - 1];
      }

      for (idx = 0; idx < nWLower_old; idx++) {
        addBoundToActiveSetMatrix_(WorkingSet, 4, TrialState->
          workingset_old->data[idx + mIneq]);
      }

      for (idx = 0; idx < nWUpper_old; idx++) {
        addBoundToActiveSetMatrix_(WorkingSet, 5, TrialState->
          workingset_old->data[(idx + mIneq) + idx_upper]);
      }
    }
  }

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
