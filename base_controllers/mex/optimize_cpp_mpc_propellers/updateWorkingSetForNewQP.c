/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * updateWorkingSetForNewQP.c
 *
 * Code generation for function 'updateWorkingSetForNewQP'
 *
 */

/* Include files */
#include "updateWorkingSetForNewQP.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include <string.h>

/* Function Definitions */
void b_updateWorkingSetForNewQP(const emxArray_real_T *xk, j_struct_T
  *WorkingSet, int32_T mIneq, int32_T mNonlinIneq, const emxArray_real_T *cIneq,
  int32_T mLB, const emxArray_real_T *lb, int32_T mUB, const emxArray_real_T *ub,
  int32_T mFixed)
{
  int32_T i;
  int32_T idx;
  int32_T idx_finite;
  int32_T idx_local_tmp;
  boolean_T hasLB;
  boolean_T hasUB;
  for (idx = 0; idx < mIneq; idx++) {
    WorkingSet->bineq->data[idx] = -cIneq->data[idx];
  }

  hasLB = (lb->size[1] != 0);
  hasUB = (ub->size[1] != 0);
  if (hasLB) {
    for (idx = 0; idx < mLB; idx++) {
      WorkingSet->lb->data[WorkingSet->indexLB->data[idx] - 1] = -lb->
        data[WorkingSet->indexLB->data[idx] - 1] + xk->data[WorkingSet->
        indexLB->data[idx] - 1];
    }
  }

  if (hasUB) {
    for (idx = 0; idx < mUB; idx++) {
      WorkingSet->ub->data[WorkingSet->indexUB->data[idx] - 1] = ub->
        data[WorkingSet->indexUB->data[idx] - 1] - xk->data[WorkingSet->
        indexUB->data[idx] - 1];
    }
  }

  if (hasLB && hasUB) {
    for (idx = 0; idx < mFixed; idx++) {
      idx_finite = WorkingSet->indexFixed->data[idx] - 1;
      WorkingSet->ub->data[WorkingSet->indexFixed->data[idx] - 1] = ub->
        data[WorkingSet->indexFixed->data[idx] - 1] - xk->data
        [WorkingSet->indexFixed->data[idx] - 1];
      WorkingSet->bwset->data[idx] = ub->data[idx_finite] - xk->data[idx_finite];
    }
  }

  if (WorkingSet->nActiveConstr > mFixed) {
    if (1 < mFixed + 1) {
      idx_finite = mFixed + 1;
    } else {
      idx_finite = 1;
    }

    i = WorkingSet->nActiveConstr;
    for (idx = idx_finite; idx <= i; idx++) {
      idx_local_tmp = WorkingSet->Wlocalidx->data[idx - 1];
      switch (WorkingSet->Wid->data[idx - 1]) {
       case 4:
        WorkingSet->bwset->data[idx - 1] = WorkingSet->lb->data
          [WorkingSet->indexLB->data[WorkingSet->Wlocalidx->data[idx - 1] - 1] -
          1];
        break;

       case 5:
        WorkingSet->bwset->data[idx - 1] = WorkingSet->ub->data
          [WorkingSet->indexUB->data[WorkingSet->Wlocalidx->data[idx - 1] - 1] -
          1];
        break;

       default:
        WorkingSet->bwset->data[idx - 1] = WorkingSet->bineq->data[idx_local_tmp
          - 1];
        if ((mNonlinIneq > 0) && (idx_local_tmp >= mNonlinIneq)) {
          b_xcopy(WorkingSet->nVar, WorkingSet->Aineq, WorkingSet->ldA *
                  (idx_local_tmp - 1) + 1, WorkingSet->ATwset, WorkingSet->ldA *
                  (idx - 1) + 1);
        }
        break;
      }
    }
  }
}

void updateWorkingSetForNewQP(j_struct_T *WorkingSet, int32_T mIneq, const
  emxArray_real_T *cIneq, int32_T mLB, const emxArray_real_T *lb, int32_T mUB,
  const emxArray_real_T *ub, int32_T mFixed)
{
  int32_T idx;
  int32_T idx_finite;
  boolean_T hasLB;
  boolean_T hasUB;
  for (idx = 0; idx < mIneq; idx++) {
    WorkingSet->bineq->data[idx] = -cIneq->data[idx];
  }

  hasLB = (lb->size[1] != 0);
  hasUB = (ub->size[1] != 0);
  if (hasLB) {
    for (idx = 0; idx < mLB; idx++) {
      WorkingSet->lb->data[WorkingSet->indexLB->data[idx] - 1] = -lb->
        data[WorkingSet->indexLB->data[idx] - 1];
    }
  }

  if (hasUB) {
    for (idx = 0; idx < mUB; idx++) {
      WorkingSet->ub->data[WorkingSet->indexUB->data[idx] - 1] = ub->
        data[WorkingSet->indexUB->data[idx] - 1];
    }
  }

  if (hasLB && hasUB) {
    for (idx = 0; idx < mFixed; idx++) {
      idx_finite = WorkingSet->indexFixed->data[idx];
      WorkingSet->ub->data[WorkingSet->indexFixed->data[idx] - 1] = ub->
        data[WorkingSet->indexFixed->data[idx] - 1];
      WorkingSet->bwset->data[idx] = ub->data[idx_finite - 1];
    }
  }
}

/* End of code generation (updateWorkingSetForNewQP.c) */
