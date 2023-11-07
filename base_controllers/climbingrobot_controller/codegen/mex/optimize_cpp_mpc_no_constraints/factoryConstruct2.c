/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct2.c
 *
 * Code generation for function 'factoryConstruct2'
 *
 */

/* Include files */
#include "factoryConstruct2.h"
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void c_factoryConstruct(int32_T mLB, const emxArray_int32_T *indexLB, int32_T
  mUB, const emxArray_int32_T *indexUB, int32_T mFixed, const emxArray_int32_T
  *indexFixed, int32_T nVar, int32_T nVarMax, int32_T mConstrMax, j_struct_T
  *obj)
{
  int32_T i;
  i = (mLB + mUB) + mFixed;
  obj->mConstr = i;
  obj->mConstrOrig = i;
  obj->mConstrMax = mConstrMax;
  obj->nVar = nVar;
  obj->nVarOrig = nVar;
  obj->nVarMax = nVarMax;
  obj->ldA = nVarMax;
  i = obj->lb->size[0];
  obj->lb->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->lb, i);
  i = obj->ub->size[0];
  obj->ub->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->ub, i);
  i = obj->indexLB->size[0];
  obj->indexLB->size[0] = nVarMax;
  emxEnsureCapacity_int32_T(obj->indexLB, i);
  i = obj->indexUB->size[0];
  obj->indexUB->size[0] = nVarMax;
  emxEnsureCapacity_int32_T(obj->indexUB, i);
  i = obj->indexFixed->size[0];
  obj->indexFixed->size[0] = nVarMax;
  emxEnsureCapacity_int32_T(obj->indexFixed, i);
  obj->mEqRemoved = 0;
  i = obj->ATwset->size[0] * obj->ATwset->size[1];
  obj->ATwset->size[0] = nVarMax;
  obj->ATwset->size[1] = mConstrMax;
  emxEnsureCapacity_real_T(obj->ATwset, i);
  i = obj->bwset->size[0];
  obj->bwset->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->bwset, i);
  obj->nActiveConstr = 0;
  i = obj->maxConstrWorkspace->size[0];
  obj->maxConstrWorkspace->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->maxConstrWorkspace, i);
  obj->sizes[0] = mFixed;
  obj->sizes[1] = 0;
  obj->sizes[2] = 0;
  obj->sizes[3] = mLB;
  obj->sizes[4] = mUB;
  obj->sizesPhaseOne[0] = mFixed;
  obj->sizesPhaseOne[1] = 0;
  obj->sizesPhaseOne[2] = 0;
  obj->sizesPhaseOne[3] = mLB + 1;
  obj->sizesPhaseOne[4] = mUB;
  obj->isActiveIdx[0] = 1;
  obj->isActiveIdx[1] = mFixed;
  obj->isActiveIdx[2] = 0;
  obj->isActiveIdx[3] = 0;
  obj->isActiveIdx[4] = mLB;
  obj->isActiveIdx[5] = mUB;
  for (i = 0; i < 5; i++) {
    obj->sizesNormal[i] = obj->sizes[i];
    obj->sizesRegularized[i] = obj->sizes[i];
    obj->sizesRegPhaseOne[i] = obj->sizesPhaseOne[i];
    obj->isActiveIdx[i + 1] += obj->isActiveIdx[i];
  }

  for (i = 0; i < 6; i++) {
    obj->isActiveIdxNormal[i] = obj->isActiveIdx[i];
  }

  obj->isActiveIdxPhaseOne[0] = 1;
  obj->isActiveIdxPhaseOne[1] = mFixed;
  obj->isActiveIdxPhaseOne[2] = 0;
  obj->isActiveIdxPhaseOne[3] = 0;
  obj->isActiveIdxPhaseOne[4] = mLB + 1;
  obj->isActiveIdxPhaseOne[5] = mUB;
  for (i = 0; i < 5; i++) {
    obj->isActiveIdxPhaseOne[i + 1] += obj->isActiveIdxPhaseOne[i];
  }

  for (i = 0; i < 6; i++) {
    obj->isActiveIdxRegularized[i] = obj->isActiveIdx[i];
    obj->isActiveIdxRegPhaseOne[i] = obj->isActiveIdxPhaseOne[i];
  }

  i = obj->isActiveConstr->size[0];
  obj->isActiveConstr->size[0] = mConstrMax;
  emxEnsureCapacity_boolean_T(obj->isActiveConstr, i);
  i = obj->Wid->size[0];
  obj->Wid->size[0] = mConstrMax;
  emxEnsureCapacity_int32_T(obj->Wid, i);
  i = obj->Wlocalidx->size[0];
  obj->Wlocalidx->size[0] = mConstrMax;
  emxEnsureCapacity_int32_T(obj->Wlocalidx, i);
  for (i = 0; i < 5; i++) {
    obj->nWConstr[i] = 0;
  }

  obj->probType = 3;
  obj->SLACK0 = 1.0E-5;
  for (i = 0; i < mLB; i++) {
    obj->indexLB->data[i] = indexLB->data[i];
  }

  for (i = 0; i < mUB; i++) {
    obj->indexUB->data[i] = indexUB->data[i];
  }

  for (i = 0; i < mFixed; i++) {
    obj->indexFixed->data[i] = indexFixed->data[i];
  }
}

/* End of code generation (factoryConstruct2.c) */
