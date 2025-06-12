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
#include "optimize_cpp_mpc_propellers_data.h"
#include "optimize_cpp_mpc_propellers_emxutil.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void c_factoryConstruct(int32_T mIneq, int32_T mLB, const emxArray_int32_T
  *indexLB, int32_T mUB, const emxArray_int32_T *indexUB, int32_T mFixed, const
  emxArray_int32_T *indexFixed, int32_T nVar, int32_T nVarMax, int32_T
  mConstrMax, j_struct_T *obj)
{
  emxArray_real_T *r;
  int32_T obj_tmp;
  int32_T obj_tmp_tmp;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  obj_tmp_tmp = mIneq + mLB;
  obj_tmp = (obj_tmp_tmp + mUB) + mFixed;
  obj->mConstr = obj_tmp;
  obj->mConstrOrig = obj_tmp;
  obj->mConstrMax = mConstrMax;
  obj->nVar = nVar;
  obj->nVarOrig = nVar;
  obj->nVarMax = nVarMax;
  obj->ldA = nVarMax;
  if (mIneq > 0) {
    emxInit_real_T(&r, 1, true);
    obj_tmp = obj->Aineq->size[0] * obj->Aineq->size[1];
    obj->Aineq->size[0] = nVarMax;
    obj->Aineq->size[1] = mIneq;
    emxEnsureCapacity_real_T(obj->Aineq, obj_tmp);
    obj_tmp = r->size[0];
    r->size[0] = mIneq;
    emxEnsureCapacity_real_T(r, obj_tmp);
    for (obj_tmp = 0; obj_tmp < mIneq; obj_tmp++) {
      r->data[obj_tmp] = 1.7976931348623157E+308;
    }

    obj_tmp = obj->bineq->size[0] * obj->bineq->size[1];
    obj->bineq->size[0] = r->size[0];
    obj->bineq->size[1] = 1;
    emxEnsureCapacity_real_T(obj->bineq, obj_tmp);
    emxFree_real_T(&r);
  } else {
    obj->Aineq->size[0] = 0;
    obj->Aineq->size[1] = 0;
    obj->bineq->size[0] = 0;
    obj->bineq->size[1] = 0;
  }

  obj_tmp = obj->lb->size[0];
  obj->lb->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->lb, obj_tmp);
  obj_tmp = obj->ub->size[0];
  obj->ub->size[0] = nVarMax;
  emxEnsureCapacity_real_T(obj->ub, obj_tmp);
  obj_tmp = obj->indexLB->size[0];
  obj->indexLB->size[0] = nVarMax;
  emxEnsureCapacity_int32_T(obj->indexLB, obj_tmp);
  obj_tmp = obj->indexUB->size[0];
  obj->indexUB->size[0] = nVarMax;
  emxEnsureCapacity_int32_T(obj->indexUB, obj_tmp);
  obj_tmp = obj->indexFixed->size[0];
  obj->indexFixed->size[0] = nVarMax;
  emxEnsureCapacity_int32_T(obj->indexFixed, obj_tmp);
  obj->mEqRemoved = 0;
  obj_tmp = obj->ATwset->size[0] * obj->ATwset->size[1];
  obj->ATwset->size[0] = nVarMax;
  obj->ATwset->size[1] = mConstrMax;
  emxEnsureCapacity_real_T(obj->ATwset, obj_tmp);
  obj_tmp = obj->bwset->size[0];
  obj->bwset->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->bwset, obj_tmp);
  obj->nActiveConstr = 0;
  obj_tmp = obj->maxConstrWorkspace->size[0];
  obj->maxConstrWorkspace->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->maxConstrWorkspace, obj_tmp);
  obj->sizes[0] = mFixed;
  obj->sizes[1] = 0;
  obj->sizes[2] = mIneq;
  obj->sizes[3] = mLB;
  obj->sizes[4] = mUB;
  obj->sizesPhaseOne[0] = mFixed;
  obj->sizesPhaseOne[1] = 0;
  obj->sizesPhaseOne[2] = mIneq;
  obj->sizesPhaseOne[3] = mLB + 1;
  obj->sizesPhaseOne[4] = mUB;
  obj->sizesRegularized[0] = mFixed;
  obj->sizesRegularized[1] = 0;
  obj->sizesRegularized[2] = mIneq;
  obj->sizesRegularized[3] = obj_tmp_tmp;
  obj->sizesRegularized[4] = mUB;
  obj->sizesRegPhaseOne[0] = mFixed;
  obj->sizesRegPhaseOne[1] = 0;
  obj->sizesRegPhaseOne[2] = mIneq;
  obj->sizesRegPhaseOne[3] = obj_tmp_tmp + 1;
  obj->sizesRegPhaseOne[4] = mUB;
  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 0;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = mLB;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (obj_tmp = 0; obj_tmp < 5; obj_tmp++) {
    obj->sizesNormal[obj_tmp] = obj->sizes[obj_tmp];
    obj->isActiveIdxRegPhaseOne[obj_tmp + 1] += obj->
      isActiveIdxRegPhaseOne[obj_tmp];
  }

  for (obj_tmp = 0; obj_tmp < 6; obj_tmp++) {
    obj->isActiveIdx[obj_tmp] = obj->isActiveIdxRegPhaseOne[obj_tmp];
    obj->isActiveIdxNormal[obj_tmp] = obj->isActiveIdxRegPhaseOne[obj_tmp];
  }

  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 0;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = mLB + 1;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (obj_tmp = 0; obj_tmp < 5; obj_tmp++) {
    obj->isActiveIdxRegPhaseOne[obj_tmp + 1] += obj->
      isActiveIdxRegPhaseOne[obj_tmp];
  }

  for (obj_tmp = 0; obj_tmp < 6; obj_tmp++) {
    obj->isActiveIdxPhaseOne[obj_tmp] = obj->isActiveIdxRegPhaseOne[obj_tmp];
  }

  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 0;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = obj_tmp_tmp;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  for (obj_tmp = 0; obj_tmp < 5; obj_tmp++) {
    obj->isActiveIdxRegPhaseOne[obj_tmp + 1] += obj->
      isActiveIdxRegPhaseOne[obj_tmp];
  }

  for (obj_tmp = 0; obj_tmp < 6; obj_tmp++) {
    obj->isActiveIdxRegularized[obj_tmp] = obj->isActiveIdxRegPhaseOne[obj_tmp];
  }

  obj->isActiveIdxRegPhaseOne[0] = 1;
  obj->isActiveIdxRegPhaseOne[1] = mFixed;
  obj->isActiveIdxRegPhaseOne[2] = 0;
  obj->isActiveIdxRegPhaseOne[3] = mIneq;
  obj->isActiveIdxRegPhaseOne[4] = (mLB + mIneq) + 1;
  obj->isActiveIdxRegPhaseOne[5] = mUB;
  obj_tmp = obj->isActiveConstr->size[0];
  obj->isActiveConstr->size[0] = mConstrMax;
  emxEnsureCapacity_boolean_T(obj->isActiveConstr, obj_tmp);
  obj_tmp = obj->Wid->size[0];
  obj->Wid->size[0] = mConstrMax;
  emxEnsureCapacity_int32_T(obj->Wid, obj_tmp);
  obj_tmp = obj->Wlocalidx->size[0];
  obj->Wlocalidx->size[0] = mConstrMax;
  emxEnsureCapacity_int32_T(obj->Wlocalidx, obj_tmp);
  for (obj_tmp = 0; obj_tmp < 5; obj_tmp++) {
    obj->isActiveIdxRegPhaseOne[obj_tmp + 1] += obj->
      isActiveIdxRegPhaseOne[obj_tmp];
    obj->nWConstr[obj_tmp] = 0;
  }

  obj->probType = 3;
  obj->SLACK0 = 1.0E-5;
  for (obj_tmp = 0; obj_tmp < mLB; obj_tmp++) {
    obj->indexLB->data[obj_tmp] = indexLB->data[obj_tmp];
  }

  for (obj_tmp = 0; obj_tmp < mUB; obj_tmp++) {
    obj->indexUB->data[obj_tmp] = indexUB->data[obj_tmp];
  }

  for (obj_tmp = 0; obj_tmp < mFixed; obj_tmp++) {
    obj->indexFixed->data[obj_tmp] = indexFixed->data[obj_tmp];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (factoryConstruct2.c) */
