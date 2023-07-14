/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct2.c
 *
 * Code generation for function 'factoryConstruct2'
 *
 */

/* Include files */
#include "factoryConstruct2.h"
#include "optimize_cpp_mpc_emxutil.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void c_factoryConstruct(int32_T nVar, int32_T nVarMax, int32_T mConstrMax,
                        h_struct_T *obj)
{
  int32_T i;
  obj->mConstr = 0;
  obj->mConstrOrig = 0;
  obj->mConstrMax = mConstrMax;
  obj->nVar = nVar;
  obj->nVarOrig = nVar;
  obj->nVarMax = nVarMax;
  obj->ldA = nVarMax;
  obj->Aineq.size[0] = 0;
  obj->Aeq.size[0] = 0;
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
  i = obj->ATwset->size[0];
  obj->ATwset->size[0] = nVarMax * mConstrMax;
  emxEnsureCapacity_real_T(obj->ATwset, i);
  i = obj->bwset->size[0];
  obj->bwset->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->bwset, i);
  obj->nActiveConstr = 0;
  i = obj->maxConstrWorkspace->size[0];
  obj->maxConstrWorkspace->size[0] = mConstrMax;
  emxEnsureCapacity_real_T(obj->maxConstrWorkspace, i);
  for (i = 0; i < 5; i++) {
    obj->sizes[i] = 0;
    obj->sizesNormal[i] = 0;
    obj->sizesPhaseOne[i] = 0;
    obj->sizesRegularized[i] = 0;
    obj->sizesRegPhaseOne[i] = 0;
  }
  for (i = 0; i < 6; i++) {
    obj->isActiveIdx[i] = 0;
    obj->isActiveIdxNormal[i] = 0;
    obj->isActiveIdxPhaseOne[i] = 0;
    obj->isActiveIdxRegularized[i] = 0;
    obj->isActiveIdxRegPhaseOne[i] = 0;
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
}

/* End of code generation (factoryConstruct2.c) */
