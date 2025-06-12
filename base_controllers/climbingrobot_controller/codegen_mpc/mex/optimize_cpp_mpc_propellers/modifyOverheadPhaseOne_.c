/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * modifyOverheadPhaseOne_.c
 *
 * Code generation for function 'modifyOverheadPhaseOne_'
 *
 */

/* Include files */
#include "modifyOverheadPhaseOne_.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void modifyOverheadPhaseOne_(j_struct_T *obj)
{
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T idxStartIneq;
  i = obj->sizes[0];
  for (idx = 0; idx < i; idx++) {
    obj->ATwset->data[(obj->nVar + obj->ldA * idx) - 1] = 0.0;
  }

  i = obj->sizes[2];
  for (idx = 0; idx < i; idx++) {
    obj->Aineq->data[(obj->nVar + obj->ldA * idx) - 1] = -1.0;
  }

  obj->indexLB->data[obj->sizes[3] - 1] = obj->nVar;
  obj->lb->data[obj->nVar - 1] = 1.0E-5;
  idxStartIneq = obj->isActiveIdx[2];
  i = obj->nActiveConstr;
  for (idx = idxStartIneq; idx <= i; idx++) {
    obj->ATwset->data[(obj->nVar + obj->ldA * (idx - 1)) - 1] = -1.0;
  }

  idxStartIneq = obj->isActiveIdx[4];
  if (obj->nWConstr[4] > 0) {
    i = obj->sizesNormal[4];
    for (idx = 0; idx < i; idx++) {
      i1 = idxStartIneq + idx;
      obj->isActiveConstr->data[i1] = obj->isActiveConstr->data[i1 - 1];
    }
  }

  obj->isActiveConstr->data[obj->isActiveIdx[4] - 2] = false;
}

/* End of code generation (modifyOverheadPhaseOne_.c) */
