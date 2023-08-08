/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * removeConstr.c
 *
 * Code generation for function 'removeConstr'
 *
 */

/* Include files */
#include "removeConstr.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void removeConstr(j_struct_T *obj, int32_T idx_global)
{
  int32_T TYPE_tmp;
  int32_T i;
  int32_T idx;
  TYPE_tmp = obj->Wid->data[idx_global - 1] - 1;
  obj->isActiveConstr->data[(obj->isActiveIdx[TYPE_tmp] + obj->Wlocalidx->
    data[idx_global - 1]) - 2] = false;
  obj->Wid->data[idx_global - 1] = obj->Wid->data[obj->nActiveConstr - 1];
  obj->Wlocalidx->data[idx_global - 1] = obj->Wlocalidx->data[obj->nActiveConstr
    - 1];
  i = obj->nVar;
  for (idx = 0; idx < i; idx++) {
    obj->ATwset->data[idx + obj->ATwset->size[0] * (idx_global - 1)] =
      obj->ATwset->data[idx + obj->ATwset->size[0] * (obj->nActiveConstr - 1)];
  }

  obj->bwset->data[idx_global - 1] = obj->bwset->data[obj->nActiveConstr - 1];
  obj->nActiveConstr--;
  obj->nWConstr[TYPE_tmp]--;
}

/* End of code generation (removeConstr.c) */
