/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * removeConstr.c
 *
 * Code generation for function 'removeConstr'
 *
 */

/* Include files */
#include "removeConstr.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void removeConstr(h_struct_T *obj, int32_T idx_global)
{
  int32_T TYPE_tmp;
  int32_T i;
  int32_T i1;
  int32_T idx;
  TYPE_tmp = obj->Wid->data[idx_global - 1] - 1;
  obj->isActiveConstr->data[(obj->isActiveIdx[TYPE_tmp] +
                             obj->Wlocalidx->data[idx_global - 1]) -
                            2] = false;
  i = obj->nActiveConstr - 1;
  obj->Wid->data[idx_global - 1] = obj->Wid->data[i];
  obj->Wlocalidx->data[idx_global - 1] = obj->Wlocalidx->data[i];
  i1 = obj->nVar;
  for (idx = 0; idx < i1; idx++) {
    obj->ATwset->data[idx + obj->ldA * (idx_global - 1)] =
        obj->ATwset->data[idx + obj->ldA * i];
  }
  obj->bwset->data[idx_global - 1] = obj->bwset->data[i];
  obj->nActiveConstr = i;
  obj->nWConstr[TYPE_tmp]--;
}

/* End of code generation (removeConstr.c) */
