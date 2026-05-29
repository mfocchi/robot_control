/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * addAineqConstr.c
 *
 * Code generation for function 'addAineqConstr'
 *
 */

/* Include files */
#include "addAineqConstr.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void addAineqConstr(h_struct_T *obj, int32_T idx_local)
{
  int32_T i;
  int32_T i1;
  int32_T iAineq0;
  int32_T iAw0;
  int32_T idx;
  obj->nWConstr[2]++;
  obj->isActiveConstr->data[(obj->isActiveIdx[2] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  i = obj->nActiveConstr - 1;
  obj->Wid->data[i] = 3;
  obj->Wlocalidx->data[i] = idx_local;
  iAineq0 = obj->ldA * (idx_local - 1);
  iAw0 = obj->ldA * i;
  i1 = obj->nVar - 1;
  for (idx = 0; idx <= i1; idx++) {
    obj->ATwset->data[iAw0 + idx] = obj->Aineq->data[iAineq0 + idx];
  }
  obj->bwset->data[i] = obj->bineq->data[idx_local - 1];
}

/* End of code generation (addAineqConstr.c) */
