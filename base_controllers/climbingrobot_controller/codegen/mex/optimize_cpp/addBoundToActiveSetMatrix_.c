/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * addBoundToActiveSetMatrix_.c
 *
 * Code generation for function 'addBoundToActiveSetMatrix_'
 *
 */

/* Include files */
#include "addBoundToActiveSetMatrix_.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void addBoundToActiveSetMatrix_(h_struct_T *obj, int32_T TYPE,
                                int32_T idx_local)
{
  int32_T colOffset;
  int32_T i;
  int32_T idx;
  int32_T idx_bnd_local;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr->data[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] =
      true;
  obj->nActiveConstr++;
  i = obj->nActiveConstr - 1;
  obj->Wid->data[i] = TYPE;
  obj->Wlocalidx->data[i] = idx_local;
  colOffset = obj->ldA * i - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj->indexUB->data[idx_local - 1];
    obj->bwset->data[i] = obj->ub->data[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj->indexLB->data[idx_local - 1];
    obj->bwset->data[i] = obj->lb->data[idx_bnd_local - 1];
  }
  for (idx = 0; idx <= idx_bnd_local - 2; idx++) {
    obj->ATwset->data[(idx + colOffset) + 1] = 0.0;
  }
  obj->ATwset->data[idx_bnd_local + colOffset] =
      2.0 * (real_T)(TYPE == 5) - 1.0;
  i = idx_bnd_local + 1;
  idx_bnd_local = obj->nVar;
  for (idx = i; idx <= idx_bnd_local; idx++) {
    obj->ATwset->data[idx + colOffset] = 0.0;
  }
  switch (obj->probType) {
  case 3:
  case 2:
    break;
  default:
    obj->ATwset->data[obj->nVar + colOffset] = -1.0;
    break;
  }
}

/* End of code generation (addBoundToActiveSetMatrix_.c) */
