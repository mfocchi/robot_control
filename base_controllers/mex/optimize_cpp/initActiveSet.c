/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * initActiveSet.c
 *
 * Code generation for function 'initActiveSet'
 *
 */

/* Include files */
#include "initActiveSet.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <string.h>

/* Function Definitions */
void initActiveSet(h_struct_T *obj)
{
  int32_T b_i;
  int32_T colOffsetATw;
  int32_T i;
  int32_T idxFillStart;
  int32_T idx_local;
  setProblemType(obj, 3);
  idxFillStart = obj->isActiveIdx[2];
  i = obj->mConstrMax;
  for (colOffsetATw = idxFillStart; colOffsetATw <= i; colOffsetATw++) {
    obj->isActiveConstr->data[colOffsetATw - 1] = false;
  }
  obj->nWConstr[0] = obj->sizes[0];
  obj->nWConstr[1] = 0;
  obj->nWConstr[2] = 0;
  obj->nWConstr[3] = 0;
  obj->nWConstr[4] = 0;
  obj->nActiveConstr = obj->nWConstr[0];
  idxFillStart = obj->sizes[0];
  for (idx_local = 0; idx_local < idxFillStart; idx_local++) {
    int32_T i1;
    obj->Wid->data[idx_local] = 1;
    obj->Wlocalidx->data[idx_local] = idx_local + 1;
    obj->isActiveConstr->data[idx_local] = true;
    colOffsetATw = obj->ldA * idx_local;
    i = obj->indexFixed->data[idx_local];
    for (b_i = 0; b_i <= i - 2; b_i++) {
      obj->ATwset->data[b_i + colOffsetATw] = 0.0;
    }
    obj->ATwset->data[(obj->indexFixed->data[idx_local] + colOffsetATw) - 1] =
        1.0;
    i = obj->indexFixed->data[idx_local] + 1;
    i1 = obj->nVar;
    for (b_i = i; b_i <= i1; b_i++) {
      obj->ATwset->data[(b_i + colOffsetATw) - 1] = 0.0;
    }
    obj->bwset->data[idx_local] =
        obj->ub->data[obj->indexFixed->data[idx_local] - 1];
  }
}

/* End of code generation (initActiveSet.c) */
