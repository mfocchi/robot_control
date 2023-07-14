/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * setProblemType.c
 *
 * Code generation for function 'setProblemType'
 *
 */

/* Include files */
#include "setProblemType.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void setProblemType(h_struct_T *obj, int32_T PROBLEM_TYPE)
{
  int32_T i;
  int32_T idx;
  int32_T idx_col;
  int32_T idx_row;
  switch (PROBLEM_TYPE) {
  case 3:
    obj->nVar = obj->nVarOrig;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (idx = 0; idx < i; idx++) {
        obj->isActiveConstr->data[(obj->isActiveIdxNormal[4] + idx) - 1] =
            obj->isActiveConstr->data[(obj->isActiveIdx[4] + idx) - 1];
      }
    }
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
    break;
  case 1: {
    int32_T idxStartIneq;
    obj->nVar = obj->nVarOrig + 1;
    obj->mConstr = obj->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
    }
    i = obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      obj->ATwset->data[(obj->nVar + obj->ldA * idx) - 1] = 0.0;
    }
    obj->indexLB->data[obj->sizes[3] - 1] = obj->nVar;
    obj->lb->data[obj->nVar - 1] = 1.0E-5;
    idxStartIneq = obj->isActiveIdx[2];
    i = obj->nActiveConstr;
    for (idx = idxStartIneq; idx <= i; idx++) {
      obj->ATwset->data[(obj->nVar + obj->ldA * (idx - 1)) - 1] = -1.0;
    }
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (idx = 0; idx <= i; idx++) {
        obj->isActiveConstr->data[(obj->isActiveIdx[4] + idx) - 1] = false;
      }
    }
    obj->isActiveConstr->data[obj->isActiveIdx[4] - 2] = false;
  } break;
  case 2: {
    obj->nVar = obj->nVarMax - 1;
    obj->mConstr = obj->mConstrMax - 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }
    if (obj->probType != 4) {
      int32_T colOffsetATw;
      int32_T i1;
      int32_T idxStartIneq;
      int32_T offsetIneq_tmp;
      offsetIneq_tmp = obj->nVarOrig + 1;
      i = obj->sizes[0];
      for (idx_col = 0; idx_col < i; idx_col++) {
        colOffsetATw = obj->ldA * idx_col;
        i1 = obj->nVar;
        for (idx_row = offsetIneq_tmp; idx_row <= i1; idx_row++) {
          obj->ATwset->data[(idx_row + colOffsetATw) - 1] = 0.0;
        }
      }
      colOffsetATw = obj->nVarOrig;
      i = obj->sizesNormal[3] + 1;
      i1 = obj->sizesRegularized[3];
      for (idx = i; idx <= i1; idx++) {
        colOffsetATw++;
        obj->indexLB->data[idx - 1] = colOffsetATw;
      }
      if (obj->nWConstr[4] > 0) {
        i = obj->sizesRegularized[4];
        for (idx = 0; idx < i; idx++) {
          obj->isActiveConstr->data[obj->isActiveIdxRegularized[4] + idx] =
              obj->isActiveConstr->data[(obj->isActiveIdx[4] + idx) - 1];
        }
      }
      i = obj->isActiveIdx[4];
      i1 = obj->isActiveIdxRegularized[4] - 1;
      for (idx = i; idx <= i1; idx++) {
        obj->isActiveConstr->data[idx - 1] = false;
      }
      i = obj->nVarOrig;
      for (idx = offsetIneq_tmp; idx <= i; idx++) {
        obj->lb->data[idx - 1] = 0.0;
      }
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx_col = idxStartIneq; idx_col <= i; idx_col++) {
        colOffsetATw = obj->ldA * (idx_col - 1) - 1;
        if (obj->Wid->data[idx_col - 1] == 3) {
          i1 = offsetIneq_tmp + obj->Wlocalidx->data[idx_col - 1];
          idx = i1 - 2;
          for (idx_row = offsetIneq_tmp; idx_row <= idx; idx_row++) {
            obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
          }
          obj->ATwset->data[(i1 + colOffsetATw) - 1] = -1.0;
          idx = obj->nVar;
          for (idx_row = i1; idx_row <= idx; idx_row++) {
            obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
          }
        } else {
          i1 = obj->nVar;
          for (idx_row = offsetIneq_tmp; idx_row <= i1; idx_row++) {
            obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
          }
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
    }
  } break;
  default: {
    int32_T idxStartIneq;
    obj->nVar = obj->nVarMax;
    obj->mConstr = obj->mConstrMax;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
    }
    i = obj->sizes[0];
    for (idx = 0; idx < i; idx++) {
      obj->ATwset->data[(obj->nVar + obj->ldA * idx) - 1] = 0.0;
    }
    obj->indexLB->data[obj->sizes[3] - 1] = obj->nVar;
    obj->lb->data[obj->nVar - 1] = 1.0E-5;
    idxStartIneq = obj->isActiveIdx[2];
    i = obj->nActiveConstr;
    for (idx = idxStartIneq; idx <= i; idx++) {
      obj->ATwset->data[(obj->nVar + obj->ldA * (idx - 1)) - 1] = -1.0;
    }
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (idx = 0; idx <= i; idx++) {
        obj->isActiveConstr->data[(obj->isActiveIdx[4] + idx) - 1] = false;
      }
    }
    obj->isActiveConstr->data[obj->isActiveIdx[4] - 2] = false;
  } break;
  }
  obj->probType = PROBLEM_TYPE;
}

/* End of code generation (setProblemType.c) */
