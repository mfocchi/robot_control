/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * setProblemType.c
 *
 * Code generation for function 'setProblemType'
 *
 */

/* Include files */
#include "setProblemType.h"
#include "modifyOverheadPhaseOne_.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void setProblemType(j_struct_T *obj, int32_T PROBLEM_TYPE)
{
  int32_T colOffsetATw;
  int32_T i;
  int32_T i1;
  int32_T idxStartIneq;
  int32_T idx_col;
  int32_T idx_lb;
  int32_T idx_row;
  int32_T offsetIneq;
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = obj->nVarOrig;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (colOffsetATw = 0; colOffsetATw < i; colOffsetATw++) {
        obj->isActiveConstr->data[(obj->isActiveIdxNormal[4] + colOffsetATw) - 1]
          = obj->isActiveConstr->data[(obj->isActiveIdx[4] + colOffsetATw) - 1];
      }
    }

    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }

    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
    break;

   case 1:
    obj->nVar = obj->nVarOrig + 1;
    obj->mConstr = obj->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesPhaseOne[i];
    }

    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
    }

    modifyOverheadPhaseOne_(obj);
    break;

   case 2:
    obj->nVar = obj->nVarMax - 1;
    obj->mConstr = obj->mConstrMax - 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }

    if (obj->probType != 4) {
      offsetIneq = obj->nVarOrig + 1;
      i = obj->sizes[0];
      for (idx_col = 0; idx_col < i; idx_col++) {
        colOffsetATw = obj->ldA * idx_col;
        i1 = obj->nVarOrig + 1;
        idx_lb = obj->nVar;
        for (idx_row = i1; idx_row <= idx_lb; idx_row++) {
          obj->ATwset->data[(idx_row + colOffsetATw) - 1] = 0.0;
        }
      }

      idx_lb = obj->nVarOrig;
      i = obj->sizesNormal[3] + 1;
      i1 = obj->sizesRegularized[3];
      for (colOffsetATw = i; colOffsetATw <= i1; colOffsetATw++) {
        idx_lb++;
        obj->indexLB->data[colOffsetATw - 1] = idx_lb;
      }

      if (obj->nWConstr[4] > 0) {
        i = obj->sizesRegularized[4];
        for (colOffsetATw = 0; colOffsetATw < i; colOffsetATw++) {
          obj->isActiveConstr->data[obj->isActiveIdxRegularized[4] +
            colOffsetATw] = obj->isActiveConstr->data[(obj->isActiveIdx[4] +
            colOffsetATw) - 1];
        }
      }

      i = obj->isActiveIdx[4];
      i1 = obj->isActiveIdxRegularized[4] - 1;
      for (colOffsetATw = i; colOffsetATw <= i1; colOffsetATw++) {
        obj->isActiveConstr->data[colOffsetATw - 1] = false;
      }

      i = obj->nVarOrig + 1;
      i1 = obj->nVarOrig;
      for (colOffsetATw = i; colOffsetATw <= i1; colOffsetATw++) {
        obj->lb->data[colOffsetATw - 1] = 0.0;
      }

      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx_col = idxStartIneq; idx_col <= i; idx_col++) {
        colOffsetATw = obj->ldA * (idx_col - 1) - 1;
        switch (obj->Wid->data[idx_col - 1]) {
         case 3:
          idx_lb = obj->Wlocalidx->data[idx_col - 1];
          i1 = offsetIneq + idx_lb;
          idx_lb = i1 - 2;
          for (idx_row = offsetIneq; idx_row <= idx_lb; idx_row++) {
            obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
          }

          obj->ATwset->data[(i1 + colOffsetATw) - 1] = -1.0;
          idx_lb = obj->nVar;
          for (idx_row = i1; idx_row <= idx_lb; idx_row++) {
            obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
          }
          break;

         default:
          i1 = obj->nVar;
          for (idx_row = offsetIneq; idx_row <= i1; idx_row++) {
            obj->ATwset->data[idx_row + colOffsetATw] = 0.0;
          }
          break;
        }
      }
    }

    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
    }
    break;

   default:
    obj->nVar = obj->nVarMax;
    obj->mConstr = obj->mConstrMax;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegPhaseOne[i];
    }

    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
    }

    modifyOverheadPhaseOne_(obj);
    break;
  }

  obj->probType = PROBLEM_TYPE;
}

/* End of code generation (setProblemType.c) */
