/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * sortLambdaQP.c
 *
 * Code generation for function 'sortLambdaQP'
 *
 */

/* Include files */
#include "sortLambdaQP.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void sortLambdaQP(emxArray_real_T *lambda, int32_T WorkingSet_nActiveConstr,
                  const int32_T WorkingSet_sizes[5], const int32_T
                  WorkingSet_isActiveIdx[6], const emxArray_int32_T
                  *WorkingSet_Wid, const emxArray_int32_T *WorkingSet_Wlocalidx,
                  emxArray_real_T *workspace)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  int32_T idx;
  int32_T idxOffset;
  int32_T mAll;
  if (WorkingSet_nActiveConstr != 0) {
    mAll = (WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4];
    if (mAll >= 1) {
      n_t = (ptrdiff_t)mAll;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dcopy(&n_t, &lambda->data[0], &incx_t, &workspace->data[0], &incy_t);
    }

    for (idx = 0; idx < mAll; idx++) {
      lambda->data[idx] = 0.0;
    }

    mAll = 0;
    idx = 0;
    while ((idx + 1 <= WorkingSet_nActiveConstr) && (WorkingSet_Wid->data[idx] <=
            2)) {
      switch (WorkingSet_Wid->data[idx]) {
       case 1:
        idxOffset = 1;
        break;

       default:
        idxOffset = WorkingSet_isActiveIdx[1];
        break;
      }

      lambda->data[(idxOffset + WorkingSet_Wlocalidx->data[idx]) - 2] =
        workspace->data[mAll];
      mAll++;
      idx++;
    }

    while (idx + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid->data[idx]) {
       case 3:
        idxOffset = WorkingSet_isActiveIdx[2];
        break;

       case 4:
        idxOffset = WorkingSet_isActiveIdx[3];
        break;

       default:
        idxOffset = WorkingSet_isActiveIdx[4];
        break;
      }

      lambda->data[(idxOffset + WorkingSet_Wlocalidx->data[idx]) - 2] =
        workspace->data[mAll];
      mAll++;
      idx++;
    }
  }
}

/* End of code generation (sortLambdaQP.c) */
