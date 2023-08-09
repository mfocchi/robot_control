/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * saveJacobian.c
 *
 * Code generation for function 'saveJacobian'
 *
 */

/* Include files */
#include "saveJacobian.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include <string.h>

/* Function Definitions */
void saveJacobian(d_struct_T *obj, int32_T nVar, int32_T mIneq, const
                  emxArray_real_T *JacCineqTrans, int32_T ineqCol0, int32_T ldJ)
{
  int32_T i;
  int32_T iCol;
  int32_T iCol_old;
  int32_T idx_col;
  iCol = ldJ * (ineqCol0 - 1) + 1;
  iCol_old = 1;
  i = mIneq - ineqCol0;
  for (idx_col = 0; idx_col <= i; idx_col++) {
    b_xcopy(nVar, JacCineqTrans, iCol, obj->JacCineqTrans_old, iCol_old);
    iCol += ldJ;
    iCol_old += ldJ;
  }
}

/* End of code generation (saveJacobian.c) */
