/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * factorQR.c
 *
 * Code generation for function 'factorQR'
 *
 */

/* Include files */
#include "factorQR.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgeqrf.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void factorQR(g_struct_T *obj, const emxArray_real_T *A, int32_T mrows, int32_T
              ncols)
{
  int32_T idx;
  boolean_T guard1 = false;
  idx = mrows * ncols;
  guard1 = false;
  if (idx > 0) {
    for (idx = 0; idx < ncols; idx++) {
      b_xcopy(mrows, A, A->size[0] * idx + 1, obj->QR, obj->ldq * idx + 1);
    }

    guard1 = true;
  } else if (idx == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    for (idx = 0; idx < ncols; idx++) {
      obj->jpvt->data[idx] = idx + 1;
    }

    obj->minRowCol = muIntScalarMin_sint32(mrows, ncols);
    xgeqrf(obj->QR, mrows, ncols, obj->tau);
  }
}

/* End of code generation (factorQR.c) */
