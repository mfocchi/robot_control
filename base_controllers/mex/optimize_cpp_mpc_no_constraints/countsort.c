/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * countsort.c
 *
 * Code generation for function 'countsort'
 *
 */

/* Include files */
#include "countsort.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void countsort(emxArray_int32_T *x, int32_T xLen, emxArray_int32_T *workspace,
               int32_T xMin, int32_T xMax)
{
  int32_T idx;
  int32_T idxEnd;
  int32_T idxFill;
  int32_T idxStart;
  int32_T maxOffset;
  if ((xLen > 1) && (xMax > xMin)) {
    idxStart = xMax - xMin;
    for (idx = 0; idx <= idxStart; idx++) {
      workspace->data[idx] = 0;
    }

    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x->data[idx] - xMin;
      workspace->data[idxStart]++;
    }

    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace->data[idx - 1] += workspace->data[idx - 2];
    }

    idxStart = 1;
    idxEnd = workspace->data[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x->data[idxFill - 1] = idx + xMin;
      }

      idxStart = workspace->data[idx] + 1;
      idxEnd = workspace->data[idx + 1];
    }

    for (idx = idxStart; idx <= idxEnd; idx++) {
      x->data[idx - 1] = xMax;
    }
  }
}

/* End of code generation (countsort.c) */
