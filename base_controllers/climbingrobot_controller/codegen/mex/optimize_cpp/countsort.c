/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * countsort.c
 *
 * Code generation for function 'countsort'
 *
 */

/* Include files */
#include "countsort.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void countsort(emxArray_int32_T *x, int32_T xLen, emxArray_int32_T *workspace,
               int32_T xMin, int32_T xMax)
{
  int32_T idx;
  int32_T idxFill;
  int32_T *workspace_data;
  int32_T *x_data;
  workspace_data = workspace->data;
  x_data = x->data;
  if ((xLen > 1) && (xMax > xMin)) {
    int32_T idxEnd;
    int32_T idxStart;
    int32_T maxOffset;
    idxStart = xMax - xMin;
    for (idx = 0; idx <= idxStart; idx++) {
      workspace_data[idx] = 0;
    }
    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x_data[idx] - xMin;
      workspace_data[idxStart]++;
    }
    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace_data[idx - 1] += workspace_data[idx - 2];
    }
    idxStart = 1;
    idxEnd = workspace_data[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x_data[idxFill - 1] = idx + xMin;
      }
      idxStart = workspace_data[idx] + 1;
      idxEnd = workspace_data[idx + 1];
    }
    for (idx = idxStart; idx <= idxEnd; idx++) {
      x_data[idx - 1] = xMax;
    }
  }
}

/* End of code generation (countsort.c) */
