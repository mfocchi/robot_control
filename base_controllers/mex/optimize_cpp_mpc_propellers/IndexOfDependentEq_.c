/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * IndexOfDependentEq_.c
 *
 * Code generation for function 'IndexOfDependentEq_'
 *
 */

/* Include files */
#include "IndexOfDependentEq_.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgeqp3.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void IndexOfDependentEq_(emxArray_int32_T *depIdx, int32_T mFixed, int32_T nDep,
  g_struct_T *qrmanager, const emxArray_real_T *AeqfPrime, int32_T mRows,
  int32_T nCols)
{
  int32_T i;
  int32_T idx;
  boolean_T guard1 = false;
  for (idx = 0; idx < mFixed; idx++) {
    qrmanager->jpvt->data[idx] = 1;
  }

  i = mFixed + 1;
  for (idx = i; idx <= nCols; idx++) {
    qrmanager->jpvt->data[idx - 1] = 0;
  }

  i = mRows * nCols;
  guard1 = false;
  if (i > 0) {
    for (idx = 0; idx < nCols; idx++) {
      b_xcopy(mRows, AeqfPrime, AeqfPrime->size[0] * idx + 1, qrmanager->QR,
              qrmanager->ldq * idx + 1);
    }

    guard1 = true;
  } else if (i == 0) {
    qrmanager->mrows = mRows;
    qrmanager->ncols = nCols;
    qrmanager->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    qrmanager->usedPivoting = true;
    qrmanager->mrows = mRows;
    qrmanager->ncols = nCols;
    qrmanager->minRowCol = muIntScalarMin_sint32(mRows, nCols);
    xgeqp3(qrmanager->QR, mRows, nCols, qrmanager->jpvt, qrmanager->tau);
  }

  for (idx = 0; idx < nDep; idx++) {
    depIdx->data[idx] = qrmanager->jpvt->data[(nCols - nDep) + idx];
  }
}

/* End of code generation (IndexOfDependentEq_.c) */
