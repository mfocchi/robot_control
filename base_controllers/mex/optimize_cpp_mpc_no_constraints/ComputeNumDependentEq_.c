/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * ComputeNumDependentEq_.c
 *
 * Code generation for function 'ComputeNumDependentEq_'
 *
 */

/* Include files */
#include "ComputeNumDependentEq_.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xdot.h"
#include "xgeqp3.h"
#include "xorgqr.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
int32_T ComputeNumDependentEq_(g_struct_T *qrmanager, const emxArray_real_T
  *beqf, int32_T mConstr, int32_T nVar)
{
  real_T tol;
  int32_T iQR0;
  int32_T idx;
  int32_T numDependent;
  int32_T y;
  boolean_T exitg1;
  y = mConstr - nVar;
  numDependent = muIntScalarMax_sint32(0, y);
  for (idx = 0; idx < nVar; idx++) {
    qrmanager->jpvt->data[idx] = 0;
  }

  if (mConstr * nVar == 0) {
    qrmanager->mrows = mConstr;
    qrmanager->ncols = nVar;
    qrmanager->minRowCol = 0;
  } else {
    qrmanager->usedPivoting = true;
    qrmanager->mrows = mConstr;
    qrmanager->ncols = nVar;
    qrmanager->minRowCol = muIntScalarMin_sint32(mConstr, nVar);
    xgeqp3(qrmanager->QR, mConstr, nVar, qrmanager->jpvt, qrmanager->tau);
  }

  tol = 100.0 * (real_T)nVar * 2.2204460492503131E-16;
  idx = muIntScalarMin_sint32(nVar, mConstr);
  while ((idx > 0) && (muDoubleScalarAbs(qrmanager->QR->data[(idx +
            qrmanager->QR->size[0] * (idx - 1)) - 1]) < tol)) {
    idx--;
    numDependent++;
  }

  if (numDependent > 0) {
    y = qrmanager->minRowCol;
    for (idx = 0; idx < y; idx++) {
      iQR0 = (qrmanager->ldq * idx + idx) + 2;
      b_xcopy((qrmanager->mrows - idx) - 1, qrmanager->QR, iQR0, qrmanager->Q,
              iQR0);
    }

    xorgqr(qrmanager->mrows, qrmanager->mrows, qrmanager->minRowCol,
           qrmanager->Q, qrmanager->ldq, qrmanager->tau);
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx <= numDependent - 1)) {
      if (muDoubleScalarAbs(xdot(mConstr, qrmanager->Q, qrmanager->ldq *
            ((mConstr - idx) - 1) + 1, beqf)) >= tol) {
        numDependent = -1;
        exitg1 = true;
      } else {
        idx++;
      }
    }
  }

  return numDependent;
}

/* End of code generation (ComputeNumDependentEq_.c) */
