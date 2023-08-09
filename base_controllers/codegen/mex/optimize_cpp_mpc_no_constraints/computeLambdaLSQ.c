/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * computeLambdaLSQ.c
 *
 * Code generation for function 'computeLambdaLSQ'
 *
 */

/* Include files */
#include "computeLambdaLSQ.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgemv.h"
#include "xgeqp3.h"
#include "xorgqr.h"
#include "xtrsv.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void computeLambdaLSQ(int32_T nVar, int32_T mConstr, g_struct_T *QRManager,
                      const emxArray_real_T *ATwset, const emxArray_real_T *grad,
                      emxArray_real_T *lambdaLSQ, emxArray_real_T *workspace)
{
  real_T tol;
  int32_T fullRank_R;
  int32_T iQR0;
  int32_T iQR_diag;
  boolean_T guard1 = false;
  for (iQR0 = 0; iQR0 < mConstr; iQR0++) {
    lambdaLSQ->data[iQR0] = 0.0;
  }

  fullRank_R = nVar * mConstr;
  guard1 = false;
  if (fullRank_R > 0) {
    for (iQR_diag = 0; iQR_diag < mConstr; iQR_diag++) {
      b_xcopy(nVar, ATwset, ATwset->size[0] * iQR_diag + 1, QRManager->QR,
              QRManager->ldq * iQR_diag + 1);
    }

    guard1 = true;
  } else if (fullRank_R == 0) {
    QRManager->mrows = nVar;
    QRManager->ncols = mConstr;
    QRManager->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    QRManager->usedPivoting = true;
    QRManager->mrows = nVar;
    QRManager->ncols = mConstr;
    QRManager->minRowCol = muIntScalarMin_sint32(nVar, mConstr);
    xgeqp3(QRManager->QR, nVar, mConstr, QRManager->jpvt, QRManager->tau);
  }

  fullRank_R = QRManager->minRowCol;
  for (iQR_diag = 0; iQR_diag < fullRank_R; iQR_diag++) {
    iQR0 = (QRManager->ldq * iQR_diag + iQR_diag) + 2;
    b_xcopy((QRManager->mrows - iQR_diag) - 1, QRManager->QR, iQR0, QRManager->Q,
            iQR0);
  }

  xorgqr(QRManager->mrows, QRManager->mrows, QRManager->minRowCol, QRManager->Q,
         QRManager->ldq, QRManager->tau);
  xgemv(nVar, nVar, QRManager->Q, QRManager->ldq, grad, workspace);
  tol = muDoubleScalarAbs(QRManager->QR->data[0]) * muDoubleScalarMin
    (1.4901161193847656E-8, (real_T)muIntScalarMax_sint32(nVar, mConstr) *
     2.2204460492503131E-16);
  fullRank_R = muIntScalarMin_sint32(nVar, mConstr);
  iQR0 = 0;
  iQR_diag = 0;
  while ((iQR0 < fullRank_R) && (muDoubleScalarAbs(QRManager->QR->data[iQR_diag])
          > tol)) {
    iQR0++;
    iQR_diag = (iQR_diag + QRManager->ldq) + 1;
  }

  xtrsv(iQR0, QRManager->QR, QRManager->ldq, workspace);
  fullRank_R = muIntScalarMin_sint32(mConstr, fullRank_R);
  for (iQR_diag = 0; iQR_diag < fullRank_R; iQR_diag++) {
    lambdaLSQ->data[QRManager->jpvt->data[iQR_diag] - 1] = workspace->
      data[iQR_diag];
  }
}

/* End of code generation (computeLambdaLSQ.c) */
