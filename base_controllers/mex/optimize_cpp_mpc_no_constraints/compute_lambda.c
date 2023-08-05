/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * compute_lambda.c
 *
 * Code generation for function 'compute_lambda'
 *
 */

/* Include files */
#include "compute_lambda.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xtrsv.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
void compute_lambda(emxArray_real_T *workspace, d_struct_T *solution, const
                    i_struct_T *objective, const g_struct_T *qrmanager)
{
  real_T tol;
  int32_T idx;
  int32_T nActiveConstr;
  boolean_T guard1 = false;
  boolean_T nonDegenerate;
  nActiveConstr = qrmanager->ncols;
  if (qrmanager->ncols > 0) {
    tol = 100.0 * (real_T)qrmanager->mrows * 2.2204460492503131E-16;
    if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
      nonDegenerate = true;
    } else {
      nonDegenerate = false;
    }

    if (nonDegenerate) {
      idx = qrmanager->ncols;
      guard1 = false;
      if (qrmanager->mrows < qrmanager->ncols) {
        while ((idx > qrmanager->mrows) && (muDoubleScalarAbs(qrmanager->
                 QR->data[(qrmanager->mrows + qrmanager->QR->size[0] * (idx - 1))
                 - 1]) >= tol)) {
          idx--;
        }

        nonDegenerate = (idx == qrmanager->mrows);
        if (nonDegenerate) {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        while ((idx >= 1) && (muDoubleScalarAbs(qrmanager->QR->data[(idx +
                  qrmanager->QR->size[0] * (idx - 1)) - 1]) >= tol)) {
          idx--;
        }

        nonDegenerate = (idx == 0);
      }
    }

    if (!nonDegenerate) {
      solution->state = -7;
    } else {
      xgemv(qrmanager->mrows, qrmanager->ncols, qrmanager->Q, qrmanager->ldq,
            objective->grad, workspace);
      xtrsv(qrmanager->ncols, qrmanager->QR, qrmanager->ldq, workspace);
      for (idx = 0; idx < nActiveConstr; idx++) {
        solution->lambda->data[idx] = -workspace->data[idx];
      }
    }
  }
}

/* End of code generation (compute_lambda.c) */
