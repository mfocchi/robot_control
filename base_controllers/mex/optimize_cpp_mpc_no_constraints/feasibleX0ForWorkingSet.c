/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * feasibleX0ForWorkingSet.c
 *
 * Code generation for function 'feasibleX0ForWorkingSet'
 *
 */

/* Include files */
#include "feasibleX0ForWorkingSet.h"
#include "factorQR.h"
#include "maxConstraintViolation.h"
#include "optimize_cpp_mpc_no_constraints_data.h"
#include "optimize_cpp_mpc_no_constraints_emxutil.h"
#include "optimize_cpp_mpc_no_constraints_types.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgemm.h"
#include "xgemv.h"
#include "xgeqrf.h"
#include "xorgqr.h"
#include "xtrsm.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
boolean_T feasibleX0ForWorkingSet(emxArray_real_T *workspace, emxArray_real_T
  *xCurrent, const j_struct_T *workingset, g_struct_T *qrmanager)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emxArray_real_T *b_workspace;
  real_T a;
  real_T constrViolation_basicX;
  int32_T exitg1;
  int32_T idx;
  int32_T idx_row;
  int32_T mWConstr;
  int32_T nVar;
  boolean_T nonDegenerateWset;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  mWConstr = workingset->nActiveConstr - 1;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (workingset->nActiveConstr != 0) {
    for (idx = 0; idx <= mWConstr; idx++) {
      workspace->data[idx] = workingset->bwset->data[idx];
      workspace->data[idx + workspace->size[0]] = workingset->bwset->data[idx];
    }

    b_xgemv(workingset->nVar, workingset->nActiveConstr, workingset->ATwset,
            workingset->ATwset->size[0], xCurrent, workspace);
    emxInit_real_T(&b_workspace, 2, true);
    if (workingset->nActiveConstr >= workingset->nVar) {
      for (idx = 0; idx < nVar; idx++) {
        for (idx_row = 0; idx_row <= mWConstr; idx_row++) {
          qrmanager->QR->data[idx_row + qrmanager->QR->size[0] * idx] =
            workingset->ATwset->data[idx + workingset->ATwset->size[0] * idx_row];
        }
      }

      if (workingset->nActiveConstr * workingset->nVar == 0) {
        qrmanager->mrows = workingset->nActiveConstr;
        qrmanager->ncols = workingset->nVar;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = false;
        qrmanager->mrows = workingset->nActiveConstr;
        qrmanager->ncols = workingset->nVar;
        idx_row = workingset->nVar;
        for (idx = 0; idx < idx_row; idx++) {
          qrmanager->jpvt->data[idx] = idx + 1;
        }

        qrmanager->minRowCol = muIntScalarMin_sint32(workingset->nActiveConstr,
          workingset->nVar);
        xgeqrf(qrmanager->QR, workingset->nActiveConstr, workingset->nVar,
               qrmanager->tau);
      }

      idx_row = qrmanager->minRowCol;
      for (idx = 0; idx < idx_row; idx++) {
        mWConstr = (qrmanager->ldq * idx + idx) + 2;
        b_xcopy((qrmanager->mrows - idx) - 1, qrmanager->QR, mWConstr,
                qrmanager->Q, mWConstr);
      }

      xorgqr(qrmanager->mrows, qrmanager->mrows, qrmanager->minRowCol,
             qrmanager->Q, qrmanager->ldq, qrmanager->tau);
      mWConstr = workspace->size[0];
      idx_row = b_workspace->size[0] * b_workspace->size[1];
      b_workspace->size[0] = workspace->size[0];
      b_workspace->size[1] = workspace->size[1];
      emxEnsureCapacity_real_T(b_workspace, idx_row);
      idx = workspace->size[0] * workspace->size[1] - 1;
      for (idx_row = 0; idx_row <= idx; idx_row++) {
        b_workspace->data[idx_row] = workspace->data[idx_row];
      }

      xgemm(workingset->nVar, workingset->nActiveConstr, qrmanager->Q,
            qrmanager->ldq, b_workspace, workspace->size[0], workspace,
            workspace->size[0]);
      xtrsm(workingset->nVar, qrmanager->QR, qrmanager->ldq, workspace, mWConstr);
    } else {
      factorQR(qrmanager, workingset->ATwset, workingset->nVar,
               workingset->nActiveConstr);
      idx_row = qrmanager->minRowCol;
      for (idx = 0; idx < idx_row; idx++) {
        mWConstr = (qrmanager->ldq * idx + idx) + 2;
        b_xcopy((qrmanager->mrows - idx) - 1, qrmanager->QR, mWConstr,
                qrmanager->Q, mWConstr);
      }

      xorgqr(qrmanager->mrows, qrmanager->minRowCol, qrmanager->minRowCol,
             qrmanager->Q, qrmanager->ldq, qrmanager->tau);
      mWConstr = workspace->size[0];
      b_xtrsm(workingset->nActiveConstr, qrmanager->QR, qrmanager->ldq,
              workspace, workspace->size[0]);
      idx_row = b_workspace->size[0] * b_workspace->size[1];
      b_workspace->size[0] = workspace->size[0];
      b_workspace->size[1] = workspace->size[1];
      emxEnsureCapacity_real_T(b_workspace, idx_row);
      idx = workspace->size[0] * workspace->size[1] - 1;
      for (idx_row = 0; idx_row <= idx; idx_row++) {
        b_workspace->data[idx_row] = workspace->data[idx_row];
      }

      b_xgemm(workingset->nVar, workingset->nActiveConstr, qrmanager->Q,
              qrmanager->ldq, b_workspace, mWConstr, workspace, mWConstr);
    }

    emxFree_real_T(&b_workspace);
    idx = 0;
    do {
      exitg1 = 0;
      if (idx <= nVar - 1) {
        a = workspace->data[idx];
        if (muDoubleScalarIsInf(a) || muDoubleScalarIsNaN(a)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          a = workspace->data[idx + workspace->size[0]];
          if (muDoubleScalarIsInf(a) || muDoubleScalarIsNaN(a)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        if (nVar >= 1) {
          a = 1.0;
          n_t = (ptrdiff_t)nVar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          daxpy(&n_t, &a, &xCurrent->data[0], &incx_t, &workspace->data[0],
                &incy_t);
        }

        a = maxConstraintViolation(workingset, workspace, 1);
        constrViolation_basicX = maxConstraintViolation(workingset, workspace,
          workspace->size[0] + 1);
        if ((a <= 2.2204460492503131E-16) || (a < constrViolation_basicX)) {
          if (nVar >= 1) {
            n_t = (ptrdiff_t)nVar;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            dcopy(&n_t, &workspace->data[0], &incx_t, &xCurrent->data[0],
                  &incy_t);
          }
        } else {
          if (nVar >= 1) {
            n_t = (ptrdiff_t)nVar;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            dcopy(&n_t, &workspace->data[workspace->size[0]], &incx_t,
                  &xCurrent->data[0], &incy_t);
          }
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return nonDegenerateWset;
}

/* End of code generation (feasibleX0ForWorkingSet.c) */
