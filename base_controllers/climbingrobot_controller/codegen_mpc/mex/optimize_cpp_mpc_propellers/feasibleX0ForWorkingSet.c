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
#include "optimize_cpp_mpc_propellers_data.h"
#include "optimize_cpp_mpc_propellers_emxutil.h"
#include "optimize_cpp_mpc_propellers_types.h"
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
  *xCurrent, j_struct_T *workingset, g_struct_T *qrmanager)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emxArray_real_T *b_workspace;
  real_T a;
  real_T v;
  int32_T exitg1;
  int32_T idx;
  int32_T ix0_2;
  int32_T mFixed;
  int32_T mLB;
  int32_T mUB;
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
      for (mLB = 0; mLB < nVar; mLB++) {
        for (ix0_2 = 0; ix0_2 <= mWConstr; ix0_2++) {
          qrmanager->QR->data[ix0_2 + qrmanager->QR->size[0] * mLB] =
            workingset->ATwset->data[mLB + workingset->ATwset->size[0] * ix0_2];
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
        ix0_2 = workingset->nVar;
        for (idx = 0; idx < ix0_2; idx++) {
          qrmanager->jpvt->data[idx] = idx + 1;
        }

        qrmanager->minRowCol = muIntScalarMin_sint32(workingset->nActiveConstr,
          workingset->nVar);
        xgeqrf(qrmanager->QR, workingset->nActiveConstr, workingset->nVar,
               qrmanager->tau);
      }

      ix0_2 = qrmanager->minRowCol;
      for (idx = 0; idx < ix0_2; idx++) {
        mWConstr = (qrmanager->ldq * idx + idx) + 2;
        b_xcopy((qrmanager->mrows - idx) - 1, qrmanager->QR, mWConstr,
                qrmanager->Q, mWConstr);
      }

      xorgqr(qrmanager->mrows, qrmanager->mrows, qrmanager->minRowCol,
             qrmanager->Q, qrmanager->ldq, qrmanager->tau);
      mWConstr = workspace->size[0];
      ix0_2 = b_workspace->size[0] * b_workspace->size[1];
      b_workspace->size[0] = workspace->size[0];
      b_workspace->size[1] = workspace->size[1];
      emxEnsureCapacity_real_T(b_workspace, ix0_2);
      mLB = workspace->size[0] * workspace->size[1] - 1;
      for (ix0_2 = 0; ix0_2 <= mLB; ix0_2++) {
        b_workspace->data[ix0_2] = workspace->data[ix0_2];
      }

      xgemm(workingset->nVar, workingset->nActiveConstr, qrmanager->Q,
            qrmanager->ldq, b_workspace, workspace->size[0], workspace,
            workspace->size[0]);
      xtrsm(workingset->nVar, qrmanager->QR, qrmanager->ldq, workspace, mWConstr);
    } else {
      factorQR(qrmanager, workingset->ATwset, workingset->nVar,
               workingset->nActiveConstr);
      ix0_2 = qrmanager->minRowCol;
      for (idx = 0; idx < ix0_2; idx++) {
        mWConstr = (qrmanager->ldq * idx + idx) + 2;
        b_xcopy((qrmanager->mrows - idx) - 1, qrmanager->QR, mWConstr,
                qrmanager->Q, mWConstr);
      }

      xorgqr(qrmanager->mrows, qrmanager->minRowCol, qrmanager->minRowCol,
             qrmanager->Q, qrmanager->ldq, qrmanager->tau);
      mWConstr = workspace->size[0];
      b_xtrsm(workingset->nActiveConstr, qrmanager->QR, qrmanager->ldq,
              workspace, workspace->size[0]);
      ix0_2 = b_workspace->size[0] * b_workspace->size[1];
      b_workspace->size[0] = workspace->size[0];
      b_workspace->size[1] = workspace->size[1];
      emxEnsureCapacity_real_T(b_workspace, ix0_2);
      mLB = workspace->size[0] * workspace->size[1] - 1;
      for (ix0_2 = 0; ix0_2 <= mLB; ix0_2++) {
        b_workspace->data[ix0_2] = workspace->data[ix0_2];
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

        ix0_2 = workspace->size[0];
        mLB = workingset->sizes[3];
        mUB = workingset->sizes[4];
        mFixed = workingset->sizes[0];
        switch (workingset->probType) {
         case 2:
          a = 0.0;
          mWConstr = workingset->sizes[2];
          if ((workingset->Aineq->size[0] != 0) && (workingset->Aineq->size[1]
               != 0)) {
            xcopy(workingset->sizes[2], workingset->bineq,
                  workingset->maxConstrWorkspace);
            c_xgemv(workingset->nVarOrig, workingset->sizes[2],
                    workingset->Aineq, workingset->ldA, workspace,
                    workingset->maxConstrWorkspace);
            for (idx = 0; idx < mWConstr; idx++) {
              workingset->maxConstrWorkspace->data[idx] -= workspace->
                data[workingset->nVarOrig + idx];
              a = muDoubleScalarMax(a, workingset->maxConstrWorkspace->data[idx]);
            }
          }
          break;

         default:
          a = 0.0;
          mWConstr = workingset->sizes[2];
          if ((workingset->Aineq->size[0] != 0) && (workingset->Aineq->size[1]
               != 0)) {
            xcopy(workingset->sizes[2], workingset->bineq,
                  workingset->maxConstrWorkspace);
            c_xgemv(workingset->nVar, workingset->sizes[2], workingset->Aineq,
                    workingset->ldA, workspace, workingset->maxConstrWorkspace);
            for (idx = 0; idx < mWConstr; idx++) {
              a = muDoubleScalarMax(a, workingset->maxConstrWorkspace->data[idx]);
            }
          }
          break;
        }

        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < mLB; idx++) {
            mWConstr = workingset->indexLB->data[idx] - 1;
            a = muDoubleScalarMax(a, -workspace->data[mWConstr] - workingset->
                                  lb->data[mWConstr]);
          }
        }

        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < mUB; idx++) {
            mLB = workingset->indexUB->data[idx] - 1;
            a = muDoubleScalarMax(a, workspace->data[mLB] - workingset->ub->
                                  data[mLB]);
          }
        }

        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < mFixed; idx++) {
            a = muDoubleScalarMax(a, muDoubleScalarAbs(workspace->
              data[workingset->indexFixed->data[idx] - 1] - workingset->ub->
              data[workingset->indexFixed->data[idx] - 1]));
          }
        }

        mLB = workingset->sizes[3];
        mUB = workingset->sizes[4];
        mFixed = workingset->sizes[0];
        switch (workingset->probType) {
         case 2:
          v = 0.0;
          mWConstr = workingset->sizes[2];
          if ((workingset->Aineq->size[0] != 0) && (workingset->Aineq->size[1]
               != 0)) {
            xcopy(workingset->sizes[2], workingset->bineq,
                  workingset->maxConstrWorkspace);
            d_xgemv(workingset->nVarOrig, workingset->sizes[2],
                    workingset->Aineq, workingset->ldA, workspace,
                    workspace->size[0] + 1, workingset->maxConstrWorkspace);
            for (idx = 0; idx < mWConstr; idx++) {
              workingset->maxConstrWorkspace->data[idx] -= workspace->data
                [(ix0_2 + workingset->nVarOrig) + idx];
              v = muDoubleScalarMax(v, workingset->maxConstrWorkspace->data[idx]);
            }
          }
          break;

         default:
          v = 0.0;
          mWConstr = workingset->sizes[2];
          if ((workingset->Aineq->size[0] != 0) && (workingset->Aineq->size[1]
               != 0)) {
            xcopy(workingset->sizes[2], workingset->bineq,
                  workingset->maxConstrWorkspace);
            d_xgemv(workingset->nVar, workingset->sizes[2], workingset->Aineq,
                    workingset->ldA, workspace, workspace->size[0] + 1,
                    workingset->maxConstrWorkspace);
            for (idx = 0; idx < mWConstr; idx++) {
              v = muDoubleScalarMax(v, workingset->maxConstrWorkspace->data[idx]);
            }
          }
          break;
        }

        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < mLB; idx++) {
            v = muDoubleScalarMax(v, -workspace->data[(ix0_2 +
              workingset->indexLB->data[idx]) - 1] - workingset->lb->
                                  data[workingset->indexLB->data[idx] - 1]);
          }
        }

        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < mUB; idx++) {
            v = muDoubleScalarMax(v, workspace->data[(ix0_2 +
              workingset->indexUB->data[idx]) - 1] - workingset->ub->
                                  data[workingset->indexUB->data[idx] - 1]);
          }
        }

        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < mFixed; idx++) {
            v = muDoubleScalarMax(v, muDoubleScalarAbs(workspace->data[(ix0_2 +
              workingset->indexFixed->data[idx]) - 1] - workingset->ub->
              data[workingset->indexFixed->data[idx] - 1]));
          }
        }

        if ((a <= 2.2204460492503131E-16) || (a < v)) {
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
            dcopy(&n_t, &workspace->data[ix0_2], &incx_t, &xCurrent->data[0],
                  &incy_t);
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
