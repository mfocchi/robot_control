/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * PresolveWorkingSet.c
 *
 * Code generation for function 'PresolveWorkingSet'
 *
 */

/* Include files */
#include "PresolveWorkingSet.h"
#include "computeQ_.h"
#include "countsort.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "optimize_cpp_mpc_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include "blas.h"
#include "mwmathutil.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void PresolveWorkingSet(g_struct_T *solution, f_struct_T *memspace,
                        h_struct_T *workingset, c_struct_T *qrmanager)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  real_T tol;
  int32_T i;
  int32_T idx;
  int32_T idxDiag;
  int32_T idxStartIneq;
  int32_T mTotalWorkingEq_tmp_tmp;
  int32_T mWorkingFixed;
  int32_T nVar_tmp;
  solution->state = 82;
  nVar_tmp = workingset->nVar;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  idxStartIneq = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    int32_T totalRank_tmp;
    for (idxStartIneq = 0; idxStartIneq < mTotalWorkingEq_tmp_tmp;
         idxStartIneq++) {
      for (idxDiag = 0; idxDiag < nVar_tmp; idxDiag++) {
        qrmanager->QR->data[idxStartIneq + qrmanager->ldq * idxDiag] =
            workingset->ATwset->data[idxDiag + workingset->ldA * idxStartIneq];
      }
    }
    idxStartIneq = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    idxStartIneq = muIntScalarMax_sint32(0, idxStartIneq);
    for (idx = 0; idx < nVar_tmp; idx++) {
      qrmanager->jpvt->data[idx] = 0;
    }
    i = mTotalWorkingEq_tmp_tmp * workingset->nVar;
    if (i == 0) {
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol = 0;
    } else {
      qrmanager->usedPivoting = true;
      qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager->ncols = workingset->nVar;
      qrmanager->minRowCol =
          muIntScalarMin_sint32(mTotalWorkingEq_tmp_tmp, workingset->nVar);
      xgeqp3(qrmanager->QR, mTotalWorkingEq_tmp_tmp, workingset->nVar,
             qrmanager->jpvt, qrmanager->tau);
    }
    tol = 100.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
    totalRank_tmp =
        muIntScalarMin_sint32(workingset->nVar, mTotalWorkingEq_tmp_tmp);
    idxDiag = totalRank_tmp + qrmanager->ldq * (totalRank_tmp - 1);
    while ((idxDiag > 0) &&
           (muDoubleScalarAbs(qrmanager->QR->data[idxDiag - 1]) < tol)) {
      idxDiag = (idxDiag - qrmanager->ldq) - 1;
      idxStartIneq++;
    }
    if (idxStartIneq > 0) {
      boolean_T exitg1;
      computeQ_(qrmanager, qrmanager->mrows);
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= idxStartIneq - 1)) {
        real_T x;
        n_t = (ptrdiff_t)mTotalWorkingEq_tmp_tmp;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        x = ddot(&n_t,
                 &qrmanager->Q->data[qrmanager->ldq *
                                     ((mTotalWorkingEq_tmp_tmp - idx) - 1)],
                 &incx_t, &workingset->bwset->data[0], &incy_t);
        if (muDoubleScalarAbs(x) >= tol) {
          idxStartIneq = -1;
          exitg1 = true;
        } else {
          idx++;
        }
      }
    }
    if (idxStartIneq > 0) {
      for (idxDiag = 0; idxDiag < mTotalWorkingEq_tmp_tmp; idxDiag++) {
        if (nVar_tmp >= 1) {
          n_t = (ptrdiff_t)nVar_tmp;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dcopy(&n_t, &workingset->ATwset->data[workingset->ldA * idxDiag],
                &incx_t, &qrmanager->QR->data[qrmanager->ldq * idxDiag],
                &incy_t);
        }
      }
      for (idx = 0; idx < mWorkingFixed; idx++) {
        qrmanager->jpvt->data[idx] = 1;
      }
      mWorkingFixed = workingset->nWConstr[0] + 1;
      for (idx = mWorkingFixed; idx <= mTotalWorkingEq_tmp_tmp; idx++) {
        qrmanager->jpvt->data[idx - 1] = 0;
      }
      if (i == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager->minRowCol = totalRank_tmp;
        xgeqp3(qrmanager->QR, workingset->nVar, mTotalWorkingEq_tmp_tmp,
               qrmanager->jpvt, qrmanager->tau);
      }
      for (idx = 0; idx < idxStartIneq; idx++) {
        memspace->workspace_int->data[idx] =
            qrmanager->jpvt
                ->data[(mTotalWorkingEq_tmp_tmp - idxStartIneq) + idx];
      }
      countsort(memspace->workspace_int, idxStartIneq, memspace->workspace_sort,
                1, mTotalWorkingEq_tmp_tmp);
      for (idx = idxStartIneq; idx >= 1; idx--) {
        i = memspace->workspace_int->data[idx - 1];
        if (i <= mTotalWorkingEq_tmp_tmp) {
          if ((workingset->nActiveConstr == mTotalWorkingEq_tmp_tmp) ||
              (i == mTotalWorkingEq_tmp_tmp)) {
            workingset->mEqRemoved++;
            /* A check that is always false is detected at compile-time.
             * Eliminating code that follows. */
          } else {
            workingset->mEqRemoved++;
            /* A check that is always false is detected at compile-time.
             * Eliminating code that follows. */
          }
        }
      }
    }
  }
  if ((idxStartIneq != -1) && (workingset->nActiveConstr <= qrmanager->ldq)) {
    boolean_T guard1;
    boolean_T okWorkingSet;
    idxStartIneq = workingset->nActiveConstr;
    if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
            workingset->nWConstr[4] >
        0) {
      tol = 100.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
      for (idx = 0; idx < mTotalWorkingEq_tmp_tmp; idx++) {
        qrmanager->jpvt->data[idx] = 1;
      }
      i = mTotalWorkingEq_tmp_tmp + 1;
      for (idx = i; idx <= idxStartIneq; idx++) {
        qrmanager->jpvt->data[idx - 1] = 0;
      }
      i = workingset->nActiveConstr;
      for (idxDiag = 0; idxDiag < i; idxDiag++) {
        if (nVar_tmp >= 1) {
          n_t = (ptrdiff_t)nVar_tmp;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dcopy(&n_t, &workingset->ATwset->data[workingset->ldA * idxDiag],
                &incx_t, &qrmanager->QR->data[qrmanager->ldq * idxDiag],
                &incy_t);
        }
      }
      if (workingset->nVar * workingset->nActiveConstr == 0) {
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = workingset->nActiveConstr;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = true;
        qrmanager->mrows = workingset->nVar;
        qrmanager->ncols = workingset->nActiveConstr;
        qrmanager->minRowCol =
            muIntScalarMin_sint32(workingset->nVar, workingset->nActiveConstr);
        xgeqp3(qrmanager->QR, workingset->nVar, workingset->nActiveConstr,
               qrmanager->jpvt, qrmanager->tau);
      }
      idxStartIneq = 0;
      for (idx = workingset->nActiveConstr - 1; idx + 1 > nVar_tmp; idx--) {
        idxStartIneq++;
        memspace->workspace_int->data[idxStartIneq - 1] =
            qrmanager->jpvt->data[idx];
      }
      if (idx + 1 <= workingset->nVar) {
        idxDiag = idx + qrmanager->ldq * idx;
        while ((idx + 1 > mTotalWorkingEq_tmp_tmp) &&
               (muDoubleScalarAbs(qrmanager->QR->data[idxDiag]) < tol)) {
          idxStartIneq++;
          memspace->workspace_int->data[idxStartIneq - 1] =
              qrmanager->jpvt->data[idx];
          idx--;
          idxDiag = (idxDiag - qrmanager->ldq) - 1;
        }
      }
      countsort(memspace->workspace_int, idxStartIneq, memspace->workspace_sort,
                mTotalWorkingEq_tmp_tmp + 1, workingset->nActiveConstr);
      for (idx = idxStartIneq; idx >= 1; idx--) {
        removeConstr(workingset, memspace->workspace_int->data[idx - 1]);
      }
    }
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace->workspace_double, solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      idxStartIneq = workingset->nActiveConstr;
      i = workingset->nWConstr[0] + workingset->nWConstr[1];
      if ((workingset->nWConstr[2] + workingset->nWConstr[3]) +
              workingset->nWConstr[4] >
          0) {
        tol = 1000.0 * (real_T)workingset->nVar * 2.2204460492503131E-16;
        for (idx = 0; idx < i; idx++) {
          qrmanager->jpvt->data[idx] = 1;
        }
        mWorkingFixed = i + 1;
        for (idx = mWorkingFixed; idx <= idxStartIneq; idx++) {
          qrmanager->jpvt->data[idx - 1] = 0;
        }
        mWorkingFixed = workingset->nActiveConstr;
        for (idxDiag = 0; idxDiag < mWorkingFixed; idxDiag++) {
          if (nVar_tmp >= 1) {
            n_t = (ptrdiff_t)nVar_tmp;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            dcopy(&n_t, &workingset->ATwset->data[workingset->ldA * idxDiag],
                  &incx_t, &qrmanager->QR->data[qrmanager->ldq * idxDiag],
                  &incy_t);
          }
        }
        if (workingset->nVar * workingset->nActiveConstr == 0) {
          qrmanager->mrows = workingset->nVar;
          qrmanager->ncols = workingset->nActiveConstr;
          qrmanager->minRowCol = 0;
        } else {
          qrmanager->usedPivoting = true;
          qrmanager->mrows = workingset->nVar;
          qrmanager->ncols = workingset->nActiveConstr;
          qrmanager->minRowCol = muIntScalarMin_sint32(
              workingset->nVar, workingset->nActiveConstr);
          xgeqp3(qrmanager->QR, workingset->nVar, workingset->nActiveConstr,
                 qrmanager->jpvt, qrmanager->tau);
        }
        idxStartIneq = 0;
        for (idx = workingset->nActiveConstr - 1; idx + 1 > nVar_tmp; idx--) {
          idxStartIneq++;
          memspace->workspace_int->data[idxStartIneq - 1] =
              qrmanager->jpvt->data[idx];
        }
        if (idx + 1 <= workingset->nVar) {
          idxDiag = idx + qrmanager->ldq * idx;
          while ((idx + 1 > i) &&
                 (muDoubleScalarAbs(qrmanager->QR->data[idxDiag]) < tol)) {
            idxStartIneq++;
            memspace->workspace_int->data[idxStartIneq - 1] =
                qrmanager->jpvt->data[idx];
            idx--;
            idxDiag = (idxDiag - qrmanager->ldq) - 1;
          }
        }
        countsort(memspace->workspace_int, idxStartIneq,
                  memspace->workspace_sort, i + 1, workingset->nActiveConstr);
        for (idx = idxStartIneq; idx >= 1; idx--) {
          removeConstr(workingset, memspace->workspace_int->data[idx - 1]);
        }
      }
      okWorkingSet = feasibleX0ForWorkingSet(
          memspace->workspace_double, solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      tol = maxConstraintViolation(workingset, solution->xstar);
      if (tol > 0.001) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    idxStartIneq = mTotalWorkingEq_tmp_tmp + 1;
    idxDiag = workingset->nActiveConstr;
    for (mWorkingFixed = idxStartIneq; mWorkingFixed <= idxDiag;
         mWorkingFixed++) {
      workingset->isActiveConstr
          ->data[(workingset->isActiveIdx
                      [workingset->Wid->data[mWorkingFixed - 1] - 1] +
                  workingset->Wlocalidx->data[mWorkingFixed - 1]) -
                 2] = false;
    }
    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = mTotalWorkingEq_tmp_tmp;
  }
}

/* End of code generation (PresolveWorkingSet.c) */
