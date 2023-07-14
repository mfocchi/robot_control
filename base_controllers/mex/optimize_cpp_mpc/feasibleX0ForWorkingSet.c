/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * feasibleX0ForWorkingSet.c
 *
 * Code generation for function 'feasibleX0ForWorkingSet'
 *
 */

/* Include files */
#include "feasibleX0ForWorkingSet.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "optimize_cpp_mpc_data.h"
#include "optimize_cpp_mpc_emxutil.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "xgeqrf.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
boolean_T feasibleX0ForWorkingSet(emxArray_real_T *workspace,
                                  emxArray_real_T *xCurrent,
                                  const h_struct_T *workingset,
                                  c_struct_T *qrmanager)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emxArray_real_T *B;
  real_T alpha1;
  real_T beta1;
  real_T *B_data;
  real_T *workspace_data;
  real_T *xCurrent_data;
  int32_T idx;
  int32_T idxLB;
  int32_T mUB;
  int32_T mWConstr;
  int32_T nVar;
  char_T SIDE1;
  char_T TRANSA;
  char_T TRANSA1;
  char_T UPLO1;
  boolean_T nonDegenerateWset;
  xCurrent_data = xCurrent->data;
  workspace_data = workspace->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  emxInit_real_T(&B, 2);
  if (mWConstr != 0) {
    int32_T offsetQR;
    for (idx = 0; idx < mWConstr; idx++) {
      alpha1 = workingset->bwset->data[idx];
      workspace_data[idx] = alpha1;
      workspace_data[idx + workspace->size[0]] = alpha1;
    }
    if ((nVar >= 1) && (mWConstr >= 1)) {
      alpha1 = -1.0;
      beta1 = 1.0;
      TRANSA = 'T';
      m_t = (ptrdiff_t)nVar;
      n_t = (ptrdiff_t)mWConstr;
      lda_t = (ptrdiff_t)workingset->ldA;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      dgemv(&TRANSA, &m_t, &n_t, &alpha1, &workingset->ATwset->data[0], &lda_t,
            &xCurrent_data[0], &incx_t, &beta1, &workspace_data[0], &incy_t);
    }
    if (mWConstr >= nVar) {
      for (idxLB = 0; idxLB < nVar; idxLB++) {
        offsetQR = qrmanager->ldq * idxLB;
        for (mUB = 0; mUB < mWConstr; mUB++) {
          qrmanager->QR->data[mUB + offsetQR] =
              workingset->ATwset->data[idxLB + workingset->ldA * mUB];
        }
      }
      if (mWConstr * nVar == 0) {
        qrmanager->mrows = mWConstr;
        qrmanager->ncols = nVar;
        qrmanager->minRowCol = 0;
      } else {
        qrmanager->usedPivoting = false;
        qrmanager->mrows = mWConstr;
        qrmanager->ncols = nVar;
        offsetQR = (nVar / 4) << 2;
        idxLB = offsetQR - 4;
        for (idx = 0; idx <= idxLB; idx += 4) {
          _mm_storeu_si128(
              (__m128i *)&qrmanager->jpvt->data[idx],
              _mm_add_epi32(
                  _mm_add_epi32(_mm_set1_epi32(idx),
                                _mm_loadu_si128((const __m128i *)&iv[0])),
                  _mm_set1_epi32(1)));
        }
        for (idx = offsetQR; idx < nVar; idx++) {
          qrmanager->jpvt->data[idx] = idx + 1;
        }
        qrmanager->minRowCol = muIntScalarMin_sint32(mWConstr, nVar);
        xgeqrf(qrmanager->QR, mWConstr, nVar, qrmanager->tau);
      }
      computeQ_(qrmanager, qrmanager->mrows);
      if ((nVar >= 1) && (mWConstr >= 1)) {
        alpha1 = 1.0;
        idxLB = B->size[0] * B->size[1];
        B->size[0] = workspace->size[0];
        B->size[1] = workspace->size[1];
        emxEnsureCapacity_real_T(B, idxLB);
        B_data = B->data;
        offsetQR = workspace->size[0] * workspace->size[1];
        for (idxLB = 0; idxLB < offsetQR; idxLB++) {
          B_data[idxLB] = workspace_data[idxLB];
        }
        beta1 = 0.0;
        TRANSA = 'N';
        TRANSA1 = 'T';
        m_t = (ptrdiff_t)nVar;
        n_t = (ptrdiff_t)2;
        incy_t = (ptrdiff_t)mWConstr;
        lda_t = (ptrdiff_t)qrmanager->ldq;
        incx_t = (ptrdiff_t)workspace->size[0];
        ldc_t = (ptrdiff_t)workspace->size[0];
        dgemm(&TRANSA1, &TRANSA, &m_t, &n_t, &incy_t, &alpha1,
              &qrmanager->Q->data[0], &lda_t, &B_data[0], &incx_t, &beta1,
              &workspace_data[0], &ldc_t);
      }
      if (nVar >= 1) {
        alpha1 = 1.0;
        TRANSA = 'N';
        TRANSA1 = 'N';
        UPLO1 = 'U';
        SIDE1 = 'L';
        m_t = (ptrdiff_t)nVar;
        n_t = (ptrdiff_t)2;
        lda_t = (ptrdiff_t)qrmanager->ldq;
        incx_t = (ptrdiff_t)workspace->size[0];
        dtrsm(&SIDE1, &UPLO1, &TRANSA1, &TRANSA, &m_t, &n_t, &alpha1,
              &qrmanager->QR->data[0], &lda_t, &workspace_data[0], &incx_t);
      }
    } else {
      factorQR(qrmanager, workingset->ATwset, nVar, mWConstr, workingset->ldA);
      computeQ_(qrmanager, qrmanager->minRowCol);
      if (mWConstr >= 1) {
        alpha1 = 1.0;
        TRANSA = 'N';
        TRANSA1 = 'T';
        UPLO1 = 'U';
        SIDE1 = 'L';
        m_t = (ptrdiff_t)mWConstr;
        n_t = (ptrdiff_t)2;
        lda_t = (ptrdiff_t)qrmanager->ldq;
        incx_t = (ptrdiff_t)workspace->size[0];
        dtrsm(&SIDE1, &UPLO1, &TRANSA1, &TRANSA, &m_t, &n_t, &alpha1,
              &qrmanager->QR->data[0], &lda_t, &workspace_data[0], &incx_t);
      }
      if ((nVar >= 1) && (mWConstr >= 1)) {
        alpha1 = 1.0;
        idxLB = B->size[0] * B->size[1];
        B->size[0] = workspace->size[0];
        B->size[1] = workspace->size[1];
        emxEnsureCapacity_real_T(B, idxLB);
        B_data = B->data;
        offsetQR = workspace->size[0] * workspace->size[1];
        for (idxLB = 0; idxLB < offsetQR; idxLB++) {
          B_data[idxLB] = workspace_data[idxLB];
        }
        beta1 = 0.0;
        TRANSA = 'N';
        TRANSA1 = 'N';
        m_t = (ptrdiff_t)nVar;
        n_t = (ptrdiff_t)2;
        incy_t = (ptrdiff_t)mWConstr;
        lda_t = (ptrdiff_t)qrmanager->ldq;
        incx_t = (ptrdiff_t)workspace->size[0];
        ldc_t = (ptrdiff_t)workspace->size[0];
        dgemm(&TRANSA1, &TRANSA, &m_t, &n_t, &incy_t, &alpha1,
              &qrmanager->Q->data[0], &lda_t, &B_data[0], &incx_t, &beta1,
              &workspace_data[0], &ldc_t);
      }
    }
    idx = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (idx <= nVar - 1) {
        alpha1 = workspace_data[idx];
        if (muDoubleScalarIsInf(alpha1) || muDoubleScalarIsNaN(alpha1)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          alpha1 = workspace_data[idx + workspace->size[0]];
          if (muDoubleScalarIsInf(alpha1) || muDoubleScalarIsNaN(alpha1)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        int32_T mFixed;
        if (nVar >= 1) {
          alpha1 = 1.0;
          n_t = (ptrdiff_t)nVar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          daxpy(&n_t, &alpha1, &xCurrent_data[0], &incx_t, &workspace_data[0],
                &incy_t);
        }
        mWConstr = workspace->size[0] - 1;
        offsetQR = workingset->sizes[3];
        mUB = workingset->sizes[4];
        mFixed = workingset->sizes[0];
        alpha1 = 0.0;
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < offsetQR; idx++) {
            idxLB = workingset->indexLB->data[idx] - 1;
            alpha1 = muDoubleScalarMax(alpha1, -workspace_data[idxLB] -
                                                   workingset->lb->data[idxLB]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < mUB; idx++) {
            offsetQR = workingset->indexUB->data[idx] - 1;
            alpha1 =
                muDoubleScalarMax(alpha1, workspace_data[offsetQR] -
                                              workingset->ub->data[offsetQR]);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < mFixed; idx++) {
            alpha1 = muDoubleScalarMax(
                alpha1,
                muDoubleScalarAbs(
                    workspace_data[workingset->indexFixed->data[idx] - 1] -
                    workingset->ub
                        ->data[workingset->indexFixed->data[idx] - 1]));
          }
        }
        offsetQR = workingset->sizes[3];
        mUB = workingset->sizes[4];
        mFixed = workingset->sizes[0];
        beta1 = 0.0;
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < offsetQR; idx++) {
            beta1 = muDoubleScalarMax(
                beta1,
                -workspace_data[mWConstr + workingset->indexLB->data[idx]] -
                    workingset->lb->data[workingset->indexLB->data[idx] - 1]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < mUB; idx++) {
            beta1 = muDoubleScalarMax(
                beta1,
                workspace_data[mWConstr + workingset->indexUB->data[idx]] -
                    workingset->ub->data[workingset->indexUB->data[idx] - 1]);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < mFixed; idx++) {
            beta1 = muDoubleScalarMax(
                beta1, muDoubleScalarAbs(
                           workspace_data[mWConstr +
                                          workingset->indexFixed->data[idx]] -
                           workingset->ub
                               ->data[workingset->indexFixed->data[idx] - 1]));
          }
        }
        if ((alpha1 <= 2.2204460492503131E-16) || (alpha1 < beta1)) {
          if (nVar >= 1) {
            n_t = (ptrdiff_t)nVar;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            dcopy(&n_t, &workspace_data[0], &incx_t, &xCurrent_data[0],
                  &incy_t);
          }
        } else if (nVar >= 1) {
          n_t = (ptrdiff_t)nVar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dcopy(&n_t, &workspace_data[workspace->size[0]], &incx_t,
                &xCurrent_data[0], &incy_t);
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  emxFree_real_T(&B);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return nonDegenerateWset;
}

/* End of code generation (feasibleX0ForWorkingSet.c) */
