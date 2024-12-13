/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_lambda.c
 *
 * Code generation for function 'compute_lambda'
 *
 */

/* Include files */
#include "compute_lambda.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void compute_lambda(emxArray_real_T *workspace, g_struct_T *solution,
                    const e_struct_T *objective, const c_struct_T *qrmanager)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T beta1;
  real_T tol;
  real_T *workspace_data;
  int32_T idx;
  int32_T nActiveConstr_tmp;
  char_T TRANSA;
  char_T TRANSA1;
  char_T UPLO1;
  workspace_data = workspace->data;
  nActiveConstr_tmp = qrmanager->ncols;
  if (qrmanager->ncols > 0) {
    int32_T idxQR;
    boolean_T guard1;
    guard1 = false;
    if (objective->objtype != 4) {
      boolean_T nonDegenerate;
      tol = 100.0 * (real_T)qrmanager->mrows * 2.2204460492503131E-16;
      if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
        nonDegenerate = true;
      } else {
        nonDegenerate = false;
      }
      if (nonDegenerate) {
        boolean_T guard2;
        idx = nActiveConstr_tmp;
        guard2 = false;
        if (qrmanager->mrows < qrmanager->ncols) {
          idxQR = qrmanager->mrows + qrmanager->ldq * (qrmanager->ncols - 1);
          while ((idx > qrmanager->mrows) &&
                 (muDoubleScalarAbs(qrmanager->QR->data[idxQR - 1]) >= tol)) {
            idx--;
            idxQR -= qrmanager->ldq;
          }
          nonDegenerate = (idx == qrmanager->mrows);
          if (nonDegenerate) {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          idxQR = idx + qrmanager->ldq * (idx - 1);
          while ((idx >= 1) &&
                 (muDoubleScalarAbs(qrmanager->QR->data[idxQR - 1]) >= tol)) {
            idx--;
            idxQR = (idxQR - qrmanager->ldq) - 1;
          }
          nonDegenerate = (idx == 0);
        }
      }
      if (!nonDegenerate) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      int32_T vectorUB;
      if (qrmanager->mrows >= 1) {
        tol = 1.0;
        beta1 = 0.0;
        TRANSA = 'T';
        m_t = (ptrdiff_t)qrmanager->mrows;
        n_t = (ptrdiff_t)qrmanager->ncols;
        lda_t = (ptrdiff_t)qrmanager->ldq;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dgemv(&TRANSA, &m_t, &n_t, &tol, &qrmanager->Q->data[0], &lda_t,
              &objective->grad->data[0], &incx_t, &beta1, &workspace_data[0],
              &incy_t);
      }
      TRANSA = 'N';
      TRANSA1 = 'N';
      UPLO1 = 'U';
      n_t = (ptrdiff_t)qrmanager->ncols;
      lda_t = (ptrdiff_t)qrmanager->ldq;
      incx_t = (ptrdiff_t)1;
      dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t, &qrmanager->QR->data[0], &lda_t,
            &workspace_data[0], &incx_t);
      idxQR = (nActiveConstr_tmp / 2) << 1;
      vectorUB = idxQR - 2;
      for (idx = 0; idx <= vectorUB; idx += 2) {
        __m128d r;
        r = _mm_loadu_pd(&workspace_data[idx]);
        _mm_storeu_pd(&solution->lambda->data[idx],
                      _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      }
      for (idx = idxQR; idx < nActiveConstr_tmp; idx++) {
        solution->lambda->data[idx] = -workspace_data[idx];
      }
    }
  }
}

/* End of code generation (compute_lambda.c) */
