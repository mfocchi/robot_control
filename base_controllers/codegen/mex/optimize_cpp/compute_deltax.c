/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_deltax.c
 *
 * Code generation for function 'compute_deltax'
 *
 */

/* Include files */
#include "compute_deltax.h"
#include "factor.h"
#include "fullColLDL2_.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "partialColLDL3_.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "xgemm.h"
#include "blas.h"
#include "lapacke.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void compute_deltax(const emxArray_real_T *H, g_struct_T *solution,
                    f_struct_T *memspace, const c_struct_T *qrmanager,
                    d_struct_T *cholmanager, const e_struct_T *objective,
                    boolean_T alwaysPositiveDef)
{
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t lda_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  emxArray_real_T *A;
  const real_T *H_data;
  real_T alpha1;
  real_T beta1;
  real_T *A_data;
  int32_T LDimSizeP1;
  int32_T i;
  int32_T idx_row;
  int32_T ldw;
  int32_T mNull_tmp;
  int32_T nVar;
  char_T TRANSA;
  char_T TRANSA1;
  char_T UPLO1;
  H_data = H->data;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  nVar = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  emxInit_real_T(&A, 2);
  if (mNull_tmp <= 0) {
    for (ldw = 0; ldw <= nVar; ldw++) {
      solution->searchDir->data[ldw] = 0.0;
    }
  } else {
    __m128d r;
    int32_T order;
    int32_T vectorUB;
    order = ((nVar + 1) / 2) << 1;
    vectorUB = order - 2;
    for (ldw = 0; ldw <= vectorUB; ldw += 2) {
      r = _mm_loadu_pd(&objective->grad->data[ldw]);
      _mm_storeu_pd(&solution->searchDir->data[ldw],
                    _mm_mul_pd(r, _mm_set1_pd(-1.0)));
    }
    for (ldw = order; ldw <= nVar; ldw++) {
      solution->searchDir->data[ldw] = -objective->grad->data[ldw];
    }
    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
      case 5:
        break;
      case 3: {
        if (alwaysPositiveDef) {
          factor(cholmanager, H, qrmanager->mrows, qrmanager->mrows);
        } else {
          vectorUB = qrmanager->mrows;
          LDimSizeP1 = cholmanager->ldm + 1;
          cholmanager->ndims = qrmanager->mrows;
          for (ldw = 0; ldw < vectorUB; ldw++) {
            if (vectorUB >= 1) {
              n_t = (ptrdiff_t)vectorUB;
              incx_t = (ptrdiff_t)1;
              incy_t = (ptrdiff_t)1;
              dcopy(&n_t, (real_T *)&H_data[qrmanager->mrows * ldw], &incx_t,
                    &cholmanager->FMat->data[cholmanager->ldm * ldw], &incy_t);
            }
          }
          if (qrmanager->mrows < 1) {
            nVar = -1;
          } else {
            n_t = (ptrdiff_t)qrmanager->mrows;
            incx_t = (ptrdiff_t)(cholmanager->ldm + 1);
            n_t = idamax(&n_t, &cholmanager->FMat->data[0], &incx_t);
            nVar = (int32_T)n_t - 1;
          }
          cholmanager->regTol_ = muDoubleScalarMax(
              muDoubleScalarAbs(
                  cholmanager->FMat->data[nVar + cholmanager->ldm * nVar]) *
                  2.2204460492503131E-16,
              0.0);
          if ((cholmanager->FMat->size[0] * cholmanager->FMat->size[1] >
               16384) &&
              (qrmanager->mrows > 128)) {
            boolean_T exitg1;
            ldw = 0;
            exitg1 = false;
            while ((!exitg1) && (ldw < vectorUB)) {
              nVar = LDimSizeP1 * ldw + 1;
              order = vectorUB - ldw;
              if (ldw + 48 <= vectorUB) {
                partialColLDL3_(cholmanager, nVar, order);
                ldw += 48;
              } else {
                fullColLDL2_(cholmanager, nVar, order);
                exitg1 = true;
              }
            }
          } else {
            fullColLDL2_(cholmanager, 1, qrmanager->mrows);
          }
          if (cholmanager->ConvexCheck) {
            ldw = 0;
            int32_T exitg2;
            do {
              exitg2 = 0;
              if (ldw <= vectorUB - 1) {
                if (cholmanager->FMat->data[ldw + cholmanager->ldm * ldw] <=
                    0.0) {
                  cholmanager->info = -ldw - 1;
                  exitg2 = 1;
                } else {
                  ldw++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else if (alwaysPositiveDef) {
          solve(cholmanager, solution->searchDir);
        } else {
          if (cholmanager->ndims >= 1) {
            TRANSA = 'U';
            TRANSA1 = 'N';
            UPLO1 = 'L';
            n_t = (ptrdiff_t)cholmanager->ndims;
            lda_t = (ptrdiff_t)cholmanager->ldm;
            incx_t = (ptrdiff_t)1;
            dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t, &cholmanager->FMat->data[0],
                  &lda_t, &solution->searchDir->data[0], &incx_t);
          }
          i = cholmanager->ndims;
          for (ldw = 0; ldw < i; ldw++) {
            solution->searchDir->data[ldw] /=
                cholmanager->FMat->data[ldw + cholmanager->ldm * ldw];
          }
          if (cholmanager->ndims >= 1) {
            TRANSA = 'U';
            TRANSA1 = 'T';
            UPLO1 = 'L';
            n_t = (ptrdiff_t)cholmanager->ndims;
            lda_t = (ptrdiff_t)cholmanager->ldm;
            incx_t = (ptrdiff_t)1;
            dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t, &cholmanager->FMat->data[0],
                  &lda_t, &solution->searchDir->data[0], &incx_t);
          }
        }
      } break;
      default:
        if (alwaysPositiveDef) {
          factor(cholmanager, H, objective->nvar, objective->nvar);
          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            solve(cholmanager, solution->searchDir);
            nVar = qrmanager->mrows - objective->nvar;
            alpha1 = 1.0 / objective->beta;
            if (nVar >= 1) {
              n_t = (ptrdiff_t)nVar;
              incx_t = (ptrdiff_t)1;
              dscal(&n_t, &alpha1, &solution->searchDir->data[objective->nvar],
                    &incx_t);
            }
          }
        }
        break;
      }
    } else {
      int32_T nullStartIdx_tmp;
      nullStartIdx_tmp = qrmanager->ldq * qrmanager->ncols;
      if (objective->objtype == 5) {
        for (ldw = 0; ldw < mNull_tmp; ldw++) {
          memspace->workspace_double->data[ldw] =
              -qrmanager->Q
                   ->data[nVar + qrmanager->ldq * (qrmanager->ncols + ldw)];
        }
        if (qrmanager->mrows >= 1) {
          alpha1 = 1.0;
          beta1 = 0.0;
          TRANSA = 'N';
          m_t = (ptrdiff_t)qrmanager->mrows;
          n_t = (ptrdiff_t)mNull_tmp;
          lda_t = (ptrdiff_t)qrmanager->ldq;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          dgemv(&TRANSA, &m_t, &n_t, &alpha1,
                &qrmanager->Q->data[nullStartIdx_tmp], &lda_t,
                &memspace->workspace_double->data[0], &incx_t, &beta1,
                &solution->searchDir->data[0], &incy_t);
        }
      } else {
        if (objective->objtype == 3) {
          xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                qrmanager->mrows, qrmanager->Q, nullStartIdx_tmp + 1,
                qrmanager->ldq, memspace->workspace_double,
                memspace->workspace_double->size[0]);
          b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp + 1, qrmanager->ldq,
                  memspace->workspace_double,
                  memspace->workspace_double->size[0], cholmanager->FMat,
                  cholmanager->ldm);
        } else if (alwaysPositiveDef) {
          nVar = qrmanager->mrows;
          ldw = memspace->workspace_double->size[0];
          xgemm(objective->nvar, mNull_tmp, objective->nvar, H, objective->nvar,
                qrmanager->Q, nullStartIdx_tmp + 1, qrmanager->ldq,
                memspace->workspace_double,
                memspace->workspace_double->size[0]);
          for (LDimSizeP1 = 0; LDimSizeP1 < mNull_tmp; LDimSizeP1++) {
            i = objective->nvar + 1;
            order = ((((nVar - i) + 1) / 2) << 1) + i;
            vectorUB = order - 2;
            for (idx_row = i; idx_row <= vectorUB; idx_row += 2) {
              r = _mm_loadu_pd(
                  &qrmanager->Q
                       ->data[(idx_row + qrmanager->Q->size[0] *
                                             (LDimSizeP1 + qrmanager->ncols)) -
                              1]);
              _mm_storeu_pd(
                  &memspace->workspace_double
                       ->data[(idx_row + memspace->workspace_double->size[0] *
                                             LDimSizeP1) -
                              1],
                  _mm_mul_pd(_mm_set1_pd(objective->beta), r));
            }
            for (idx_row = order; idx_row <= nVar; idx_row++) {
              memspace->workspace_double
                  ->data[(idx_row +
                          memspace->workspace_double->size[0] * LDimSizeP1) -
                         1] =
                  objective->beta *
                  qrmanager->Q
                      ->data[(idx_row + qrmanager->Q->size[0] *
                                            (LDimSizeP1 + qrmanager->ncols)) -
                             1];
            }
          }
          b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                  nullStartIdx_tmp + 1, qrmanager->ldq,
                  memspace->workspace_double, ldw, cholmanager->FMat,
                  cholmanager->ldm);
        }
        if (alwaysPositiveDef) {
          cholmanager->ndims = mNull_tmp;
          i = A->size[0] * A->size[1];
          A->size[0] = cholmanager->FMat->size[0];
          A->size[1] = cholmanager->FMat->size[1];
          emxEnsureCapacity_real_T(A, i);
          A_data = A->data;
          nVar = cholmanager->FMat->size[0] * cholmanager->FMat->size[1];
          for (i = 0; i < nVar; i++) {
            A_data[i] = cholmanager->FMat->data[i];
          }
          n_t = LAPACKE_dpotrf_work(102, 'U', (ptrdiff_t)mNull_tmp, &A_data[0],
                                    (ptrdiff_t)cholmanager->ldm);
          if ((int32_T)n_t < 0) {
            nVar = A->size[0];
            ldw = A->size[1];
            i = A->size[0] * A->size[1];
            A->size[0] = nVar;
            A->size[1] = ldw;
            emxEnsureCapacity_real_T(A, i);
            A_data = A->data;
            nVar *= ldw;
            for (i = 0; i < nVar; i++) {
              A_data[i] = rtNaN;
            }
          }
          cholmanager->info = (int32_T)n_t;
          i = cholmanager->FMat->size[0] * cholmanager->FMat->size[1];
          cholmanager->FMat->size[0] = A->size[0];
          cholmanager->FMat->size[1] = A->size[1];
          emxEnsureCapacity_real_T(cholmanager->FMat, i);
          nVar = A->size[0] * A->size[1];
          for (i = 0; i < nVar; i++) {
            cholmanager->FMat->data[i] = A_data[i];
          }
        } else {
          LDimSizeP1 = cholmanager->ldm + 1;
          cholmanager->ndims = mNull_tmp;
          n_t = (ptrdiff_t)mNull_tmp;
          incx_t = (ptrdiff_t)(cholmanager->ldm + 1);
          n_t = idamax(&n_t, &cholmanager->FMat->data[0], &incx_t);
          cholmanager->regTol_ = muDoubleScalarMax(
              muDoubleScalarAbs(
                  cholmanager->FMat
                      ->data[((int32_T)n_t +
                              cholmanager->ldm * ((int32_T)n_t - 1)) -
                             1]) *
                  2.2204460492503131E-16,
              0.0);
          if ((cholmanager->FMat->size[0] * cholmanager->FMat->size[1] >
               16384) &&
              (mNull_tmp > 128)) {
            boolean_T exitg1;
            ldw = 0;
            exitg1 = false;
            while ((!exitg1) && (ldw < mNull_tmp)) {
              nVar = LDimSizeP1 * ldw + 1;
              order = mNull_tmp - ldw;
              if (ldw + 48 <= mNull_tmp) {
                partialColLDL3_(cholmanager, nVar, order);
                ldw += 48;
              } else {
                fullColLDL2_(cholmanager, nVar, order);
                exitg1 = true;
              }
            }
          } else {
            fullColLDL2_(cholmanager, 1, mNull_tmp);
          }
          if (cholmanager->ConvexCheck) {
            ldw = 0;
            int32_T exitg2;
            do {
              exitg2 = 0;
              if (ldw <= mNull_tmp - 1) {
                if (cholmanager->FMat->data[ldw + cholmanager->ldm * ldw] <=
                    0.0) {
                  cholmanager->info = -ldw - 1;
                  exitg2 = 1;
                } else {
                  ldw++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }
        }
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows >= 1) {
            alpha1 = -1.0;
            beta1 = 0.0;
            TRANSA = 'T';
            m_t = (ptrdiff_t)qrmanager->mrows;
            n_t = (ptrdiff_t)mNull_tmp;
            lda_t = (ptrdiff_t)qrmanager->ldq;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            dgemv(&TRANSA, &m_t, &n_t, &alpha1,
                  &qrmanager->Q->data[nullStartIdx_tmp], &lda_t,
                  &objective->grad->data[0], &incx_t, &beta1,
                  &memspace->workspace_double->data[0], &incy_t);
          }
          i = A->size[0] * A->size[1];
          A->size[0] = memspace->workspace_double->size[0];
          A->size[1] = memspace->workspace_double->size[1];
          emxEnsureCapacity_real_T(A, i);
          A_data = A->data;
          nVar = memspace->workspace_double->size[0] *
                 memspace->workspace_double->size[1];
          for (i = 0; i < nVar; i++) {
            A_data[i] = memspace->workspace_double->data[i];
          }
          if (alwaysPositiveDef) {
            if (cholmanager->ndims >= 1) {
              TRANSA = 'N';
              TRANSA1 = 'T';
              UPLO1 = 'U';
              n_t = (ptrdiff_t)cholmanager->ndims;
              lda_t = (ptrdiff_t)cholmanager->ldm;
              incx_t = (ptrdiff_t)1;
              dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t,
                    &cholmanager->FMat->data[0], &lda_t, &A_data[0], &incx_t);
            }
            if (cholmanager->ndims >= 1) {
              TRANSA = 'N';
              TRANSA1 = 'N';
              UPLO1 = 'U';
              n_t = (ptrdiff_t)cholmanager->ndims;
              lda_t = (ptrdiff_t)cholmanager->ldm;
              incx_t = (ptrdiff_t)1;
              dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t,
                    &cholmanager->FMat->data[0], &lda_t, &A_data[0], &incx_t);
            }
          } else {
            if (cholmanager->ndims >= 1) {
              TRANSA = 'U';
              TRANSA1 = 'N';
              UPLO1 = 'L';
              n_t = (ptrdiff_t)cholmanager->ndims;
              lda_t = (ptrdiff_t)cholmanager->ldm;
              incx_t = (ptrdiff_t)1;
              dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t,
                    &cholmanager->FMat->data[0], &lda_t, &A_data[0], &incx_t);
            }
            i = cholmanager->ndims;
            for (ldw = 0; ldw < i; ldw++) {
              A_data[ldw] /=
                  cholmanager->FMat->data[ldw + cholmanager->ldm * ldw];
            }
            if (cholmanager->ndims >= 1) {
              TRANSA = 'U';
              TRANSA1 = 'T';
              UPLO1 = 'L';
              n_t = (ptrdiff_t)cholmanager->ndims;
              lda_t = (ptrdiff_t)cholmanager->ldm;
              incx_t = (ptrdiff_t)1;
              dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t,
                    &cholmanager->FMat->data[0], &lda_t, &A_data[0], &incx_t);
            }
          }
          i = memspace->workspace_double->size[0] *
              memspace->workspace_double->size[1];
          memspace->workspace_double->size[0] = A->size[0];
          memspace->workspace_double->size[1] = A->size[1];
          emxEnsureCapacity_real_T(memspace->workspace_double, i);
          nVar = A->size[0] * A->size[1];
          for (i = 0; i < nVar; i++) {
            memspace->workspace_double->data[i] = A_data[i];
          }
          if (qrmanager->mrows >= 1) {
            alpha1 = 1.0;
            beta1 = 0.0;
            TRANSA = 'N';
            m_t = (ptrdiff_t)qrmanager->mrows;
            n_t = (ptrdiff_t)mNull_tmp;
            lda_t = (ptrdiff_t)qrmanager->ldq;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            dgemv(&TRANSA, &m_t, &n_t, &alpha1,
                  &qrmanager->Q->data[nullStartIdx_tmp], &lda_t, &A_data[0],
                  &incx_t, &beta1, &solution->searchDir->data[0], &incy_t);
          }
        }
      }
    }
  }
  emxFree_real_T(&A);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (compute_deltax.c) */
