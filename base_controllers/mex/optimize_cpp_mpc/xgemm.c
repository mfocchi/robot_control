/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgemm.c
 *
 * Code generation for function 'xgemm'
 *
 */

/* Include files */
#include "xgemm.h"
#include "optimize_cpp_mpc_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void b_xgemm(int32_T m, int32_T n, int32_T k, const emxArray_real_T *A,
             int32_T ia0, int32_T lda, const emxArray_real_T *B, int32_T ldb,
             emxArray_real_T *C, int32_T ldc)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  const real_T *A_data;
  const real_T *B_data;
  real_T alpha1;
  real_T beta1;
  real_T *C_data;
  char_T TRANSA1;
  char_T TRANSB1;
  C_data = C->data;
  B_data = B->data;
  A_data = A->data;
  if ((m >= 1) && (n >= 1) && (k >= 1)) {
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSB1 = 'N';
    TRANSA1 = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    k_t = (ptrdiff_t)k;
    lda_t = (ptrdiff_t)lda;
    ldb_t = (ptrdiff_t)ldb;
    ldc_t = (ptrdiff_t)ldc;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1,
          (real_T *)&A_data[ia0 - 1], &lda_t, (real_T *)&B_data[0], &ldb_t,
          &beta1, &C_data[0], &ldc_t);
  }
}

void xgemm(int32_T m, int32_T n, int32_T k, const emxArray_real_T *A,
           int32_T lda, const emxArray_real_T *B, int32_T ib0, int32_T ldb,
           emxArray_real_T *C, int32_T ldc)
{
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  const real_T *A_data;
  const real_T *B_data;
  real_T alpha1;
  real_T beta1;
  real_T *C_data;
  char_T TRANSA1;
  char_T TRANSB1;
  C_data = C->data;
  B_data = B->data;
  A_data = A->data;
  if ((m >= 1) && (n >= 1) && (k >= 1)) {
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    k_t = (ptrdiff_t)k;
    lda_t = (ptrdiff_t)lda;
    ldb_t = (ptrdiff_t)ldb;
    ldc_t = (ptrdiff_t)ldc;
    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, (real_T *)&A_data[0],
          &lda_t, (real_T *)&B_data[ib0 - 1], &ldb_t, &beta1, &C_data[0],
          &ldc_t);
  }
}

/* End of code generation (xgemm.c) */
