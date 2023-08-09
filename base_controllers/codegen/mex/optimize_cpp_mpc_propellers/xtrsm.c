/*
 * Non-Degree Granting Education License -- for use at non-degree
 * granting, nonprofit, educational organizations only. Not for
 * government, commercial, or other organizational use.
 *
 * xtrsm.c
 *
 * Code generation for function 'xtrsm'
 *
 */

/* Include files */
#include "xtrsm.h"
#include "optimize_cpp_mpc_propellers_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void b_xtrsm(int32_T m, const emxArray_real_T *A, int32_T lda, emxArray_real_T
             *B, int32_T ldb)
{
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  char_T DIAGA1;
  char_T SIDE1;
  char_T TRANSA1;
  char_T UPLO1;
  if (m >= 1) {
    alpha1 = 1.0;
    DIAGA1 = 'N';
    TRANSA1 = 'T';
    UPLO1 = 'U';
    SIDE1 = 'L';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)2;
    lda_t = (ptrdiff_t)lda;
    ldb_t = (ptrdiff_t)ldb;
    dtrsm(&SIDE1, &UPLO1, &TRANSA1, &DIAGA1, &m_t, &n_t, &alpha1, &A->data[0],
          &lda_t, &B->data[0], &ldb_t);
  }
}

void xtrsm(int32_T m, const emxArray_real_T *A, int32_T lda, emxArray_real_T *B,
           int32_T ldb)
{
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  real_T alpha1;
  char_T DIAGA1;
  char_T SIDE1;
  char_T TRANSA1;
  char_T UPLO1;
  if (m >= 1) {
    alpha1 = 1.0;
    DIAGA1 = 'N';
    TRANSA1 = 'N';
    UPLO1 = 'U';
    SIDE1 = 'L';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)2;
    lda_t = (ptrdiff_t)lda;
    ldb_t = (ptrdiff_t)ldb;
    dtrsm(&SIDE1, &UPLO1, &TRANSA1, &DIAGA1, &m_t, &n_t, &alpha1, &A->data[0],
          &lda_t, &B->data[0], &ldb_t);
  }
}

/* End of code generation (xtrsm.c) */
