/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solve.c
 *
 * Code generation for function 'solve'
 *
 */

/* Include files */
#include "solve.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "blas.h"
#include <stddef.h>
#include <string.h>

/* Function Definitions */
void solve(const d_struct_T *obj, emxArray_real_T *rhs)
{
  ptrdiff_t incx_t;
  ptrdiff_t lda_t;
  ptrdiff_t n_t;
  real_T *rhs_data;
  char_T DIAGA1;
  char_T TRANSA1;
  char_T UPLO1;
  rhs_data = rhs->data;
  if (obj->ndims >= 1) {
    DIAGA1 = 'N';
    TRANSA1 = 'T';
    UPLO1 = 'U';
    n_t = (ptrdiff_t)obj->ndims;
    lda_t = (ptrdiff_t)obj->ldm;
    incx_t = (ptrdiff_t)1;
    dtrsv(&UPLO1, &TRANSA1, &DIAGA1, &n_t, &obj->FMat->data[0], &lda_t,
          &rhs_data[0], &incx_t);
    DIAGA1 = 'N';
    TRANSA1 = 'N';
    UPLO1 = 'U';
    n_t = (ptrdiff_t)obj->ndims;
    lda_t = (ptrdiff_t)obj->ldm;
    incx_t = (ptrdiff_t)1;
    dtrsv(&UPLO1, &TRANSA1, &DIAGA1, &n_t, &obj->FMat->data[0], &lda_t,
          &rhs_data[0], &incx_t);
  }
}

/* End of code generation (solve.c) */
